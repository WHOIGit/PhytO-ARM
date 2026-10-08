import datetime
import json
import re
import socket
import struct
import typing

import rospy
import pydantic
import serial

from typing import Any, Dict, Literal, Optional, Union, List, Tuple

from ds_core_msgs.msg import RawData
from std_msgs.msg import (Bool, Float64, Float64MultiArray, Int64,
                          Int64MultiArray, Int8MultiArray, String)


# Type mapping for ROS message types
ROS_TYPE_MAP = {
    'bool': Bool,
    'bool[]': Int8MultiArray,
    'float': Float64,
    'float[]': Float64MultiArray,
    'int': Int64,
    'int[]': Int64MultiArray,
    'str': String,
}

class ConversionError(Exception):
    """Error converting a Python value to a ROS message."""
    pass

def convert_to_ros_msg(value: Any, type_hint: str) -> Any:
    try:
        msg_type = ROS_TYPE_MAP[type_hint]
        if type_hint == 'float' and not isinstance(value, (int, float)):
            raise ConversionError(f'Expected float, got {type(value)}')
        if type_hint == 'int' and not isinstance(value, int):
            raise ConversionError(f'Expected int, got {type(value)}')
        if type_hint == 'bool' and not isinstance(value, bool):
            raise ConversionError(f'Expected bool, got {type(value)}')
        if type_hint == 'str' and not isinstance(value, str):
            raise ConversionError(f'Expected str, got {type(value)}')
        if type_hint.endswith('[]') and not isinstance(value, (list, tuple)):
            raise ConversionError(f'Expected list for {type_hint}, got {type(value)}')

        if type_hint == 'bool[]':
            msg = msg_type(data=[int(x) for x in value])
        elif type_hint.endswith('[]'):
            msg = msg_type(data=value)
        else:
            msg = msg_type(data=value)
        return msg
    except KeyError:
        raise ConversionError(f'Unknown type hint: {type_hint}')
    except (ValueError, TypeError) as e:
        raise ConversionError(f'Error converting to {type_hint}: {e}')


# Configuration models for transports (UDP and serial)
class UDPConfig(pydantic.BaseModel):
    type: Literal['UDP']
    rx_port: int
    tx_port: int
    rx_address: str = '0.0.0.0'
    tx_address: Optional[str] = None

class SerialConfig(pydantic.BaseModel):
    type: Literal['Serial']
    port: str
    baud: int = 9600
    data_bits: int = 8
    stop_bits: Literal[1, 2] = 1
    parity: Literal['none', 'even', 'odd'] = 'none'

TransportConfig = Union[UDPConfig, SerialConfig]


# Configuration models for framing
class RawFramingConfig(pydantic.BaseModel):
    type: Literal['raw'] = 'raw'

class DelimitedFramingConfig(pydantic.BaseModel):
    type: Literal['delimited']
    pattern: re.Pattern

    @pydantic.field_validator('pattern', mode='after')
    @classmethod
    def check_pattern(cls, pattern: re.Pattern) -> re.Pattern:
        if pattern.match(b''):
            raise ValueError('Pattern must not match empty string')
        return pattern

    @pydantic.field_validator('pattern', mode='before')
    @classmethod
    def compile_regex(cls, value: Union[str, re.Pattern]) -> re.Pattern:
        if isinstance(value, str):
            return re.compile(value.encode())
        return value

class JsonFramingConfig(pydantic.BaseModel):
    type: Literal['json']

FramingConfig = RawFramingConfig | DelimitedFramingConfig | JsonFramingConfig


# Configuration models for extractors
class FieldSpec(pydantic.BaseModel):
    name: str
    type: Literal['bool', 'bool[]', 'float', 'float[]', 'int', 'int[]', 'str']

    @property
    def is_array_type(self) -> bool:
        return self.type.endswith('[]')

    # The selector represents a dot-separated path to the field in the data
    # structure. This is loosely based on JSONPath or jq syntax.
    #
    # At validation time, we parse the selector into a list of indices which
    # can be used by iteratively subscripting into the object.
    #
    # Examples:
    #   - [field] -> ['field']  # bracket syntax
    #   - .field -> ['field']  # dot syntax
    #   - .field.subfield -> ['field', 'subfield']  # nested field
    #   - field[0] -> ['field', 0]  # numeric index bracket syntax
    #   - field.0 -> ['field', 0]  # numeric index dot syntax
    #   - field -> ['field']  # leading dot is optional
    #   - $.field -> ['field']  # leading $ (from JSONPath) is optional
    #
    # Not supported yet:
    #   - .escaped\.char -> ['escaped.char']  # escaped characters
    #   - .\"field with spaces\" -> ['field with spaces']  # quoted field
    #   - .field['0'] -> ['field', '0']  # numeric string
    selector: str

    @pydantic.field_validator('selector', mode='after')
    @classmethod
    def validate_selector(cls, selector: str) -> str:
        # Normalize the selector to ensure it starts with a dot or bracket
        if selector.startswith('$.'):
            selector = selector[1:]  # keep leading dot
        elif selector and selector[0] not in ('.', '['):
            selector = f'.{selector}'
    
        # Run through the path logic to make sure it works
        cls._parse_selector(selector)

        return selector

    @classmethod
    def _parse_selector(cls, selector: str) -> List[Union[str, int]]:
        token_pattern = re.compile(r'''
            (?:\.(\w+))      # .field or .0
            | (?:\[(\w+)\])  # [field] or [0]
        ''', re.VERBOSE)

        # Pop tokens off the selector into the path
        tokens = []
        while m := token_pattern.match(selector):
            field = m.group(1) or m.group(2)
            tokens.append(int(field) if field.isdigit() else field)
            selector = selector[m.end():]

        if selector:
            raise ValueError(f'Unable to parse selector: {selector}')

        return tokens

    @property
    def path(self) -> List[Union[str, int]]:
        return self._parse_selector(self.selector)


# Check that the ROS_TYPE_MAP keys match the type hints in FieldSpec
assert set(ROS_TYPE_MAP.keys()) == \
       set(typing.get_args(typing.get_type_hints(FieldSpec)['type']))


class DelimitedExtractorConfig(pydantic.BaseModel):
    type: Literal['delimited']
    pattern: re.Pattern
    fields: List[FieldSpec]

    @pydantic.field_validator('pattern', mode='before')
    @classmethod
    def compile_regex(cls, value: Union[str, re.Pattern]) -> re.Pattern:
        if isinstance(value, str):
            return re.compile(value.encode())
        return value

    @pydantic.field_validator('pattern', mode='after')
    @classmethod
    def check_pattern(cls, pattern: re.Pattern) -> re.Pattern:
        if pattern.match(b''):
            raise ValueError('Pattern must not match empty string')
        return pattern

    @pydantic.model_validator(mode='after')
    def check_field_indices(self) -> 'DelimitedExtractorConfig':
        for f in self.fields:
            if len(f.path) != 1 or not isinstance(f.path[0], int):
                raise ValueError(
                    f"Delimited extractor requires one numeric index; got {f.path} from '{f.selector}'"
                )
            if f.is_array_type:
                raise ValueError(f"Array type '{f.type}' not supported for delimited extractor")
        return self


class DelimitedFramingConfig(pydantic.BaseModel):
    type: Literal['delimited']
    pattern: re.Pattern

    @pydantic.field_validator('pattern', mode='after')
    @classmethod
    def check_pattern(cls, pattern: re.Pattern) -> re.Pattern:
        if pattern.match(b''):
            raise ValueError('Pattern must not match empty string')
        return pattern

    @pydantic.field_validator('pattern', mode='before')
    @classmethod
    def compile_regex(cls, value: Union[str, re.Pattern]) -> re.Pattern:
        if isinstance(value, str):
            return re.compile(value.encode())
        return value

class JsonExtractorConfig(pydantic.BaseModel):
    type: Literal['json']
    fields: List[FieldSpec]

class NoExtractorConfig(pydantic.BaseModel):
    type: Literal['none'] = 'none'

ExtractorConfig = (DelimitedExtractorConfig | JsonExtractorConfig |
                  NoExtractorConfig)


# Root configuration model
class Config(pydantic.BaseModel):
    transport: TransportConfig
    framing: FramingConfig = pydantic.Field(default_factory=RawFramingConfig)
    extractor: ExtractorConfig = pydantic.Field(default_factory=NoExtractorConfig)

    @pydantic.model_validator(mode='after')
    def check_serial_framing(self) -> 'Config':
        # It doesn't make sense to use raw framing with a serial device, because
        if (isinstance(self.transport, SerialConfig) and
            isinstance(self.framing, RawFramingConfig)):
            raise ValueError('Raw framing not supported for serial transport, '
                             'try delimited framing instead')
        return self




def is_multicast(addr: str) -> bool:
    o = socket.inet_aton(addr)
    return 224 <= o[0] <= 239


class Transport:
    def read(self) -> bytes:
        raise NotImplementedError
    
    def write(self, data: bytes) -> None:
        raise NotImplementedError


class UDPTransport(Transport):
    def __init__(self, cfg: UDPConfig):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        if is_multicast(cfg.rx_address):
            self.rx.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP, 
                struct.pack('4sL', socket.inet_aton(cfg.rx_address),
                            socket.INADDR_ANY)
            )
        self.rx.bind((cfg.rx_address, cfg.rx_port))
        self.rx.settimeout(0.1)

        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = cfg.tx_address or cfg.rx_address
        self.tx.connect((addr, cfg.tx_port))

    def read(self) -> bytes:
        try:
            data, _ = self.rx.recvfrom(65536)
        except socket.timeout:
            return b''
        return data

    def write(self, data: bytes) -> None:
        self.tx.send(data)


class SerialTransport(Transport):
    def __init__(self, cfg: SerialConfig):
        pmap = {
            'none': serial.PARITY_NONE,
            'even': serial.PARITY_EVEN,
            'odd': serial.PARITY_ODD,
        }
        bmap = {
            5: serial.FIVEBITS,
            6: serial.SIXBITS,
            7: serial.SEVENBITS,
            8: serial.EIGHTBITS,
        }
        smap = {
            1: serial.STOPBITS_ONE,
            2: serial.STOPBITS_TWO,
        }
        self.ser = serial.Serial(
            port=cfg.port,
            baudrate=cfg.baud,
            bytesize=bmap[cfg.data_bits],
            parity=pmap[cfg.parity],
            stopbits=smap[cfg.stop_bits],
            timeout=0.1,
        )

    def read(self) -> bytes:
        return self.ser.read(self.ser.in_waiting or 1)

    def write(self, data: bytes) -> None:
        self.ser.write(data)


class Framer:
    def __init__(self):
        self.timestamp: Optional[datetime.datetime] = None

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        raise NotImplementedError


class RawFramer(Framer):
    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        return ([(datetime.datetime.now(datetime.timezone.utc), data)]
                if data else [])


class DelimitedFramer(Framer):
    def __init__(self, pattern: re.Pattern):
        super().__init__()
        self.pattern = pattern
        self.buffer = bytearray()

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        if not data:
            return []

        # Append data to the buffer. If the buffer was empty, record the
        # timestamp for the DsHeader.io_time field.
        now = datetime.datetime.now(datetime.timezone.utc)
        if not self.buffer:
            self.timestamp = now
        self.buffer.extend(data)

        # Pop delimited frames from the buffer
        out: List[Tuple[datetime.datetime, bytes]] = []
        while m := self.pattern.search(self.buffer):
            out.append((
                typing.cast(datetime.datetime, self.timestamp),
                bytes(self.buffer[:m.end()])
            ))
            del self.buffer[:m.end()]
            self.timestamp = now
        return out


class JsonFramer(Framer):
    def __init__(self):
        super().__init__()
        self.buffer = bytearray()
        self.decoder = json.JSONDecoder()

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        if not data:
            return []
        
        # Append data to the buffer. If the buffer was empty, record the
        # timestamp for the DsHeader.io_time field.
        now = datetime.datetime.now(datetime.timezone.utc)
        if not self.buffer:
            self.timestamp = now
        self.buffer.extend(data)

        # Pop JSON objects from the buffer
        out: List[Tuple[datetime.datetime, bytes]] = []
        while True:
            try:
                _, idx = self.decoder.raw_decode(self.buffer.decode())
                out.append((
                    typing.cast(datetime.datetime, self.timestamp),
                    bytes(self.buffer[:idx])
                ))
                del self.buffer[:idx]
                self.timestamp = now
            except json.JSONDecodeError:
                break
        return out


class Extractor:
    def extract(self, pkt: bytes) -> dict:
        raise NotImplementedError


class DelimitedExtractor(Extractor):
    def __init__(self, pattern: re.Pattern, fields: List[FieldSpec]):
        self.pattern = pattern
        self.fields = [(f.name, f.path[0]) for f in fields]

    def extract(self, pkt: bytes) -> Dict[str, str]:
        parts = self.pattern.split(pkt)
        result: Dict[str, str] = {}
        for n, idx in self.fields:
            if 0 <= idx < len(parts):
                # TODO: Cast to correct type
                result[n] = parts[idx]
        return result


class JsonExtractor(Extractor):
    def __init__(self, fields: List[FieldSpec]):
        self.fields = [(f.name, f.path, f.type) for f in fields]

    def extract(self, pkt: bytes) -> Dict[str, Any]:
        obj = json.loads(pkt)
        res: Dict[str, Any] = {}
        for n, path, _ in self.fields:
            cur = obj
            for step in path:
                cur = cur[step] if isinstance(step, int) else cur.get(step)
            res[n] = cur
        return res


def main():
    rospy.init_node('io_node')
    cfg = Config.model_validate(rospy.get_param('~'))
    tr = UDPTransport(cfg.transport) if cfg.transport.type == 'UDP' else SerialTransport(cfg.transport)
    if isinstance(cfg.framing, DelimitedFramingConfig):
        fr = DelimitedFramer(cfg.framing.pattern)
    elif isinstance(cfg.framing, JsonFramingConfig):
        fr = JsonFramer()
    else:
        fr = RawFramer()
    if isinstance(cfg.extractor, DelimitedExtractorConfig):
        ex = DelimitedExtractor(cfg.extractor.pattern, cfg.extractor.fields)
    elif isinstance(cfg.extractor, JsonExtractorConfig):
        ex = JsonExtractor(cfg.extractor.fields)
    else:
        ex = Extractor()
    pub_raw = rospy.Publisher('~in', RawData, queue_size=10)
    field_specs = {f.name: f for f in (cfg.extractor.fields or [])}
    pub_fields = {}
    for name, spec in field_specs.items():
        topic = f'~in/{name}'
        if isinstance(cfg.extractor, DelimitedExtractorConfig):
            msg_type = String
        else:
            msg_type = ROS_TYPE_MAP[spec.type]
        pub_fields[name] = rospy.Publisher(topic, msg_type, queue_size=10)

    def on_out(msg: RawData):
        tr.write(bytes(msg.data))

    rospy.Subscriber('~out', RawData, on_out)

    while not rospy.is_shutdown():
        data = tr.read()
        if not data:
            continue
        now_stamp = rospy.Time.now()
        for ts_dt, pkt in fr.frame(data):
            m = RawData()
            m.data = list(pkt)
            m.data_direction = RawData.DATA_IN
            m.ds_header.io_time = rospy.Time.from_sec(ts_dt.timestamp())
            m.header.stamp = now_stamp
            pub_raw.publish(m)
            for name, val in ex.extract(pkt).items():
                spec = field_specs.get(name)
                if isinstance(cfg.extractor, DelimitedExtractorConfig):
                    msg = String(str(val))
                else:
                    msg = convert_to_ros_msg(val, spec.type)
                pub_fields[name].publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
