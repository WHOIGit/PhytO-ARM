from datetime import datetime, timezone
import json
import re
import socket
import struct

import rospy
import pydantic
import serial

from typing import Any, Literal, Optional, Union, List, Tuple

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


class FramingConfig(pydantic.BaseModel):
    type: Literal['raw', 'delimited', 'json']
    pattern: Optional[re.Pattern] = None

    @pydantic.field_validator('pattern', pre=True)
    def compile_regex(cls, v):
        if isinstance(v, str):
            return re.compile(v.encode())
        return v

    @pydantic.model_validator(mode='after')
    def check_pattern(self) -> 'FramingConfig':
        if self.type == 'delimited' and not self.pattern:
            raise ValueError('pattern required for delimited framing')
        return self


class FieldSpecDelimited(pydantic.BaseModel):
    name: str
    index: int


class FieldSpecJson(pydantic.BaseModel):
    name: str
    path: str
    type: Literal['bool', 'bool[]', 'float', 'float[]', 'int', 'int[]', 'str']


class ExtractorConfig(pydantic.BaseModel):
    type: Literal['none', 'delimited', 'json'] = 'none'
    pattern: Optional[re.Pattern] = None
    fields: Optional[List[Union[FieldSpecDelimited, FieldSpecJson]]] = None

    @pydantic.field_validator('pattern', pre=True)
    def compile_regex(cls, v):
        if isinstance(v, str):
            return re.compile(v.encode())
        return v

    @pydantic.model_validator(mode='after')
    def check_fields(self) -> 'ExtractorConfig':
        if self.type in ('delimited', 'json') and not self.fields:
            raise ValueError('fields required for extractor')
        if self.type != 'delimited':
            self.pattern = None
        return self


class Config(pydantic.BaseModel):
    transport: Union[UDPConfig, SerialConfig]
    framing: FramingConfig
    extractor: ExtractorConfig = ExtractorConfig()


def is_multicast(addr: str) -> bool:
    o = socket.inet_aton(addr)
    return 224 <= o[0] <= 239


class UDPTransport:
    def __init__(self, cfg: UDPConfig):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        if is_multicast(cfg.rx_address):
            mreq = struct.pack('4sL', socket.inet_aton(cfg.rx_address), socket.INADDR_ANY)
            self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
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


class DsSerial:
    def __init__(self, cfg: SerialConfig):
        pmap = {'none': serial.PARITY_NONE, 'even': serial.PARITY_EVEN, 'odd': serial.PARITY_ODD}
        bmap = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}
        smap = {1: serial.STOPBITS_ONE, 2: serial.STOPBITS_TWO}
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
        self.first_byte_time: Optional[datetime.datetime] = None

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        raise NotImplementedError


class RawFramer(Framer):
    def __init__(self):
        super().__init__()

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        now = datetime.datetime.now(datetime.timezone.utc)
        if data and self.first_byte_time is None:
            self.first_byte_time = now
        out: List[Tuple[datetime.datetime, bytes]] = []
        if data:
            out.append((self.first_byte_time, data))
            self.first_byte_time = None
        return out


class DelimitedFramer(Framer):
    def __init__(self, pattern: re.Pattern):
        super().__init__()
        self.buf = bytearray()
        self.pattern = pattern

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        now = datetime.datetime.now(datetime.timezone.utc)
        was_empty = not self.buf
        if data and was_empty and self.first_byte_time is None:
            self.first_byte_time = now
        self.buf.extend(data)
        out: List[Tuple[datetime.datetime, bytes]] = []
        while True:
            m = self.pattern.search(self.buf)
            if not m:
                break
            end = m.end()
            out.append((self.first_byte_time, bytes(self.buf[:end])))
            del self.buf[:end]
            self.first_byte_time = now
        if not self.buf:
            self.first_byte_time = None
        return out


class JsonFramer(Framer):
    def __init__(self):
        super().__init__()
        self.buf = bytearray()
        self.decoder = json.JSONDecoder()

    def frame(self, data: bytes) -> List[Tuple[datetime.datetime, bytes]]:
        now = datetime.datetime.now(datetime.timezone.utc)
        was_empty = not self.buf
        if data and was_empty and self.first_byte_time is None:
            self.first_byte_time = now
        self.buf.extend(data)
        out: List[Tuple[datetime.datetime, bytes]] = []
        while True:
            try:
                _, idx = self.decoder.raw_decode(self.buf.decode('utf-8'))
                out.append((self.first_byte_time, bytes(self.buf[:idx])))
                del self.buf[:idx]
                self.first_byte_time = now
            except Exception:
                break
        if not self.buf:
            self.first_byte_time = None
        return out


class Extractor:
    def extract(self, pkt: bytes) -> dict:
        return {}


class DelimitedExtractor(Extractor):
    def __init__(self, pattern: re.Pattern, fields: List[FieldSpecDelimited]):
        self.pattern = pattern
        self.fields = [(f.name, f.index) for f in fields]

    def extract(self, pkt: bytes) -> dict:
        parts = self.pattern.split(pkt)
        return {n: parts[i] for n, i in self.fields if i < len(parts)}


class JsonExtractor(Extractor):
    def __init__(self, fields: List[FieldSpecJson]):
        self.fields = [(f.name, f.path) for f in fields]

    def extract(self, pkt: bytes) -> dict:
        obj = json.loads(pkt)
        res = {}
        for n, path in self.fields:
            cur = obj
            for key in path.lstrip('.').split('.'):
                cur = cur[int(key)] if key.isdigit() else cur.get(key)
            res[n] = cur
        return res


def main():
    rospy.init_node('io_node')
    cfg = Config.model_validate(rospy.get_param('~'))
    tr = UDPTransport(cfg.transport) if cfg.transport.type == 'UDP' else DsSerial(cfg.transport)
    if cfg.framing.type == 'raw':
        fr = RawFramer()
    elif cfg.framing.type == 'delimited':
        fr = DelimitedFramer(cfg.framing.pattern)
    else:
        fr = JsonFramer()
    if cfg.extractor.type == 'delimited':
        ex = DelimitedExtractor(cfg.extractor.pattern, cfg.extractor.fields or [])
    elif cfg.extractor.type == 'json':
        ex = JsonExtractor(cfg.extractor.fields or [])
    else:
        ex = Extractor()
    pub_raw = rospy.Publisher('~in', RawData, queue_size=10)
    field_specs = {f.name: f for f in (cfg.extractor.fields or [])}
    pub_fields = {}
    for name, spec in field_specs.items():
        topic = f'~in/{name}'
        if isinstance(spec, FieldSpecDelimited):
            msg_type = String
        else:
            msg_type = ROS_TYPE_MAP[spec.type]  # type: ignore
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
                if isinstance(spec, FieldSpecDelimited):
                    msg = String(str(val))
                else:
                    msg = convert_to_ros_msg(val, spec.type)  # type: ignore
                pub_fields[name].publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
