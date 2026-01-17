#!/usr/bin/env python3
import socketserver
import struct
import threading

import rospy

from jvl_motor import mac400
from jvl_motor.mac400_model import MotorModel
from pyModbusTCP import constants as modbus
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


FEEDBACK = (mac400.ERR_STAT, mac400.MODE_REG, mac400.P_IST, mac400.V_IST_16)

# The driver reads MODE_REG through ERR_STAT together and U_BUS separately
READ_WINDOWS = (
    (mac400.MODE_REG.addr[0], mac400.ERR_STAT.addr[-1]),
    mac400.U_BUS.addr,
)

# Setpoints retain the exact words written by the driver
SETPOINTS = (
    mac400.A_SOLL, mac400.MAX_P_IST, mac400.MIN_P_IST, mac400.P_NEW,
    mac400.P_SOLL, mac400.T_SOLL, mac400.V_SOLL,
)

# Report nominal voltages and zero following error. U_24V is a 12-bit ADC
# reading with a 55 V full scale.
STATIC_REGISTERS = {
    mac400.FLWERR: 0,
    mac400.U_24V: round(24 * 4095 / 55),
    mac400.U_BUS: 325.0,
}

# Modbus/TCP frames begin with transaction, protocol, length and unit ID
TCP_HEADER = struct.Struct('>HHHB')

# MODE_REG and P_IST can also be written, but reads report the live drive state
WRITABLE = {
    reg.addr[0]: reg for reg in (*SETPOINTS, mac400.MODE_REG, mac400.P_IST)
}


# Serialize clock updates and complete Modbus transactions
class MotorRegisters:
    def __init__(self, model, use_sim_time=False):
        self.lock = threading.Lock()
        self.model = model
        self.use_sim_time = use_sim_time

        # Static readings and commanded setpoints are stored as Modbus words
        initial = {reg: model.read(reg) for reg in SETPOINTS}
        initial.update(STATIC_REGISTERS)
        self.stored_words = {}
        for reg, value in initial.items():
            self.stored_words.update(zip(reg.addr, reg.encode(value)))

        # Process clock rewinds even when there are no Modbus requests
        if use_sim_time:
            rospy.Subscriber('/clock', Clock, self.on_clock)

            # Include any tick rospy received before our callback was attached
            with self.lock:
                self.model.advance(rospy.Time.now().to_nsec())

    def on_clock(self, message):
        with self.lock:
            self.model.advance(message.clock.to_nsec())

    # In wall time, advance to the request before reading or applying a command
    def advance_for_request(self):
        if not self.use_sim_time:
            self.model.advance(rospy.Time.now().to_nsec())

    def shaft_position(self):
        with self.lock:
            self.advance_for_request()
            return self.model.shaft_position

    def feedback_words(self, reg):
        value = self.model.read(reg)

        # Round the measured RPM once to the nearest signed hardware unit
        if reg == mac400.V_IST_16:
            return mac400.pack('l', round(value * mac400.RPM_SCALE))
        return reg.encode(value)

    def read_words(self, address, count):
        last = address + count - 1
        if not any(lo <= address <= last <= hi for lo, hi in READ_WINDOWS):
            raise KeyError(address)

        # Sample live feedback at one instant and zero-fill unused addresses
        with self.lock:
            self.advance_for_request()
            words = self.stored_words.copy()
            for reg in FEEDBACK:
                words.update(zip(reg.addr, self.feedback_words(reg)))
            return [words.get(addr, 0) for addr in range(address, last + 1)]

    def write_words(self, address, words):
        if address % 2 or len(words) % 2:
            raise ValueError('A MAC400 register requires two aligned words')

        # Validate every register before committing any part of the command
        decoded = {}
        raw_setpoints = {}
        for offset in range(0, len(words), 2):
            reg = WRITABLE[address + offset]
            raw = words[offset:offset + 2]
            decoded[reg] = reg.decode(*raw)
            if reg in SETPOINTS:
                raw_setpoints.update(zip(reg.addr, raw))

        # Finish the old motion before applying the command and its readback
        with self.lock:
            self.advance_for_request()
            self.model.write(decoded)
            self.stored_words.update(raw_setpoints)


# Serve the driver's Modbus register reads and writes
def respond(registers, request):
    function = request[0]
    try:
        if function == modbus.READ_HOLDING_REGISTERS:
            address, count = struct.unpack('>HH', request[1:])
            if not 1 <= count <= 125:
                raise ValueError('Invalid read quantity')
            words = registers.read_words(address, count)
            payload = struct.pack(f'>{count}H', *words)
            return bytes((function, len(payload))) + payload

        # A successful write echoes the function, starting address and count
        if function == modbus.WRITE_MULTIPLE_REGISTERS:
            address, count, size = struct.unpack('>HHB', request[1:6])
            if not 1 <= count <= 123 or size != 2 * count:
                raise ValueError('Invalid write quantity')
            words = struct.unpack(f'>{count}H', request[6:])
            registers.write_words(address, words)
            return request[:5]

        error = modbus.EXP_ILLEGAL_FUNCTION
    except KeyError:
        error = modbus.EXP_DATA_ADDRESS
    except (ValueError, struct.error):
        error = modbus.EXP_DATA_VALUE

    # Modbus exception replies set the high bit of the request's function code
    return bytes((function | 0x80, error))


# Match each reply to its request using the Modbus/TCP header
class ModbusConnection(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            while True:
                header = self.rfile.read(TCP_HEADER.size)
                if len(header) != TCP_HEADER.size:
                    return
                transaction, protocol, length, unit = TCP_HEADER.unpack(header)

                # Length includes the unit byte already consumed and up to
                # 253 bytes of Modbus request data.
                if protocol != 0 or not 2 <= length <= 254:
                    return
                request = self.rfile.read(length - 1)
                if len(request) != length - 1:
                    return

                # Echo the transaction and unit identifiers in the reply
                reply = respond(self.server.registers, request)
                header = TCP_HEADER.pack(transaction, protocol, len(reply) + 1,
                                         unit)
                self.wfile.write(header + reply)
        except ConnectionError:
            return


class MotorServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    # Read the endpoint and encoder configuration from the node's namespace
    rospy.init_node('mac400_sim')
    address = rospy.get_param('~address', '127.0.0.1')
    counts_per_turn = rospy.get_param('~counts_per_turn', 8192)
    port = rospy.get_param('~port', 1502)

    # Use the same clock source as the production driver
    use_sim_time = rospy.get_param('/use_sim_time', False)
    registers = MotorRegisters(MotorModel(counts_per_turn), use_sim_time)

    # Publish mechanical travel even when the encoder has been re-zeroed
    shaft = rospy.Publisher('~shaft_position', Float64, queue_size=1)
    rospy.Timer(rospy.Duration(0.05),
                lambda event: shaft.publish(registers.shaft_position()),
                reset=True)

    # A wall-time socket timeout allows shutdown while ROS time is paused
    with MotorServer((address, port), ModbusConnection) as server:
        server.registers = registers
        server.timeout = 0.25
        rospy.loginfo('MAC400 simulator listening on %s:%s', address, port)
        while not rospy.is_shutdown():
            server.handle_request()


if __name__ == '__main__':
    main()
