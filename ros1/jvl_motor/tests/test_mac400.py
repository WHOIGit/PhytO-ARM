import enum
import os
import struct
import sys
import unittest

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import mac400


class TestPackUnpack(unittest.TestCase):
    def test_pack_expected(self):
        x = 0x12345678
        lo, hi = mac400.pack('L', x)
        self.assertEqual(x, (hi << 16 | lo))

    def test_pack_unpack_unsigned(self):
        x = 0x12345678
        lo, hi = mac400.pack('L', x)
        y = mac400.unpack('L', lo, hi)
        self.assertEqual(x, y)

    def test_pack_unpack_signed(self):
        x = -0x12345678
        lo, hi = mac400.pack('l', x)
        y = mac400.unpack('l', lo, hi)
        self.assertEqual(x, y)


class TestBitsExplodeImplode(unittest.TestCase):
    INPUTS = [0, 1, 0x12345678, 0x80000000, 0xFFFFFFFF]

    def test_explode_bits(self):
        for x in self.INPUTS:
            bits = [ int(b) for b in bin(x)[2:].zfill(32) ]
            bits.reverse()  # least-significant bit first
            self.assertEqual(mac400.explode_bits(x, n=32), bits)

    def test_implode_bits(self):
        for x in self.INPUTS:
            bits = [ int(b) for b in bin(x)[2:].zfill(32) ]
            bits.reverse()  # least-significant bit first
            self.assertEqual(mac400.implode_bits(bits), x)

    def test_explode_implode_bits(self):
        for x in self.INPUTS:
            y = mac400.implode_bits(mac400.explode_bits(x, n=32))
            self.assertEqual(x, y)


class TestRegisterClass(unittest.TestCase):
    def test_addr(self):
        # The register number maps to two consecutive modbus addresses
        reg = mac400.Register('TEST', num=5)
        self.assertEqual(reg.addr, (10, 11))

    def test_hash(self):
        # Registers with the same num should be equal and hash-identical
        r1 = mac400.Register('FOO', num=5)
        r2 = mac400.Register('BAR', num=5)
        self.assertEqual(r1, r2)
        self.assertEqual(hash(r1), hash(r2))

        # Using them as dict keys should collapse to a single entry
        d = {r1: 1}
        d[r2] = 2
        self.assertEqual(len(d), 1)
        self.assertEqual(d[r1], 2)


class TestRegisters(unittest.TestCase):
    def test_register_globals(self):
        for reg in mac400.all_registers:
            self.assertTrue(hasattr(mac400, reg.name))
            self.assertIs(getattr(mac400, reg.name), reg)

    def test_register_by_name(self):
        for reg in mac400.all_registers:
            self.assertIs(mac400.register_for_name(reg.name), reg)

    def test_register_by_address(self):
        # This function looks up a register by one of the two addresses the
        # register occupies.
        for reg in mac400.all_registers:
            self.assertIs(mac400.register_for_address(reg.addr[0]), reg)
            self.assertIs(mac400.register_for_address(reg.addr[1]), reg)

    def test_registers_sorted_by_address(self):
        # Required for register_for_address() to work
        addrs = [reg.addr for reg in mac400.all_registers]
        self.assertEqual(addrs, sorted(addrs))

    def test_registers_unique(self):
        # Look for duplicated register names and numbers
        names = [reg.name for reg in mac400.all_registers]
        duped_names = [n for n in names if names.count(n) > 1]
        self.assertEqual(len(duped_names), 0,
                         f'Duplicate register names: {duped_names}')

        nums = [reg.num for reg in mac400.all_registers]
        duped_nums = [n for n in nums if nums.count(n) > 1]
        self.assertEqual(len(duped_nums), 0,
                         f'Duplicate register numbers: {duped_nums}')

    def test_register_codecs(self):
        for reg in mac400.all_registers:
            # Ignore registers that do not override encode/decode
            if hasattr(reg.encode, 'func'):
                continue

            # Figure out the type of value returned by decode(). This is a bit
            # of a hack because we don't know if (0, 0) is a valid encoding.
            nilvalue = reg.decode(0, 0)
            if isinstance(nilvalue, enum.Enum):
                for v in type(nilvalue):
                    self.assertEqual(v, reg.decode(*reg.encode(v)), reg.name)

            elif isinstance(nilvalue, (int, float)):
                tests = []
                for lo in (0, 1, 2**8, 2**16 - 1):
                    for hi in (0, 1, 2**8, 2**16 - 1):
                        tests.append((lo, hi))

                for lo, hi in tests:
                    lo2, hi2 = reg.encode(reg.decode(lo, hi))

                    if isinstance(nilvalue, float):
                        # Allow a small quantization error in the least
                        # significant bit for floats.
                        packed = (hi << 16) | lo
                        repacked = (hi2 << 16) | lo2
                        delta = abs(packed - repacked)
                        if delta == (2**32 - 1):
                            delta = 1  # wrap-around case
                        self.assertLessEqual(delta, 1, reg.name)
                    else:
                        self.assertEqual((lo, hi), (lo2, hi2), reg.name)

            # Special case: ERR_STAT is a list of flags currently as bits
            elif reg.name == 'ERR_STAT':
                self.assertIsInstance(nilvalue, list)
                self.assertTrue(all(x in [0, 1] for x in nilvalue))

            else:
                self.fail(f'Unknown register type: {type(nilvalue)} '
                          f'for {reg.name}')


if __name__ == '__main__':
    unittest.main()
