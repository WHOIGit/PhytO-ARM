import enum
import functools
import struct


def pack(pattern, x):
    '''Convenience function to pack into two unsigned 16-bit values'''
    return struct.unpack('<HH', struct.pack(f'<{pattern}', x))

def unpack(pattern, lo, hi):
    '''Convenience function to unpack from two unsigned 16-bit values'''
    values = struct.unpack(f'<{pattern}', struct.pack('<HH', lo, hi))
    return values[0] if len(pattern) == 1 else values


def explode_bits(x, n):
    return [ (x >> i) & 1 for i in range(n) ]

def implode_bits(bits):
    n = len(bits)
    return functools.reduce(lambda acc, bit: (bit << (n-1)) | (acc >> 1), bits)


class Register:
    def __init__(self, name, num, decode=None, encode=None):
        self.name, self.num = name, num
        self.decode = decode or functools.partial(unpack, 'L')
        self.encode = encode or functools.partial(pack, 'L')

    def __eq__(self, other):
        return self.num == other.num

    def __hash__(self):
        return hash(self.num)

    def __repr__(self):
        return f'{self.__class__.__name__}({repr(self.name)}, {self.num})'

    @property
    def addr(self):
        # This was formerly a separate instance variable that contained the list
        # of Modbus addresses that make up this register. But since all the
        # registers follow the same pattern, it was simpler to compute them.
        return (2*self.num, 2*self.num+1)


class MODE(enum.IntEnum):
    '''Modes for the MODE_REG register.'''
    # More modes are described in table 5.12.3 of the user manual.
    #
    # NOTE: If the motor gets into a mode which is not captured by this list,
    # the code is likely to throw an exception.
    PASSIVE = 0
    VELOCITY = 1
    POSITION = 2

    # Might briefly read this if a position limit is hit, before switching to
    # PASSIVE mode.
    STOP = 11


# Register numbers are taken from the user manual, listing 5.12.3. All registers
# are 32-bits, stored as two 16-bit half-words in big endian order.
all_registers = [
    # register 0 is reserved
    Register(name='PROG_VERSION',    num=1),
    Register(name='MODE_REG',        num=2,
        decode=lambda lo, hi: MODE(unpack('L', lo, hi)),
        encode=lambda x: pack('L', x.value),
    ),
    Register(name='P_SOLL',          num=3,
        decode=lambda lo, hi: unpack('l', lo, hi),  # encoder counts
        encode=lambda x: pack('l', x),
    ),
    Register(name='P_NEW',           num=4,
        decode=lambda lo, hi: unpack('l', lo, hi),  # see P_SOLL
        encode=lambda x: pack('l', x),
    ),
    Register(name='V_SOLL',          num=5,
        decode=lambda lo, hi: unpack('l', lo, hi) / 2.77056,  # RPM
        encode=lambda x: pack('l', int(x * 2.77056)),
    ),
    Register(name='A_SOLL',          num=6,
        decode=lambda lo, hi: unpack('l', lo, hi) / 3.598133e-3,  # RPM/s
        encode=lambda x: pack('l', int(x * 3.598133e-3)),
    ),
    Register(name='T_SOLL',          num=7,
        decode=lambda lo, hi: unpack('L', lo, hi) / 341,  # % of nominal load
        encode=lambda x: pack('L', int(x * 341)),
    ),
    Register(name='P_FNC',           num=8),
    Register(name='INDEX_OFFSET',    num=9),
    Register(name='P_IST',           num=10,
        decode=lambda lo, hi: unpack('l', lo, hi),  # encoder counts
        encode=lambda x: pack('l', x),
    ),
    Register(name='V_IST_16',        num=11,
        decode=lambda lo, hi: unpack('l', lo, hi) / (0.17316 * 16),  # RPM
        encode=lambda x: pack('l', int(x * (0.17316 * 16))),
    ),
    Register(name='V_IST',           num=12,
        decode=lambda lo, hi: unpack('l', lo, hi) / 0.17316,  # RPM
        encode=lambda x: pack('l', int(x * 0.17316)),
    ),
    Register(name='KVOUT',           num=13),
    Register(name='GEARF1',          num=14),
    Register(name='GEARF2',          num=15),
    Register(name='I2T',             num=16),
    Register(name='I2TLIM',          num=17),
    Register(name='UIT',             num=18),
    Register(name='UITLIM',          num=19),
    Register(name='FLWERR',          num=20),
    Register(name='U_24V',           num=21),
    Register(name='FLWERRMAX',       num=22),
    Register(name='UV_HANDLE',       num=23),
    Register(name='FNCERR',          num=24),
    Register(name='P_IST_TURNTAB',   num=25),
    Register(name='FNCERRMAX',       num=26),
    Register(name='TURNTAB_COUNT',   num=27),
    Register(name='MIN_P_IST',       num=28,
        decode=lambda lo, hi: unpack('l', lo, hi),  # encoder counts
        encode=lambda x: pack('l', x),
    ),
    Register(name='DEGC',            num=29),
    Register(name='MAX_P_IST',       num=30,
        decode=lambda lo, hi: unpack('l', lo, hi),  # encoder counts
        encode=lambda x: pack('l', x),
    ),
    Register(name='DEGCMAX',         num=31),
    Register(name='ACC_EMERG',       num=32),
    Register(name='INPOSWIN',        num=33),
    Register(name='INPOSCNT',        num=34),
    Register(name='ERR_STAT',        num=35,
        decode=lambda lo, hi: explode_bits(unpack('L', lo, hi), n=32),
        encode=lambda x: pack('L', implode_bits(x)),
    ),
    Register(name='CNTRL_BITS',      num=36),
    Register(name='START_MODE',      num=37),
    Register(name='P_HOME',          num=38),
    Register(name='HW_SETUP',        num=39),
    Register(name='V_HOME',          num=40),
    Register(name='T_HOME',          num=41),
    Register(name='HOME_MODE',       num=42),
    Register(name='P_REG_P',         num=43),
    Register(name='V_REG_P',         num=44),
    Register(name='A_REG_P',         num=45),
    Register(name='T_REG_P',         num=46),
    Register(name='L_REG_P',         num=47),
    Register(name='Z_REG_P',         num=48),
    Register(name='POS0',            num=49),
    Register(name='CAPCOM0',         num=50),
    Register(name='POS1',            num=51),
    Register(name='CAPCOM1',         num=52),
    Register(name='POS2',            num=53),
    Register(name='CAPCOM2',         num=54),
    Register(name='POS3',            num=55),
    Register(name='CAPCOM3',         num=56),
    Register(name='POS4',            num=57),
    Register(name='CAPCOM4',         num=58),
    Register(name='POS5',            num=59),
    Register(name='CAPCOM5',         num=60),
    Register(name='POS6',            num=61),
    Register(name='CAPCOM6',         num=62),
    Register(name='POS7',            num=63),
    Register(name='CAPCOM7',         num=64),
    Register(name='VEL0',            num=65),
    Register(name='VEL1',            num=66),
    Register(name='VEL2',            num=67),
    Register(name='VEL3',            num=68),
    Register(name='VEL4',            num=69),
    Register(name='VEL5',            num=70),
    Register(name='VEL6',            num=71),
    Register(name='VEL7',            num=72),
    Register(name='ACC0',            num=73),
    Register(name='ACC1',            num=74),
    Register(name='ACC2',            num=75),
    Register(name='ACC3',            num=76),
    Register(name='TQ0',             num=77),
    Register(name='TQ1',             num=78),
    Register(name='TQ2',             num=79),
    Register(name='TQ3',             num=80),
    Register(name='LOAD0',           num=81),
    Register(name='LOAD1',           num=82),
    Register(name='LOAD2',           num=83),
    Register(name='LOAD3',           num=84),
    Register(name='ZERO0',           num=85),
    Register(name='ZERO1',           num=86),
    Register(name='ZERO2',           num=87),
    Register(name='ZERO3',           num=88),
    Register(name='MODE0',           num=89),
    Register(name='MODE1',           num=90),
    Register(name='MODE2',           num=91),
    Register(name='MODE3',           num=92),
    Register(name='HWI0',            num=93),
    Register(name='HWI1',            num=94),
    Register(name='HWI2',            num=95),
    Register(name='HWI3',            num=96),
    Register(name='HWI4',            num=97),
    Register(name='HWI5',            num=98),
    Register(name='HWI6',            num=99),
    Register(name='HWI70',           num=100),
    Register(name='HWI80',           num=101),
    Register(name='HWI90',           num=102),
    Register(name='HWI100',          num=103),
    Register(name='HWI110',          num=104),
    Register(name='MAC00_TYP0',      num=105),
    Register(name='MAC00_10',        num=106),
    Register(name='MAC00_20',        num=107),
    Register(name='MAC00_30',        num=108),
    Register(name='MAC00_40',        num=109),
    Register(name='MAC00_51',        num=110),
    Register(name='MAC00_61',        num=111),
    Register(name='MAC00_71',        num=112),
    Register(name='MAC00_81',        num=113),
    Register(name='MAC00_91',        num=114),
    Register(name='MAC00_101',       num=115),
    Register(name='MAC00_111',       num=116),
    Register(name='MAC00_121',       num=117),
    Register(name='MAC00_131',       num=118),
    Register(name='MAC00_141',       num=119),
    Register(name='MAC00_152',       num=120),
    Register(name='KFF5',            num=121),
    Register(name='KFF4',            num=122),
    Register(name='KFF',             num=123),
    Register(name='KFF2',            num=124),
    Register(name='KFF1',            num=125),
    Register(name='KFF0',            num=126),
    Register(name='KVFX6',           num=127),
    Register(name='KVFX5',           num=128),
    Register(name='KVFX4',           num=129),
    Register(name='KVFX3',           num=130),
    Register(name='KVFX2',           num=131),
    Register(name='KVFX1',           num=132),
    Register(name='KVFY5',           num=133),
    Register(name='KVFY',            num=134),
    Register(name='KVFY3',           num=135),
    Register(name='KVFY2',           num=136),
    Register(name='KVFY1',           num=137),
    Register(name='KVFY',            num=138),
    Register(name='KVB4',            num=139),
    Register(name='KVB3',            num=140),
    Register(name='KVB2',            num=141),
    Register(name='KVB1',            num=142),
    Register(name='KVB0',            num=143),
    Register(name='KIFX2',           num=144),
    Register(name='KIFX1',           num=145),
    Register(name='KIFY1',           num=146),
    Register(name='KIFY0',           num=147),
    Register(name='KIB1',            num=148),
    Register(name='KIB0',            num=149),
    # registers 150-154 are reserved
    Register(name='ID_RESERVED',     num=155),
    Register(name='S_ORDER',         num=156),
    Register(name='OUTLOOPDIV',      num=157),
    Register(name='SAMPLE1',         num=158),
    Register(name='SAMPLE2',         num=159),
    Register(name='SAMPLE3',         num=160),
    Register(name='SAMPLE4',         num=161),
    Register(name='REC_CNT',         num=162),
    Register(name='V_EXT',           num=163),
    Register(name='GV_EXT',          num=164),
    Register(name='G_FNC',           num=165),
    Register(name='FNC_OUT',         num=166),
    Register(name='FF_OUT',          num=167),
    Register(name='VB_OUT',          num=168),
    Register(name='VF_OUT',          num=169),
    Register(name='ANINP',           num=170),
    Register(name='ANINP_OFFSET',    num=171),
    Register(name='ELDEG_OFFSET',    num=172),
    Register(name='PHASE_COMP',      num=173),
    Register(name='AMPLITUDE',       num=174),
    Register(name='MAN_I_NOM',       num=175),
    Register(name='MAN_ALPHA',       num=176),
    Register(name='UMEAS',           num=177),
    Register(name='I_NOM',           num=178),
    Register(name='PHI_SOLL',        num=179),
    Register(name='IA_SOLL',         num=180),
    Register(name='IB_SOLL',         num=181),
    Register(name='IC_SOLL',         num=182),
    Register(name='IA_IST',          num=183),
    Register(name='IB_IST',          num=184),
    Register(name='IC_IST',          num=185),
    Register(name='IA_OFFSET',       num=186),
    Register(name='IB_OFFSET',       num=187),
    Register(name='KIA',             num=188),
    Register(name='KIB',             num=189),
    Register(name='ELDEG_IST',       num=190),
    Register(name='V_ELDEG',         num=191),
    Register(name='UA_VAL',          num=192),
    Register(name='UB_VAL',          num=193),
    Register(name='UC_VAL',          num=194),
    Register(name='EMK_A',           num=195),
    Register(name='EMK_B',           num=196),
    Register(name='EMK_C',           num=197),
    Register(name='U_BUS',           num=198,
        decode=lambda lo, hi: unpack('L', lo, hi) * 0.888,  # V
        encode=lambda x: pack('L', int(x / 0.888)),
    ),
    Register(name='U_BUS_OFFSET',    num=199),
    Register(name='TC0_CV1',         num=200),
    Register(name='TC0_CV2',         num=201),
    Register(name='MY_ADDR',         num=202),
    Register(name='MOTOR_TYPE',      num=203),
    Register(name='SERIAL_NUMBER',   num=204),
    Register(name='HW_VERSION',      num=205),
    Register(name='CHKSUM',          num=206),
    Register(name='USEROUTVAL',      num=207),
    Register(name='COMM_ERRS',       num=208),
    Register(name='INDEX_IST',       num=209),
    Register(name='HW_PLIM',         num=210),
    Register(name='COMMAND_REG',     num=211),
    Register(name='UART0_SETUP',     num=212),
    Register(name='UART1_SETUP',     num=213),
    Register(name='EXTENC_BITS',     num=214),
    Register(name='INPUT_LEVELS',    num=215),
    Register(name='ANINP1',          num=216),
    Register(name='ANINP1_OFFSET',   num=217),
    Register(name='ANINP2',          num=218),
    Register(name='ANINP2_OFFSET',   num=219),
    Register(name='ANINP3',          num=220),
    Register(name='ANINP3_OFFSET',   num=221),
    Register(name='IOSETUP',         num=222),
    Register(name='ANOUT1',          num=223),
    Register(name='ANOUT1_OFFSET',   num=224),
    Register(name='P_OFFSET',        num=225),
    Register(name='P_MULTITURN',     num=226),
    Register(name='AIFILT_MAXSLOPE', num=227),
    Register(name='AIFILT_FILTFACT', num=228),
    Register(name='P_QUICK',         num=229),
    Register(name='XREG_ADDR',       num=230),
    Register(name='XREG_DATA',       num=231),
]

# NOTE:
# Additional modbus module registers are defined in listing 7.3.1 of the
# MAC00-EC4/-EC41 user manual.


# Create lookup table by name
_registers_by_name = { reg.name: reg for reg in all_registers }


# Export all registers
globals().update(_registers_by_name)


# Look up by name
def register_for_name(name):
    return _registers_by_name[name]


# Look up by address
def register_for_address(addr):
    lo, hi = 0, len(all_registers)
    while lo < hi:
        mid = (lo+hi)//2
        if all_registers[mid].addr[-1] < addr:
            lo = mid+1
        else:
            hi = mid

    if addr not in all_registers[lo].addr:
        raise ValueError('Register not found for this address')
    return all_registers[lo]


__all__ = list(_registers_by_name.keys()) + [
    all_registers,
    register_for_name,
    register_for_address,
]
