import datetime
import os
import sys
import unittest

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from aml_ctd.amlxparser import parseAMLx, iterAMLx, Measurement


class TestIterAMLx(unittest.TestCase):
    def test_basic(self):
        s = 'msg1{a[data=x,1,u]}'
        expected = [(1, 'a', 'data', 'x', '1', 'u')]
        self.assertEqual(list(iterAMLx(s)), expected)

    def test_no_msg_prefix(self):
        with self.assertRaises(ValueError):
            list(iterAMLx('invalid'))

    def test_checksum(self):
        # Long string captured from a real instrument
        sample = (
            'msg226{'
            'mux[meta=time,1748877153.19,s]'
            '[data=uv,1],'
            'port1[data=Cond,0.000000,mS/cm]'
            '[rawi=ADC,557,none]'
            '[data=TempCT,26.409889,C]'
            '[rawi=ADC,441415,none],'
            'port2[data=Chloro-blue,0.459863,ug/L]'
            '[rawi=ADC,75724,2sComp],'
            'port3[data=DO,285.149994,umol/l]'
            '[data=TempDO,27.024000,C],'
            'port4[data=Phycoerythrin,0.707371,ppb]'
            '[rawi=ADC,28688,2sComp],'
            'port5[data=Turbidity,28.260294,NTU]'
            '[rawi=ADC,60042,2sComp],'
            'port6[data=Pressure,0.032359,dBar]'
            '[rawi=ADC,932854,2sComp],'
            'port7[data=PAR,0.000776,Volts]'
            '[rawi=ADC,2053,2sComp],'
            'derive[data=Dens,-99.989998,kg/m3]'
            '[data=Depth,0.032181,m]'
            '[data=SalC,-99.989998,psu]'
            '[data=SV,-99.989998,m/s]'
            '}*4E'
        )
        list(iterAMLx(sample))

        with self.assertRaises(ValueError):
            list(iterAMLx(sample[:-1] + 'F'))


class TestParseAMLx(unittest.TestCase):
    def test_parse_msgnum(self):
        result = parseAMLx('msg128{port1[data=x,1,u][rawi=ADC,1,none]}')
        self.assertEqual(result['msgnum'], 128)

    def test_parse_timestamp(self):
        result = parseAMLx('msg128{mux[meta=time,1590605500.55,s]}')
        expected = datetime.datetime(2020, 5, 27, 18, 51, 40, 550000,
                                     tzinfo=datetime.timezone.utc)
        self.assertEqual(result['mux']['time'], expected)

    def test_parse_uv(self):
        # uv doesn't have a unit
        result = parseAMLx('msg128{mux[data=uv,1]}')
        self.assertIsInstance(result['mux']['uv'], bool)
        self.assertTrue(result['mux']['uv'])

    def test_measurement_rawi(self):
        result = parseAMLx(
            'msg128{'
                'port1'
                '[data=TempCT,23.881313,C]'
                '[rawi=ADC,428710,none]'
            '}'
        )

        temp = result['port1']['TempCT']
        self.assertAlmostEqual(temp.value, 23.881313)
        self.assertEqual(temp.unit, 'C')
        self.assertEqual(temp.rawname, 'ADC')
        self.assertEqual(temp.rawvalue, 428710)

    def test_measurement_rawf(self):
        result = parseAMLx(
            'msg128{'
                'port3'
                '[data=SV,0.000000,m/s]'
                '[rawf=NSV,0.000000,samples]'
            '}'
        )

        temp = result['port3']['SV']
        self.assertAlmostEqual(temp.value, 0.000000)
        self.assertEqual(temp.unit, 'm/s')
        self.assertEqual(temp.rawname, 'NSV')
        self.assertAlmostEqual(temp.rawvalue, 0.000000)

    def test_complete_packet(self):
        result = parseAMLx(
            'msg138{'
                'mux'
                '[meta=time,1590605500.55,s]'
                '[data=uv,1],'
                'port1'
                '[data=Cond,0.000000,mS/cm]'
                '[rawi=ADC,563,none]'
                '[data=TempCT,23.881313,C]'
                '[rawi=ADC,428710,none],'
                'port2'
                '[data=Pressure,0.071390,dbar]'
                '[rawi=ADC,844470,2sComp],'
                'port3'
                '[data=SV,0.000000,m/s]'
                '[rawf=NSV,0.000000,samples],'
                'derive'
                '[data=Depth,0.070998,m]'
            '}'
        )

        keys = lambda d: tuple(sorted(d.keys()))
        self.assertEqual(
            keys(result),
            ('derive', 'msgnum', 'mux', 'port1', 'port2', 'port3')
        )
        self.assertEqual(keys(result['derive']), ('Depth',))
        self.assertEqual(keys(result['mux']), ('time', 'uv'))
        self.assertEqual(keys(result['port1']), ('Cond', 'TempCT'))
        self.assertEqual(keys(result['port2']), ('Pressure',))
        self.assertEqual(keys(result['port3']), ('SV',))

    def test_unexpected_mux_entry(self):
        with self.assertRaises(ValueError):
            parseAMLx('msg1{mux[data=foo,bar,baz]}')

    def test_unexpected_sensor(self):
        with self.assertRaises(ValueError):
            parseAMLx('msg1{foo[data=x,1,u]}')


if __name__ == '__main__':
    unittest.main()
