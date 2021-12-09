import collections
import datetime
import re


Measurement = collections.namedtuple(
    'Measurement',
    'value unit rawname rawvalue rawunit'
)


def parseAMLx(s):
    parsed, msgnum = {}, None
    iterator = iterAMLx(s)
    
    for msgnum, sensor, kind, name, value, unit in iterator:
        if sensor == 'mux':
            if kind == 'meta' and name == 'time' and unit == 's':
                parsed.setdefault('mux', {})['time'] = \
                    datetime.datetime.fromtimestamp(float(value))
            elif kind == 'data' and name == 'uv' and unit == '':
                parsed.setdefault('mux', {})['uv'] = bool(value)
            else:
                raise ValueError("Unexpected entry in mux section")
        elif sensor.startswith('port'):
            assert kind == 'data'
            _, rawsensor, rawkind, rawname, rawvalue, rawunit = next(iterator)
            assert rawsensor == sensor, rawkind in ('rawi', 'rawf')
            rawvalue = int(rawvalue) if rawkind == 'rawi' else float(rawvalue)
            parsed.setdefault(sensor, {})[name] = \
                Measurement(float(value), unit, rawname, rawvalue, rawunit)
        elif sensor == 'derive':
            assert kind == 'data'
            parsed.setdefault('derive', {})[name] = \
                Measurement(float(value), unit, None, None, None)
        else:
            raise ValueError("Unexpected sensor type")

    parsed['msgnum'] = msgnum
    return parsed



def iterAMLx(s):
    # Parse the message number and between the {}s
    m = re.match(r'msg(\d+)\{(.*?)\}', s)
    assert m
    msgnum, inner = m.groups()
    msgnum = int(msgnum)

    # Split into a list of sensors and their parameter lists
    m = re.findall(r'(.+?)((?:\[.+?\])+)(?:,|$)', inner)
    for sensor, paramlist in m:
        m = re.findall(r'\[(.*?)=(.*?),(.*?)(?:,(.*?))?\]', paramlist)
        for kind, name, value, unit in m:
            yield msgnum, sensor, kind, name, value, unit





if __name__ == '__main__':
    import pprint

    sample = '''msg138{mux[meta=time,1590605500.55,s][data=uv,1],port1[data=Cond,0.000000,mS/cm][rawi=ADC,563,none][data=TempCT,23.881313,C][rawi=ADC,428710,none],port2[data=Pressure,0.071390,dbar][rawi=ADC,844470,2sComp],port3[data=SV,0.000000,m/s][rawf=NSV,0.000000,samples],derive[data=Depth,0.070998,m]}'''

    pprint.pprint(parseAMLx(sample))
