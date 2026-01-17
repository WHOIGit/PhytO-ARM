import collections
import datetime
import re


Measurement = collections.namedtuple(
    'Measurement',
    'port name value unit rawname rawvalue rawunit'
)


def parseAMLx(s):
    parsed, msgnum = {}, None
    iterator = iterAMLx(s)

    for msgnum, sensor, kind, name, value, unit in iterator:
        if sensor == 'mux':
            if kind == 'meta' and name == 'time' and unit == 's':
                # Assume the timestamp is in UTC
                timestamp = datetime.datetime.fromtimestamp(
                    float(value), datetime.timezone.utc)
                parsed.setdefault('mux', {})['time'] = timestamp
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
                Measurement(sensor, name, float(value), unit, \
                            rawname, rawvalue, rawunit)
        elif sensor == 'derive':
            assert kind == 'data'
            parsed.setdefault('derive', {})[name] = \
                Measurement(sensor, name, float(value), unit, None, None, None)
        else:
            raise ValueError(f"Unexpected sensor type: {sensor}")

    parsed['msgnum'] = msgnum
    return parsed



def iterAMLx(s):
    # Parse the message number and between the {}s
    m = re.match(r'msg(\d+)\{(.*?)\}(?:[*]([0-9a-fA-F]{2}))?', s)
    if not m:
        raise ValueError('Invalid message format')
    msgnum, inner, checksum = m.groups()
    msgnum = int(msgnum)
    checksum = int(checksum, 16) if checksum else None

    # Checksums are included when using `set monitor checksum y`
    if checksum is not None:
        for c in s[:m.start(3)-1]:
            checksum ^= ord(c)
        if checksum != 0:
            raise ValueError('Incorrect checksum')

    # Split into a list of sensors and their parameter lists
    m = re.findall(r'(.+?)((?:\[.+?\])+)(?:,|$)', inner)
    for sensor, paramlist in m:
        m = re.findall(r'\[(.*?)=(.*?),(.*?)(?:,(.*?))?\]', paramlist)
        for kind, name, value, unit in m:
            yield msgnum, sensor, kind, name, value, unit





if __name__ == '__main__':
    import pprint

    import sys
    sample = sys.argv[1]

    pprint.pprint(parseAMLx(sample))
