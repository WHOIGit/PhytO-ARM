import os
import sys
import unittest

from urllib.request import HTTPDigestAuthHandler

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from dli_power_switch import api


class MockHTTPResponse:
    def __init__(self, body: bytes, status: int = 200):
        self.body = body
        self.status = status

    def read(self) -> bytes:
        return self.body


class MockOpener:
    def __init__(self, response: MockHTTPResponse):
        self.response = response
        self.requests = []

    def open(self, req):
        self.requests.append(req)
        return self.response


class TestDLIClient(unittest.TestCase):
    def test_digest_handler_installed(self):
        client = api.DLIClient('localhost', 'user', 'pass')
        self.assertTrue(any(
            isinstance(h, HTTPDigestAuthHandler)
            for h in client.opener.handlers
        ))

    def test_set_outlet_state(self):
        client = api.DLIClient('localhost', 'user', 'pass')
        client.opener = MockOpener(MockHTTPResponse(b'ok', status=200))
        client.set_outlet_state(3, True)
        self.assertEqual(len(client.opener.requests), 1)

        req = client.opener.requests[0]
        self.assertEqual(req.get_method(), 'PUT')
        self.assertEqual(req.data.decode(), 'value=true')
    
    def test_set_outlet_state_csrf_header(self):
        client = api.DLIClient('localhost', 'user', 'pass')
        client.opener = MockOpener(MockHTTPResponse(b'ok', status=200))
        client.set_outlet_state(3, True)
        req = client.opener.requests[0]
        self.assertTrue(any(
            k.lower() == 'x-csrf' and v == 'x'
            for k, v in req.headers.items()
        ))

    def test_get_outlet_state(self):
        client = api.DLIClient('localhost', 'user', 'pass')
        client.opener = MockOpener(MockHTTPResponse(b'true'))
        self.assertTrue(client.get_outlet_state(5))
        client.opener = MockOpener(MockHTTPResponse(b'false'))
        self.assertFalse(client.get_outlet_state(5))


if __name__ == '__main__':
    unittest.main()
