#!/usr/bin/env python3
"""
API client for Digital Loggers Web Power Switch Pro.
"""
import urllib.request


class DLIClient:
    def __init__(self, address: str, username: str, password: str) -> None:
        self.address = address

        # Configure opener for HTTP Digest Authentication
        pwrd_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        pwrd_mgr.add_password(None, f'http://{address}/', username, password)
        auth_handler = urllib.request.HTTPDigestAuthHandler(pwrd_mgr)
        self.opener = urllib.request.build_opener(auth_handler)

    def set_outlet_state(self, number: int, is_active: bool) -> None:
        self.opener.open(urllib.request.Request(
            f'http://{self.address}/restapi/relay/outlets/{number}/state/',
            data=f'value={str(is_active).lower()}'.encode(),
            method='PUT',
            headers={
                'X-CSRF': 'x'
            },
        ))

    def get_outlet_state(self, number: int) -> bool:
        resp = self.opener.open(urllib.request.Request(
            f'http://{self.address}/restapi/relay/outlets/{number}/state/'
        ))
        return resp.read() == b'true'
