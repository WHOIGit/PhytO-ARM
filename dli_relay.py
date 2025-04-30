#!/usr/bin/env python3
"""
Digital Loggers Relay Control Script

A simple script to control DLI Power Relays via their REST API.
Usage: python dli_relay.py --host <hostname> --user <username> --password <password> --outlet <outlet_number> --action <on|off|cycle>
"""

import argparse
import requests
from requests.auth import HTTPDigestAuth
import sys
import json

def main():
    parser = argparse.ArgumentParser(description='Control a Digital Loggers relay via REST API')
    parser.add_argument('--host', required=True, help='Hostname or IP address of the relay')
    parser.add_argument('--user', default='admin', help='Username (default: admin)')
    parser.add_argument('--password', required=True, help='Password')
    parser.add_argument('--outlet', required=True, type=int, help='Outlet number (starting from 0)')
    parser.add_argument('--action', required=True, choices=['on', 'off', 'cycle', 'status'], 
                        help='Action to perform: on, off, cycle, or status')
    parser.add_argument('--protocol', default='http', choices=['http', 'https'], 
                        help='Protocol to use (default: http)')
    
    args = parser.parse_args()
    
    # Form the base URL and authentication
    base_url = f"{args.protocol}://{args.host}/restapi/"
    auth = HTTPDigestAuth(args.user, args.password)
    
    try:
        # First check if we can access the API
        response = requests.get(f"{base_url}", auth=auth)
        response.raise_for_status()
        
        if args.action == 'status':
            # Get the outlet state
            url = f"{base_url}relay/outlets/{args.outlet}/state/"
            response = requests.get(url, auth=auth, headers={'Accept': 'application/json'})
            response.raise_for_status()
            state = response.json()
            
            # Also get the name for better output
            name_url = f"{base_url}relay/outlets/{args.outlet}/name/"
            name_response = requests.get(name_url, auth=auth, headers={'Accept': 'application/json'})
            name = name_response.json() if name_response.status_code == 200 else f"Outlet {args.outlet}"
            
            print(f"Outlet {args.outlet} ({name}) is {'ON' if state else 'OFF'}")
            
        elif args.action == 'cycle':
            # Cycle the outlet (turn off then on)
            url = f"{base_url}relay/outlets/{args.outlet}/cycle/"
            response = requests.post(url, auth=auth, headers={'X-CSRF': 'x'})
            response.raise_for_status()
            print(f"Outlet {args.outlet} cycled successfully")
            
        else:  # on or off
            # Set the outlet state
            value = 'true' if args.action == 'on' else 'false'
            url = f"{base_url}relay/outlets/{args.outlet}/state/"
            response = requests.put(
                url, 
                auth=auth, 
                headers={
                    'Content-Type': 'application/x-www-form-urlencoded',
                    'X-CSRF': 'x'  # Required for CSRF protection
                },
                data=f'value={value}'
            )
            response.raise_for_status()
            print(f"Outlet {args.outlet} turned {'ON' if args.action == 'on' else 'OFF'} successfully")
        
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
