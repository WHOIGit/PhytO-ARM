import re
import sys

if len(sys.argv) != 2:
    print("Usage: python3 udp_regex_test.py '<regex_pattern>'", file=sys.stderr)
    sys.exit(1)

# Compile regex from command-line argument
try:
    pattern = re.compile(sys.argv[1])
except re.error as e:
    print(f"Invalid regular expression: {e}", file=sys.stderr)
    sys.exit(1)

# Read lines from stdin and split using the regex
for line in sys.stdin:
    line = line.strip()
    tokens = pattern.split(line)
    print("Split result:", tokens)