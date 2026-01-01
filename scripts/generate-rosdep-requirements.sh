#!/bin/bash
# Generate rosdep apt requirements file
#
# Usage:
#   ./scripts/generate-rosdep-requirements.sh [OPTIONS]
#
# Options:
#   --verify    Verify mode - check if existing file matches current deps
#   --help      Show this help message
#
# Environment Variables:
#   DOCKER      Container runtime to use (default: docker)

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_FILE="$PROJECT_ROOT/deps/apt-rosdep-requirements.txt"
DOCKER=${DOCKER:-docker}
VERIFY_MODE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --verify)
            VERIFY_MODE=true
            shift
            ;;
        --help)
            head -n 11 "$0" | tail -n +2 | sed 's/^# \?//'
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Run with --help for usage information"
            exit 1
            ;;
    esac
done

cd "$PROJECT_ROOT"

# Build the special container target for dumping requirements
echo "Building container image with $DOCKER..."
$DOCKER build --target generate-rosdep-requirements --tag phyto-arm:gen-deps . \
    >/dev/null

echo "Extracting rosdep requirements..."
TEMP_PACKAGES=$(mktemp)
$DOCKER run --rm phyto-arm:gen-deps > "$TEMP_PACKAGES"

PACKAGE_COUNT=$(wc -l < "$TEMP_PACKAGES" | tr -d ' ')
echo "Found $PACKAGE_COUNT packages"

if [ "$VERIFY_MODE" = true ]; then
    echo "Verifying $OUTPUT_FILE..."

    if [ ! -f "$OUTPUT_FILE" ]; then
        echo "ERROR: $OUTPUT_FILE does not exist!"
        echo "Run without --verify to generate it."
        rm "$TEMP_PACKAGES"
        exit 1
    fi

    # Compare package lists (ignoring headers/comments/blank lines)
    EXPECTED=$(grep -v '^#' "$OUTPUT_FILE" | grep -v '^$' | sort)
    ACTUAL=$(sort "$TEMP_PACKAGES")

    if [ "$EXPECTED" != "$ACTUAL" ]; then
        echo "ERROR: $OUTPUT_FILE is out of date!"
        echo ""
        echo "Differences:"
        diff -u <(echo "$EXPECTED") <(echo "$ACTUAL") || true
        echo ""
        echo "To update, run: ./scripts/generate-rosdep-requirements.sh"
        rm "$TEMP_PACKAGES"
        exit 1
    fi

    rm "$TEMP_PACKAGES"
    echo "✓ $OUTPUT_FILE is up to date"
else
    # Generate output file with header
    TIMESTAMP=$(date -u "+%Y-%m-%d %H:%M:%S UTC")

    cat > "$OUTPUT_FILE" <<EOF
# Auto-generated rosdep apt requirements
# DO NOT EDIT MANUALLY - regenerate with scripts/generate-rosdep-requirements.sh
#
# Generated from:
#   - deps/deps.rosinstall (VCS dependencies)
#   - src/**/package.xml (local packages)
#
# Last updated: $TIMESTAMP
#
# To regenerate: ./scripts/generate-rosdep-requirements.sh
# To verify: ./scripts/generate-rosdep-requirements.sh --verify

EOF

    cat "$TEMP_PACKAGES" >> "$OUTPUT_FILE"
    rm "$TEMP_PACKAGES"

    echo "✓ Generated $OUTPUT_FILE with $PACKAGE_COUNT packages"
fi
