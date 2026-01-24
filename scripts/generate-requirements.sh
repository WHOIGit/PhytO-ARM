#!/bin/bash
# Generate apt dependency requirements file
#
# Usage:
#   ./scripts/generate-requirements.sh [OPTIONS]
#
# Options:
#   --verify    Verify mode - check if existing file matches current deps
#   --help      Show this help message
#
# Outputs:
#   deps/apt-rosdep-requirements.txt  - APT packages from rosdep
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
            head -n 15 "$0" | tail -n +2 | sed 's/^# \?//'
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
$DOCKER build --target generate-dependencies --tag phyto-arm:gen-deps . \
    >/dev/null

echo "Extracting apt dependencies..."
TEMP_APT=$(mktemp)
$DOCKER run --rm phyto-arm:gen-deps | grep '^apt:' | sed 's/^apt://' > "$TEMP_APT"

APT_COUNT=$(wc -l < "$TEMP_APT" | tr -d ' ')
echo "Found $APT_COUNT apt packages"

# Check if file needs updating
NEEDS_UPDATE=false
if [ -f "$OUTPUT_FILE" ]; then
    EXPECTED=$(grep -v '^#' "$OUTPUT_FILE" | grep -v '^$' | sort)
    ACTUAL=$(sort "$TEMP_APT")
    if [ "$EXPECTED" != "$ACTUAL" ]; then
        NEEDS_UPDATE=true
    fi
else
    NEEDS_UPDATE=true
fi

# Handle verify mode
if [ "$VERIFY_MODE" = true ]; then
    if [ "$NEEDS_UPDATE" = true ]; then
        if [ ! -f "$OUTPUT_FILE" ]; then
            echo "ERROR: $OUTPUT_FILE does not exist!"
        else
            echo "ERROR: $OUTPUT_FILE is out of date!"
            diff -u <(echo "$EXPECTED") <(echo "$ACTUAL") || true
        fi
        rm "$TEMP_APT"
        echo ""
        echo "To update, run: ./scripts/generate-requirements.sh"
        exit 1
    else
        echo "✓ $OUTPUT_FILE is up to date"
        rm "$TEMP_APT"
        exit 0
    fi
fi

# Generate mode
if [ "$NEEDS_UPDATE" = false ]; then
    echo "✓ $OUTPUT_FILE is up to date"
    rm "$TEMP_APT"
    exit 0
fi

TIMESTAMP=$(date -u "+%Y-%m-%d %H:%M:%S UTC")

cat > "$OUTPUT_FILE" <<EOF
# Auto-generated rosdep apt requirements
# DO NOT EDIT MANUALLY - regenerate with scripts/generate-requirements.sh
#
# Last updated: $TIMESTAMP

EOF
cat "$TEMP_APT" >> "$OUTPUT_FILE"
rm "$TEMP_APT"

echo "✓ Generated $OUTPUT_FILE with $APT_COUNT packages"
