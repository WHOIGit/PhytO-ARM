#!/bin/bash
# Generate and inject parallel build stages into Dockerfile
#
# Usage:
#   ./scripts/generate-build-stages.sh [OPTIONS]
#
# Options:
#   --verify    Verify mode - check if Dockerfile is up to date
#   --help      Show this help message
#
# This script:
#   1. Builds the generate-dependencies Docker stage
#   2. Extracts package info and dependency graph
#   3. Generates parallel build stages for each package
#   4. Injects stages into Dockerfile between BUILD STEPS markers
#
# Environment Variables:
#   DOCKER           Container runtime to use (default: docker)
#   SYMLINK_INSTALL  If "true", use --symlink-install and copy build/ dirs

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKERFILE="$PROJECT_ROOT/Dockerfile"
DOCKER=${DOCKER:-docker}
SYMLINK_INSTALL=${SYMLINK_INSTALL:-false}
VERIFY_MODE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --verify)
            VERIFY_MODE=true
            shift
            ;;
        --help)
            head -n 18 "$0" | tail -n +2 | sed 's/^# \?//'
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

echo "Extracting package info and dependencies..."
TEMP_OUTPUT=$(mktemp)
$DOCKER run --rm phyto-arm:gen-deps > "$TEMP_OUTPUT"

# Extract package info and dependency graph to temp files
TEMP_PKG_INFO=$(mktemp)
TEMP_DEP_GRAPH=$(mktemp)
grep '^pkg:' "$TEMP_OUTPUT" | sed 's/^pkg://' > "$TEMP_PKG_INFO"
grep '^dep:' "$TEMP_OUTPUT" | sed 's/^dep://' > "$TEMP_DEP_GRAPH" || true
rm "$TEMP_OUTPUT"

PKG_COUNT=$(wc -l < "$TEMP_PKG_INFO" | tr -d ' ')
DEP_COUNT=$(wc -l < "$TEMP_DEP_GRAPH" | tr -d ' ')
echo "Found $PKG_COUNT packages, $DEP_COUNT dependency pairs"

# Determine symlink install flag
SYMLINK_FLAG=""
if [ "$SYMLINK_INSTALL" = "true" ]; then
    SYMLINK_FLAG="--symlink-install"
fi

# Generate build stages using awk (bash read collapses consecutive tabs)
TEMP_STAGES=$(mktemp)
awk -F'\t' -v dep_file="$TEMP_DEP_GRAPH" -v symlink="$SYMLINK_INSTALL" '
BEGIN {
    # Load dependency graph
    while ((getline line < dep_file) > 0) {
        split(line, parts, "\t")
        pkg = parts[1]
        dep = parts[2]
        if (pkg != "" && dep != "") {
            deps[pkg] = deps[pkg] " " dep
        }
    }
    close(dep_file)
}

# Recursively get all transitive dependencies for a package
function get_all_deps(pkg, visited,    i, n, dep_list, dep, result) {
    if (pkg in visited) return ""
    visited[pkg] = 1

    result = ""
    if (pkg in deps) {
        n = split(deps[pkg], dep_list, " ")
        for (i = 1; i <= n; i++) {
            dep = dep_list[i]
            if (dep != "" && !(dep in visited)) {
                result = result " " dep
                result = result get_all_deps(dep, visited)
            }
        }
    }
    return result
}

{
    pkg = $1
    path = $2
    is_external = $3

    if (pkg == "") next

    # Stage header
    print "FROM with-sources AS build-" pkg

    # Copy source if local package
    if (path != "" && is_external == "false") {
        print "COPY ros2/" path " ros2/src/" path
    }

    # Get all transitive dependencies
    delete visited
    all_deps_str = get_all_deps(pkg, visited)

    # Split and deduplicate into array
    n = split(all_deps_str, raw_deps, " ")
    delete seen
    dep_count = 0
    for (i = 1; i <= n; i++) {
        if (raw_deps[i] != "" && !(raw_deps[i] in seen)) {
            seen[raw_deps[i]] = 1
            dep_count++
            dep_list[dep_count] = raw_deps[i]
        }
    }

    # Sort dependencies
    for (i = 1; i <= dep_count; i++) {
        for (j = i + 1; j <= dep_count; j++) {
            if (dep_list[i] > dep_list[j]) {
                tmp = dep_list[i]
                dep_list[i] = dep_list[j]
                dep_list[j] = tmp
            }
        }
    }

    # Copy all transitive dependencies
    for (i = 1; i <= dep_count; i++) {
        print "COPY --from=build-" dep_list[i] " /app/ros2/install/" dep_list[i] " /app/ros2/install/" dep_list[i]
        if (symlink == "true") {
            print "COPY --from=build-" dep_list[i] " /app/ros2/build/" dep_list[i] " /app/ros2/build/" dep_list[i]
        }
    }

    # Build command
    cmake_args = ""
    symlink_flag = ""
    if (is_external == "true") {
        cmake_args = "--cmake-args -DBUILD_TESTING=OFF "
    }
    if (symlink == "true") {
        symlink_flag = "--symlink-install "
    }

    print "RUN bash -c \" \\"
    print "    source /opt/ros/${ROS_DISTRO}/setup.bash && \\"
    print "    source /app/ros2/install/setup.bash && \\"
    print "    cd /app/ros2 && \\"
    print "    colcon build " symlink_flag cmake_args "--packages-select " pkg "\""
    print ""
}
' "$TEMP_PKG_INFO" > "$TEMP_STAGES"

rm "$TEMP_PKG_INFO" "$TEMP_DEP_GRAPH"

STAGE_COUNT=$(grep -c '^FROM with-sources AS build-' "$TEMP_STAGES" || echo 0)

# Extract current build stages from Dockerfile for comparison
TEMP_CURRENT=$(mktemp)
sed -n '/^# BEGIN GENERATED BUILD STEPS$/,/^# END GENERATED BUILD STEPS$/p' "$DOCKERFILE" \
    | sed '1d;$d' > "$TEMP_CURRENT"

# Check if update needed
if diff -q "$TEMP_CURRENT" "$TEMP_STAGES" >/dev/null 2>&1; then
    echo "✓ Dockerfile build stages are up to date ($STAGE_COUNT stages)"
    rm "$TEMP_STAGES" "$TEMP_CURRENT"
    exit 0
fi

rm "$TEMP_CURRENT"

# Handle verify mode
if [ "$VERIFY_MODE" = true ]; then
    echo "ERROR: Dockerfile build stages are out of date!"
    echo ""
    echo "To update, run: ./scripts/generate-build-stages.sh"
    rm "$TEMP_STAGES"
    exit 1
fi

# Inject stages into Dockerfile
echo "Injecting $STAGE_COUNT parallel build stages..."

TEMP_DOCKERFILE=$(mktemp)
awk -v stages_file="$TEMP_STAGES" '
    /^# BEGIN GENERATED BUILD STEPS$/ {
        print
        while ((getline line < stages_file) > 0) {
            print line
        }
        close(stages_file)
        in_section = 1
        next
    }
    /^# END GENERATED BUILD STEPS$/ {
        in_section = 0
    }
    !in_section {
        print
    }
' "$DOCKERFILE" > "$TEMP_DOCKERFILE"

mv "$TEMP_DOCKERFILE" "$DOCKERFILE"
rm "$TEMP_STAGES"

echo "✓ Updated Dockerfile with $STAGE_COUNT parallel build stages"
