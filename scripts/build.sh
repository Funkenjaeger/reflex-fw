#!/usr/bin/env bash
# Build the firmware. Two variants, and the difference matters:
#
#   (default)  RELEASE     -- what belongs on the machine
#   --diag     DIAGNOSTIC  -- adds -DELS_DIAG_SCRATCH, which compiles in the
#                             settle-trace probe. NEVER on dev-staging/dev/main.
#
# The variants live in separate build directories on purpose. Reconfiguring one
# directory back and forth means the flag's presence depends on whatever the
# last cmake invocation happened to say, and a stale CMakeCache would silently
# give you the wrong one -- which is exactly the confusion the diagSchema
# register exists to catch at runtime. Two directories, no ambiguity.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

VARIANT=release
BUILD_DIR=build
CLEAN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --diag)  VARIANT=diagnostic; BUILD_DIR=build-diag ;;
        --clean) CLEAN=1 ;;
        -h|--help)
            sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
            echo
            echo "usage: $(basename "$0") [--diag] [--clean]"
            exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

CFLAGS=""
[ "$VARIANT" = diagnostic ] && CFLAGS="-DELS_DIAG_SCRATCH"

[ "$CLEAN" = 1 ] && rm -rf "$BUILD_DIR"

# Quiet on success, everything on failure. Building on a 9p mount (WSL against
# the Windows filesystem) emits a stream of benign "clock skew" warnings that
# would otherwise bury the one line you care about; filtering them by pattern
# would risk hiding a real one, so show all or nothing.
LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT
if ! { cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release \
            ${CFLAGS:+-DCMAKE_C_FLAGS="$CFLAGS"} &&
       cmake --build "$BUILD_DIR" -j"$(nproc)"; } >"$LOG" 2>&1; then
    cat "$LOG" >&2
    echo >&2
    echo "BUILD FAILED (${VARIANT})" >&2
    exit 1
fi

ELF="$BUILD_DIR/reflex-fw.elf"
REV="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
DIRTY=""
git diff --quiet 2>/dev/null && git diff --cached --quiet 2>/dev/null || DIRTY=" +dirty"

echo "built    ${VARIANT^^}${DIRTY}"
echo "  rev    ${REV}${DIRTY}"
echo "  elf    ${ELF}  ($(stat -c %s "$ELF") bytes)"
echo "  bin    ${BUILD_DIR}/reflex-fw.bin  ($(stat -c %s "$BUILD_DIR/reflex-fw.bin") bytes)"
if [ "$VARIANT" = diagnostic ]; then
    echo
    echo "  NOTE: diagnostic build. Carries the settle-trace probe and must not"
    echo "        reach dev-staging, dev or main. The UI will log 'ELS diagnostic"
    echo "        recorder active' when this is running."
fi
