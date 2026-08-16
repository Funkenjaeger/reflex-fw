#!/usr/bin/env bash
# Build the firmware. Two variants, and the difference matters:
#
#   (default)      RELEASE     -- what belongs on the machine
#   --diag=PROBE   DIAGNOSTIC  -- compiles in ONE diagnostic probe.
#                                 NEVER on dev-staging/dev/main.
#
# --diag REQUIRES a probe name; bare --diag lists what is available and stops.
# That is deliberate. The probes share one 64-register scratchpad, so only one
# can be compiled in at a time, and which one it is changes what every field in
# that block means. A default would make "which probe am I running" a question
# you have to remember the answer to, which is the confusion the whole scheme
# exists to remove. See DIAG.md.
#
# The variants live in separate build directories on purpose, one per probe.
# Reconfiguring one directory back and forth means the flag's presence depends on
# whatever the last cmake invocation happened to say, and a stale CMakeCache
# would silently give you the wrong one -- which is exactly the confusion the
# diagSchema register exists to catch at runtime. Separate directories, no
# ambiguity.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
# shellcheck source=lib/diag.sh
. "$REPO/scripts/lib/diag.sh"

VARIANT=release
BUILD_DIR=build
PROBE=""
CLEAN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --diag)
            echo "--diag requires a probe: --diag=<name>" >&2
            diag_usage_probes "$REPO"
            exit 2 ;;
        --diag=*)
            PROBE="${1#--diag=}"
            VARIANT=diagnostic
            BUILD_DIR="$(diag_build_dir "$PROBE")" ;;
        --clean) CLEAN=1 ;;
        -h|--help)
            sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
            echo
            echo "usage: $(basename "$0") [--diag=<probe>] [--clean]"
            diag_usage_probes "$REPO"
            exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

CFLAGS=""
if [ "$VARIANT" = diagnostic ]; then
    # Resolved against the schema defines in Ramps.h, so an unrecognised name
    # fails HERE with the list in hand. Passing it through unchecked would reach
    # the preprocessor as an undefined identifier, evaluate to 0, and produce a
    # "diagnostic" build carrying no probe -- caught by the #error in Ramps.h,
    # but with a worse message and after a full configure.
    if ! MACRO="$(diag_resolve "$REPO" "$PROBE")"; then
        echo "unknown diagnostic probe: $PROBE" >&2
        diag_usage_probes "$REPO"
        exit 2
    fi
    CFLAGS="-DELS_DIAG_PROBE=$MACRO"
fi

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
    echo "  NOTE: diagnostic build. Carries the '${PROBE}' probe (${MACRO}) and"
    echo "        must not reach dev-staging, dev or main. The UI will log 'ELS"
    echo "        diagnostic recorder active' with this schema when it is running."
    echo "        Probe details: DIAG.md"
fi
