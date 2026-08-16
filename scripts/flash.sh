#!/usr/bin/env bash
# Build and flash the firmware, in one command.
#
#   ./scripts/flash.sh            release build -> elspi
#   ./scripts/flash.sh --diag     diagnostic build (settle-trace probe)
#   ./scripts/flash.sh --no-build flash whatever is already built
#   ./scripts/flash.sh --host X   target a different machine
#
# IT BUILDS FIRST BY DEFAULT, and that is the point. The ARM toolchain lives on
# the build host while the ST-Link hangs off the Pi, so "flash" has always meant
# build here, copy there, run openocd there -- three steps across two machines,
# with the copy silently able to be a version behind. Building every time costs
# a few seconds and removes the entire class of "which binary is actually on the
# machine".
#
# IT RECORDS WHAT IT FLASHED, in ~/firmware/flashed.json on the target. Working
# out what firmware was on this lathe once took an afternoon of archaeology
# across build-artifact timestamps on a laptop that was powered off. One line of
# JSON per flash makes that a lookup. Nothing reads this file yet; it exists so
# the question has an answer at all.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

VARIANT=release
BUILD_DIR=build
HOST=elspi
DO_BUILD=1
DRY=0

while [ $# -gt 0 ]; do
    case "$1" in
        --diag)     VARIANT=diagnostic; BUILD_DIR=build-diag ;;
        --no-build) DO_BUILD=0 ;;
        --host)     HOST="${2:?--host needs a value}"; shift ;;
        --dry-run)  DRY=1 ;;
        -h|--help)
            sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
            exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

if [ "$DO_BUILD" = 1 ]; then
    if [ "$VARIANT" = diagnostic ]; then "$REPO/scripts/build.sh" --diag
    else "$REPO/scripts/build.sh"; fi
    echo
fi

ELF="$BUILD_DIR/reflex-fw.elf"
[ -f "$ELF" ] || { echo "no firmware at $ELF (drop --no-build?)" >&2; exit 1; }

REV="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
DIRTY=false
git diff --quiet 2>/dev/null && git diff --cached --quiet 2>/dev/null || DIRTY=true

# A dirty tree WARNS rather than blocks. Refusing would just get worked around,
# and iterating on the machine is a legitimate thing to do -- but it does mean
# the revision alone no longer identifies the binary, so the manifest records it.
if [ "$DIRTY" = true ]; then
    echo "WARNING: working tree is dirty. '$REV' does not fully identify this"
    echo "         binary; recording it as ${REV}-dirty."
    echo
fi

MD5="$(md5sum "$ELF" | cut -d' ' -f1)"
REMOTE_ELF="firmware/reflex-fw-${VARIANT}.elf"

# Preflight the connection before doing anything that looks like progress.
# Failing here with an explanation beats failing three steps in with a raw
# resolver error. Note SSH config is per-context: a host alias that works in a
# Windows terminal does not exist inside WSL, which has its own ~/.ssh.
if ! ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" true 2>/dev/null; then
    cat >&2 <<EOF
cannot reach '$HOST' over ssh from this machine.

Needs passwordless ssh to the machine with the ST-Link attached. Check that:
  - the host name resolves here (ssh config alias, /etc/hosts, or mDNS)
  - your key is authorized there, in THIS shell's ssh context

Override the target with:  $(basename "$0") --host <name-or-ip>
EOF
    exit 1
fi

echo "flashing ${VARIANT^^} (${REV}$([ "$DIRTY" = true ] && echo -dirty)) -> ${HOST}"

ssh "$HOST" 'mkdir -p ~/firmware'
scp -q "$ELF" "$HOST:$REMOTE_ELF"

# Verify the copy before flashing it. A truncated scp that then gets written to
# the controller of a machine with moving parts is not a failure mode worth
# saving two seconds on.
REMOTE_MD5="$(ssh "$HOST" "md5sum $REMOTE_ELF | cut -d' ' -f1")"
[ "$MD5" = "$REMOTE_MD5" ] || {
    echo "TRANSFER CORRUPT: local $MD5 != remote $REMOTE_MD5 -- not flashing" >&2
    exit 1
}

# 'transport select swd' is explicit to suppress openocd's auto-select
# deprecation warning. 'reset run' leaves the target executing rather than
# halted -- a halted lathe controller with the UI still polling it looks like a
# comms fault and invites someone to start debugging the wrong thing.
OPENOCD_CMD="openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c 'transport select swd' \
    -c 'program $REMOTE_ELF verify reset exit'"

if [ "$DRY" = 1 ]; then
    echo
    echo "DRY RUN -- everything above actually happened (build, copy, checksum"
    echo "verified). Stopping before the write. Would now run on ${HOST}:"
    echo
    echo "  $OPENOCD_CMD"
    echo
    exit 0
fi

ssh "$HOST" "$OPENOCD_CMD"

STAMP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
ssh "$HOST" "printf '%s\n' '{\"utc\":\"$STAMP\",\"variant\":\"$VARIANT\",\"rev\":\"$REV\",\"dirty\":$DIRTY,\"md5\":\"$MD5\",\"elf\":\"$REMOTE_ELF\"}' >> ~/firmware/flashed.json"

echo
echo "flashed  ${VARIANT^^}  rev ${REV}$([ "$DIRTY" = true ] && echo " (dirty)")"
echo "  recorded in ${HOST}:~/firmware/flashed.json"
if [ "$VARIANT" = diagnostic ]; then
    echo
    echo "  The UI reconnects on its own after the reset. Confirm it logs"
    echo "  'ELS diagnostic recorder active' -- 'dormant' means the flash did not"
    echo "  take, or reflex-ui on the target predates the recorder."
fi
