#!/usr/bin/env bash
# Build and flash the firmware, in one command.
#
#   ./scripts/flash.sh              release build, flash locally
#   ./scripts/flash.sh --diag       diagnostic build (ELS settle-trace probe)
#   ./scripts/flash.sh --no-build   flash what is already built
#   ./scripts/flash.sh --dry-run    everything except the write
#   ./scripts/flash.sh --host NAME  build here, flash on NAME over ssh
#
# LOCAL IS THE DEFAULT, and that is the whole point. Run this on the machine
# with the ST-Link plugged in -- which for this project is the Pi that also runs
# the UI -- and there is no copy, no second machine, and no way for the binary on
# the target to be a different revision from the checkout you are looking at.
# `git rev-parse HEAD` there IS what is flashed.
#
# --host exists for the case where the probe host genuinely cannot build (no ARM
# toolchain, or you want the faster machine to compile). It adds a copy and a
# checksum, because a transfer that can silently truncate is worth verifying
# before it gets written to the controller of a machine with moving parts.
#
# EITHER WAY IT REBUILDS FIRST. --no-build opts out. A stale binary is the
# easiest mistake to make and the hardest to notice, and rebuilding costs
# seconds.
#
# IT RECORDS WHAT IT FLASHED, in ~/firmware/flashed.json on the probe host.
# Working out what firmware was on this lathe once took an afternoon of
# archaeology across build-artifact timestamps on a machine that was powered
# off. One line of JSON per flash makes that a lookup. Nothing reads it yet; it
# exists so the question has an answer.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

VARIANT=release
BUILD_DIR=build
HOST=""          # empty = flash on this machine
DO_BUILD=1
DRY=0

while [ $# -gt 0 ]; do
    case "$1" in
        --diag)     VARIANT=diagnostic; BUILD_DIR=build-diag ;;
        --no-build) DO_BUILD=0 ;;
        --host)     HOST="${2:?--host needs a value}"; shift ;;
        --dry-run)  DRY=1 ;;
        -h|--help)  sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

if [ "$DO_BUILD" = 1 ]; then
    # NOT `test && a || b` -- if the diagnostic build FAILED, that idiom falls
    # through and quietly builds release instead, which is the precise
    # wrong-variant confusion this script exists to prevent.
    if [ "$VARIANT" = diagnostic ]; then
        "$REPO/scripts/build.sh" --diag
    else
        "$REPO/scripts/build.sh"
    fi
    echo
fi

ELF="$BUILD_DIR/reflex-fw.elf"
[ -f "$ELF" ] || { echo "no firmware at $ELF (drop --no-build?)" >&2; exit 1; }

REV="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
DIRTY=false
git diff --quiet 2>/dev/null && git diff --cached --quiet 2>/dev/null || DIRTY=true

# A dirty tree WARNS rather than blocks. Refusing would just get worked around,
# and iterating at the machine is legitimate -- but the revision alone then stops
# identifying the binary, so the manifest records it.
if [ "$DIRTY" = true ]; then
    echo "WARNING: working tree is dirty. '$REV' does not fully identify this"
    echo "         binary; recording it as ${REV}-dirty."
    echo
fi

MD5="$(md5sum "$ELF" | cut -d' ' -f1)"

if [ -n "$HOST" ]; then
    # Preflight before doing anything that looks like progress. Note ssh config
    # is per-context: a host alias that works in a Windows terminal does not
    # exist inside WSL, which has its own ~/.ssh.
    if ! ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" true 2>/dev/null; then
        cat >&2 <<EOF
cannot reach '$HOST' over ssh from this machine.

Needs passwordless ssh to the machine with the ST-Link attached: the host must
resolve here, and your key must be authorized there in THIS shell's ssh context.

Simpler option: install the ARM toolchain on the probe host and run this script
there with no --host at all.
EOF
        exit 1
    fi
    TARGET_ELF="firmware/reflex-fw-${VARIANT}.elf"
    RUN=(ssh "$HOST")
    WHERE="$HOST"
else
    TARGET_ELF="$REPO/$ELF"
    RUN=(bash -c)
    WHERE="this machine"
fi

echo "flashing ${VARIANT^^} (${REV}$([ "$DIRTY" = true ] && echo -dirty)) on ${WHERE}"

if [ -n "$HOST" ]; then
    ssh "$HOST" 'mkdir -p ~/firmware'
    scp -q "$ELF" "$HOST:$TARGET_ELF"
    REMOTE_MD5="$(ssh "$HOST" "md5sum $TARGET_ELF | cut -d' ' -f1")"
    [ "$MD5" = "$REMOTE_MD5" ] || {
        echo "TRANSFER CORRUPT: local $MD5 != remote $REMOTE_MD5 -- not flashing" >&2
        exit 1
    }
else
    mkdir -p ~/firmware
fi

# 'transport select swd' is explicit to suppress openocd's auto-select
# deprecation warning. 'reset' leaves the target executing rather than halted --
# a halted lathe controller with the UI still polling it looks like a comms
# fault and invites debugging the wrong thing.
OPENOCD_CMD="openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c 'transport select swd' \
    -c 'program $TARGET_ELF verify reset exit'"

if [ "$DRY" = 1 ]; then
    echo
    echo "DRY RUN -- everything above actually happened. Stopping before the"
    echo "write. Would now run on ${WHERE}:"
    echo
    echo "  $OPENOCD_CMD"
    exit 0
fi

"${RUN[@]}" "$OPENOCD_CMD"

STAMP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
MANIFEST="{\"utc\":\"$STAMP\",\"variant\":\"$VARIANT\",\"rev\":\"$REV\",\"dirty\":$DIRTY,\"md5\":\"$MD5\"}"
"${RUN[@]}" "printf '%s\n' '$MANIFEST' >> ~/firmware/flashed.json"

echo
echo "flashed  ${VARIANT^^}  rev ${REV}$([ "$DIRTY" = true ] && echo " (dirty)")"
echo "  recorded in ${WHERE}:~/firmware/flashed.json"
if [ "$VARIANT" = diagnostic ]; then
    echo
    echo "  The UI reconnects on its own after the reset. Confirm it logs"
    echo "  'ELS diagnostic recorder active' -- 'dormant' means the flash did not"
    echo "  take, or reflex-ui on that machine predates the recorder."
fi
