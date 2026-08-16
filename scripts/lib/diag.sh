# Diagnostic probe registry. Sourced by build.sh and flash.sh; not executable.
#
# THE REGISTRY LIVES IN Core/Inc/Ramps.h, NOT HERE. These functions parse the
# ELS_DIAG_SCHEMA_* defines out of the header at call time, so the list of probes
# a script will accept is derived from the firmware rather than duplicated
# alongside it. A probe added to the header is immediately selectable; a probe
# that does not exist cannot be selected however it is spelled. The alternative
# -- a hand-maintained list in shell -- drifts the first time someone adds a
# probe and forgets this file, and then `--diag=whatever` builds something other
# than what it names.
#
# Retirement is expressed in the header too: a schema line commented RETIRED is
# parsed out of the selectable list but keeps its id, so a retired probe's number
# is never handed to a different probe. reflex-ui mirrors these ids.

DIAG_HEADER_REL="Core/Inc/Ramps.h"

# CLI name <-> macro name. `ELS_DIAG_SCHEMA_TAKEUP_SETTLE_V2` <-> `takeup-settle-v2`.
_diag_macro_to_name() {
    printf '%s' "${1#ELS_DIAG_SCHEMA_}" | tr 'A-Z_' 'a-z-'
}
_diag_name_to_macro() {
    printf 'ELS_DIAG_SCHEMA_%s' "$(printf '%s' "$1" | tr 'a-z-' 'A-Z_')"
}

# Selectable probes, one CLI name per line. Excludes NONE (the absence of a
# probe is not a probe) and anything marked RETIRED.
diag_probe_list() {
    local header="$1/$DIAG_HEADER_REL"
    [ -r "$header" ] || { echo "cannot read $header" >&2; return 1; }
    grep -E '^#define[[:space:]]+ELS_DIAG_SCHEMA_[A-Z0-9_]+[[:space:]]+[0-9]+' "$header" \
      | grep -v 'RETIRED' \
      | awk '{print $2}' \
      | grep -v '^ELS_DIAG_SCHEMA_NONE$' \
      | while read -r macro; do _diag_macro_to_name "$macro"; done
}

# Resolve a CLI name to its macro, or fail. Validating against the parsed header
# is what makes a typo a hard error here rather than an undefined identifier that
# the C preprocessor would quietly evaluate to 0.
diag_resolve() {
    local repo="$1" name="$2" macro
    macro="$(_diag_name_to_macro "$name")"
    if ! diag_probe_list "$repo" | grep -qx "$name"; then
        return 1
    fi
    printf '%s' "$macro"
}

# One build directory per probe, for the same reason release and diagnostic have
# separate ones: a directory reused across configurations makes the flag depend
# on whatever the last cmake invocation happened to say.
diag_build_dir() {
    printf 'build-diag-%s' "$1"
}

# Shared by both scripts' --diag error paths. The list IS the documentation at
# the point of use -- nobody has to remember a probe name to discover it.
diag_usage_probes() {
    local repo="$1"
    echo "  available probes (see DIAG.md):" >&2
    diag_probe_list "$repo" | sed 's/^/    --diag=/' >&2
}
