#!/usr/bin/bash
set -e

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=false}"

# ---- ANT_BUILD_VERSION export (for local pixi/distrobox runs) ----------------
# submit.sh sets BUILD_VERSION for docker builds.  Local pixi runs bypass that,
# so without this they show "build_version=unknown" in ANT.py's startup log.
# Re-evaluating each activation captures the current SHA at shell time; the
# -dirty suffix flags uncommitted work so we never wonder which code ran.
if [[ -z "${ANT_BUILD_VERSION:-}" ]]; then
  _aic_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]:-${0}}")" && pwd)"
  if command -v git >/dev/null 2>&1 && [[ -d "${_aic_repo_root}/.git" ]]; then
    _aic_sha="$(git -C "${_aic_repo_root}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    if [[ "${_aic_sha}" != "unknown" ]] && ! git -C "${_aic_repo_root}" diff --quiet HEAD 2>/dev/null; then
      _aic_sha="${_aic_sha}-dirty"
    fi
    export ANT_BUILD_VERSION="local-${_aic_sha}"
  fi
  unset _aic_repo_root _aic_sha
fi
