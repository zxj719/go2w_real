#!/usr/bin/env bash
set -euo pipefail

# Unitree's bundled xt16_driver rejects frames whose inter-frame delta is below
# 70 ms. This robot's XT16 currently produces stable frames around 63 ms, so the
# stock binary discards nearly every cloud before /scan can be generated.
#
# Keep the vendor binary intact. A side-by-side copy is patched at the single
# double constant that represents the lower accepted frame interval:
#   0.07 s -> 0.05 s

VENDOR_XT16_BIN="${GO2W_XT16_VENDOR_BIN:-/unitree/module/unitree_slam/bin/xt16_driver}"
PATCHED_XT16_BIN="${GO2W_XT16_PATCHED_BIN:-$(dirname "${VENDOR_XT16_BIN}")/xt16_driver_go2w_timefix}"
THRESHOLD_OFFSET="${GO2W_XT16_TIMEFIX_OFFSET:-0x10fa70}"
ORIGINAL_LOWER_BOUND_HEX="ec51b81e85ebb13f"
PATCHED_LOWER_BOUND_HEX="9a9999999999a93f"

offset_dec() {
  printf '%d' "$((THRESHOLD_OFFSET))"
}

hex_at_threshold() {
  local file="$1"
  od -An -tx1 -N8 -j"$(offset_dec)" "${file}" 2>/dev/null | tr -d ' \n'
}

ensure_patched_driver() {
  if [[ ! -x "${VENDOR_XT16_BIN}" ]]; then
    echo "[xt16_driver_timefix] vendor xt16_driver not executable: ${VENDOR_XT16_BIN}" >&2
    exit 1
  fi

  if [[ "${PATCHED_XT16_BIN}" == "${VENDOR_XT16_BIN}" ]]; then
    echo "[xt16_driver_timefix] patched binary path must differ from vendor binary" >&2
    exit 1
  fi

  local vendor_hex
  vendor_hex="$(hex_at_threshold "${VENDOR_XT16_BIN}")"
  if [[ "${vendor_hex}" != "${ORIGINAL_LOWER_BOUND_HEX}" && "${vendor_hex}" != "${PATCHED_LOWER_BOUND_HEX}" ]]; then
    echo "[xt16_driver_timefix] unexpected vendor bytes at ${THRESHOLD_OFFSET}: ${vendor_hex}" >&2
    echo "[xt16_driver_timefix] refusing to patch an unknown xt16_driver build" >&2
    exit 1
  fi

  if [[ ! -x "${PATCHED_XT16_BIN}" || "${GO2W_XT16_REBUILD_PATCHED:-0}" == "1" || "${VENDOR_XT16_BIN}" -nt "${PATCHED_XT16_BIN}" ]]; then
    mkdir -p "$(dirname "${PATCHED_XT16_BIN}")"
    cp -f "${VENDOR_XT16_BIN}" "${PATCHED_XT16_BIN}"
    chmod --reference="${VENDOR_XT16_BIN}" "${PATCHED_XT16_BIN}"
  fi

  local patched_hex
  patched_hex="$(hex_at_threshold "${PATCHED_XT16_BIN}")"
  case "${patched_hex}" in
    "${PATCHED_LOWER_BOUND_HEX}")
      ;;
    "${ORIGINAL_LOWER_BOUND_HEX}")
      printf '\x9a\x99\x99\x99\x99\x99\xa9\x3f' \
        | dd of="${PATCHED_XT16_BIN}" bs=1 seek="$(offset_dec)" conv=notrunc status=none
      ;;
    *)
      echo "[xt16_driver_timefix] unexpected patched bytes at ${THRESHOLD_OFFSET}: ${patched_hex}" >&2
      echo "[xt16_driver_timefix] delete ${PATCHED_XT16_BIN} or set GO2W_XT16_REBUILD_PATCHED=1" >&2
      exit 1
      ;;
  esac
}

ensure_patched_driver

if [[ "${GO2W_XT16_TIMEFIX_DRY_RUN:-0}" == "1" ]]; then
  echo "[xt16_driver_timefix] patched binary ready: ${PATCHED_XT16_BIN}"
  exit 0
fi

echo "[xt16_driver_timefix] using ${PATCHED_XT16_BIN} with 50 ms lower frame-delta bound" >&2
cd "$(dirname "${VENDOR_XT16_BIN}")"
exec "${PATCHED_XT16_BIN}" "$@"
