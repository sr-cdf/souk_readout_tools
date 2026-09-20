#!/bin/bash
# Board-side measurements for one firmware stage of the v7.12 comparison
# (doc/v712-notes.md "Staged comparison"): B0 state, B1 transport scaling,
# B2 sweep-step profile, B3 read-path timing. Run from this directory:
#
#     sudo ./run_stage.sh S0
#
# Stops both readout servers (and timing-monitor, whose 3 s chronyc comb shows
# up in tail latencies) and restores them on exit. B1/B2 overwrite pipeline 0's
# TX LO control buffer, so re-set tones afterwards. Results land in
# ./stages/<STAGE>/ -- copy that directory back to the host.
#
# UIO, ZDMA and the DRAM-ceiling runs are only attempted when the kernel
# exposes user_io and a spare u-dma-buf (the v7.12 image.ub, i.e. S3).
set -u
STAGE=${1:?usage: sudo ./run_stage.sh <STAGE>}
[ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)"; exit 1; }
cd "$(dirname "$0")"
PY=/home/casper/py3.12-venv/bin/python3.12
CFG=/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml
OUT=stages/$STAGE
mkdir -p "$OUT"
chown casper:casper stages "$OUT"      # the rootless UIO runs write here too

TM_WAS=$(systemctl is-active timing-monitor)
restore() {
    [ "$TM_WAS" = active ] && systemctl start timing-monitor
    systemctl start readout_server_0 readout_server_1
    chown -R casper:casper stages
    echo "== servers restarted; results in $(pwd)/$OUT"
}
trap restore EXIT
systemctl stop readout_server_0 readout_server_1 timing-monitor
sleep 2

echo "== B0 board state"
$PY board_state.py --label "$STAGE" --out "$OUT/b0_state.json" > /dev/null
$PY -c "import json; s=json.load(open('$OUT/b0_state.json')); \
print(json.dumps({k: s[k] for k in ('boot_files','ps_registers','software','configured_fpg','uio')}, indent=1))"
eval "$($PY fpg_addrs.py "$CFG" --shell)"
echo "targets: write $WRITE_NAME $WRITE_ADDR, read $READ_NAME $READ_ADDR, ref $REF_NAME $REF_ADDR"
[ -x ./devmem_profile ] || make

scale() {   # scale <tag> <access_size_scaling args...>
    local tag=$1; shift
    echo "== B1 $tag"
    $PY access_size_scaling.py "$@" --label "$STAGE $tag" --out "$OUT/b1_$tag.json" \
        > "$OUT/b1_$tag.txt" 2>&1 || echo "   FAILED, see $OUT/b1_$tag.txt"
    grep -E "^fit|frombuffer alone" "$OUT/b1_$tag.txt"
}
scale devmem_acc --transport devmem --read-addr "$READ_ADDR" --no-write
scale devmem_lo  --transport devmem --read-addr "$WRITE_ADDR" --write-addr "$WRITE_ADDR"
scale devmem_ref --transport devmem --read-addr "$REF_ADDR" --max-bytes "$REF_SIZE" --no-write

HAVE_UIO=$(grep -lx user_io /sys/class/uio/uio*/name 2>/dev/null | head -1)
if [ -n "$HAVE_UIO" ]; then
    UIO=/dev/$(basename "$(dirname "$HAVE_UIO")")
    echo "== B1 uio as casper ($UIO): rootless read AND write"
    sudo -u casper $PY access_size_scaling.py --transport uio --dev "$UIO" \
        --read-addr "$READ_ADDR" --write-addr "$WRITE_ADDR" --label "$STAGE uio" \
        --out "$OUT/b1_uio_acc.json" > "$OUT/b1_uio_acc.txt" 2>&1 \
        || echo "   FAILED, see $OUT/b1_uio_acc.txt"
    grep -E "^fit|euid" "$OUT/b1_uio_acc.txt"
fi
if [ -e /dev/udmabuf1 ]; then
    # ZDMA only against the AXI4 BRAM it was validated on: a DMA burst into an
    # AXI-lite slave is untested, and a bus hang here needs someone at the board.
    scale zdma_acc      --transport zdma --read-addr "$READ_ADDR"
    # whole LO register written by devmem vs by DMA from a u-dma-buf (verified by read-back)
    scale lo_full_write --transport devmem --read-addr "$WRITE_ADDR" --write-addr "$WRITE_ADDR" \
        --write-bytes "$WRITE_SIZE" --zdma-write
    scale dram_uncached --transport dram-uncached
    scale dram_cached   --transport dram-cached
fi

echo "== B2 sweep-step profile (1000 tones / 1000 chans / 10000 steps)"
STEP="--write-addr $WRITE_ADDR --read-addr $READ_ADDR --steps 10000 --tones 1000 --read-chans 1000"
$PY devmem_profile.py --dev /dev/mem $STEP --out "$OUT/b2_py.txt" | grep -E "write|read |step"
./devmem_profile --dev /dev/mem $STEP --bulk --out "$OUT/b2_c_bulk.txt" | grep -E "write|read |step"
./devmem_profile --dev /dev/mem $STEP --out "$OUT/b2_c_perword.txt" | grep -E "write|read |step"
if [ -e /dev/udmabuf1 ]; then
    $PY devmem_profile.py --dev /dev/mem $STEP --read-transport zdma \
        --out "$OUT/b2_py_zdma.txt" | grep -E "write|read |step"
fi
if [ -n "$HAVE_UIO" ]; then
    sudo -u casper $PY devmem_profile.py --dev "$UIO" $STEP \
        --out "$OUT/b2_py_uio.txt" | grep -E "write|read |step"
fi

echo "== B3 read path, end to end"
$PY fast_path_timing.py --out "$OUT/b3_read_path.json" > "$OUT/b3_read_path.txt" 2>&1 \
    || echo "   FAILED, see $OUT/b3_read_path.txt"
cat "$OUT/b3_read_path.txt"
