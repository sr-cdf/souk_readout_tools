#!/bin/bash
# CPU and network layout for two readout servers streaming at full speed on the
# 4-core A53 (doc/v712-notes.md, "Pushing the zero-drop rate"). Board-side, root:
#
#     sudo ./rt_tuning.sh status
#     sudo ./rt_tuning.sh apply [--journald-volatile] [--split-threads]
#     sudo ./rt_tuning.sh revert
#
# Layout applied:
#   CPU0-1  everything else: systemd default CPUAffinity, so ptp4l, tsu-strobe,
#           tcpborphserver, chrony, journald, ssh and any later process; the
#           movable kernel threads (e.g. watchdogd), unbound workqueues and new
#           IRQs. The Ethernet (end0) and SD-card (mmc1) interrupts already land
#           on CPU0.
#   CPU2    readout_server_0 alone; CPU3 readout_server_1 alone.
#           --split-threads: only each server's event-loop thread (it builds and
#           sends the frames) stays there, its other threads (request workers,
#           katcp I/O) go to CPU0-1 via SOUK_STREAM_CPUS / SOUK_WORKER_CPUS.
#           Measured worse on 2026-09-17 (GIL contention), so off by default.
#   net.core.wmem_max / rmem_max 32 MiB, so the servers' 1 MiB SO_SNDBUF request
#           is honoured instead of being capped at 212 992 B (~13 ms at 2 kHz).
#
# Deliberately NOT applied: SCHED_FIFO. The frame producers busy-wait for each
# accumulation; a spinning real-time task is either throttled by
# kernel.sched_rt_runtime_us (a 50 ms stall every second) or, with throttling
# off, starves the per-CPU kernel threads on its core (RCU stalls). Revisit
# only once the wait sleeps between accumulations.
#
# --journald-volatile keeps the journal in RAM (removed large stalls at
# 3.5-4 kHz before); the journal is then lost on reboot. Flags in any order.
set -u
[ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)"; exit 1; }
CMD=${1:-status}
SYSTEMD_CONF=/etc/systemd/system.conf.d/50-souk-cpu.conf
SYSCTL_CONF=/etc/sysctl.d/60-souk-net.conf
JOURNALD_CONF=/etc/systemd/journald.conf.d/60-souk-volatile.conf
SERVERS="readout_server_0:2 readout_server_1:3"
HOUSEKEEPING=0,1

status() {
    echo "== processes (PSR = CPU last run on, allowed CPUs, policy)"
    for p in $(pgrep -f "readout-server|tcpborphserver|ptp4l|tsu-strobe|chronyd|systemd-journald|timing_monitor"); do
        printf "%6s %-18s psr=%-2s allowed=%-8s %s\n" "$p" "$(ps -o comm= -p "$p")" \
            "$(ps -o psr= -p "$p" | tr -d ' ')" "$(taskset -pc "$p" 2>/dev/null | awk '{print $NF}')" \
            "$(ps -o cls=,ni=,rtprio= -p "$p")"
    done
    echo "== readout server threads (thread id: allowed CPUs)"
    for p in $(pgrep -f "souk-readout-server -p"); do
        printf "  %s:" "$(tr '\0' ' ' < /proc/$p/cmdline | grep -o -- '-p [01]')"
        for t in /proc/$p/task/*; do printf " %s:%s" "$(basename $t)" "$(taskset -pc "$(basename $t)" | awk '{print $NF}')"; done
        echo
    done
    echo "== kernel threads on CPU2-3 that could be moved"
    for p in $(ps -eo pid=,args= | awk '$2 ~ /^\[/ {print $1}'); do
        a=$(taskset -pc "$p" 2>/dev/null | awk '{print $NF}')
        case "$a" in *2*|*3*) [ "$a" != "2" ] && [ "$a" != "3" ] && printf "  %s %s allowed=%s\n" "$p" "$(ps -o comm= -p $p)" "$a";; esac
    done
    echo "== workqueue / default IRQ affinity"
    echo "  $(cat /sys/devices/virtual/workqueue/cpumask 2>/dev/null) / $(cat /proc/irq/default_smp_affinity)"
    echo "== interrupts per CPU (end0 = Ethernet, mmc1 = SD card)"
    grep -E "CPU0|end0|mmc1" /proc/interrupts
    echo "== sysctl"
    sysctl net.core.wmem_max net.core.rmem_max kernel.sched_rt_runtime_us
    echo "== files"
    for f in $SYSTEMD_CONF $SYSCTL_CONF $JOURNALD_CONF /etc/systemd/system/readout_server_{0,1}.service.d/50-cpu.conf; do
        [ -e "$f" ] && echo "  present: $f" || echo "  absent:  $f"
    done
}

apply() {
    mkdir -p "$(dirname $SYSTEMD_CONF)" "$(dirname $SYSCTL_CONF)"
    printf '[Manager]\nCPUAffinity=0 1\n' > $SYSTEMD_CONF
    printf 'net.core.wmem_max = 33554432\nnet.core.rmem_max = 33554432\n' > $SYSCTL_CONF
    volatile=no; split=no
    for arg in "$@"; do
        case "$arg" in
            --journald-volatile) volatile=yes ;;
            --split-threads) split=yes ;;
            *) echo "unknown option $arg"; exit 1 ;;
        esac
    done
    for s in $SERVERS; do
        unit=${s%%:*}; cpu=${s##*:}
        mkdir -p /etc/systemd/system/$unit.service.d
        if [ $split = yes ]; then
            printf '[Service]\nCPUAffinity=%s\nEnvironment=SOUK_STREAM_CPUS=%s SOUK_WORKER_CPUS=%s\n' \
                "$cpu" "$cpu" "$HOUSEKEEPING" > /etc/systemd/system/$unit.service.d/50-cpu.conf
        else
            printf '[Service]\nCPUAffinity=%s\n' "$cpu" > /etc/systemd/system/$unit.service.d/50-cpu.conf
        fi
    done
    if [ $volatile = yes ]; then
        mkdir -p "$(dirname $JOURNALD_CONF)"
        printf '[Journal]\nStorage=volatile\n' > $JOURNALD_CONF
        systemctl restart systemd-journald
    fi
    sysctl --system > /dev/null
    systemctl daemon-reexec          # new default CPUAffinity for units started from now on
    systemctl daemon-reload
    # move what is already running: userspace, then the kernel threads that
    # allow it (per-CPU ones refuse, which is fine), unbound workqueues, new IRQs
    for p in $(ps -eo pid=,args= | awk '$2 !~ /^\[/ {print $1}'); do
        taskset -a -pc $HOUSEKEEPING "$p" > /dev/null 2>&1
    done
    for p in $(ps -eo pid=,args= | awk '$2 ~ /^\[/ {print $1}'); do
        case "$(taskset -pc "$p" 2>/dev/null | awk '{print $NF}')" in
            0-3) taskset -pc $HOUSEKEEPING "$p" > /dev/null 2>&1 ;;
        esac
    done
    echo 3 > /sys/devices/virtual/workqueue/cpumask 2>/dev/null
    echo 3 > /proc/irq/default_smp_affinity
    systemctl restart readout_server_0 readout_server_1
    sleep 3
    status
}

revert() {
    had_journald=$([ -e $JOURNALD_CONF ] && echo yes)
    rm -f $SYSTEMD_CONF $SYSCTL_CONF $JOURNALD_CONF \
          /etc/systemd/system/readout_server_{0,1}.service.d/50-cpu.conf
    sysctl -w net.core.wmem_max=212992 net.core.rmem_max=212992 > /dev/null
    [ "$had_journald" = yes ] && systemctl restart systemd-journald
    systemctl daemon-reexec
    systemctl daemon-reload
    for p in $(ps -eo pid=,args= | awk '$2 !~ /^\[/ {print $1}'); do
        taskset -a -pc 0-3 "$p" > /dev/null 2>&1
    done
    for p in $(ps -eo pid=,args= | awk '$2 ~ /^\[/ {print $1}'); do
        [ "$(taskset -pc "$p" 2>/dev/null | awk '{print $NF}')" = "0,1" ] && taskset -pc 0-3 "$p" > /dev/null 2>&1
    done
    echo f > /sys/devices/virtual/workqueue/cpumask 2>/dev/null
    echo f > /proc/irq/default_smp_affinity
    systemctl restart readout_server_0 readout_server_1
    sleep 3
    status
}

case "$CMD" in
    status) status ;;
    apply)  shift; apply "$@" ;;
    revert) revert ;;
    *) echo "usage: sudo $0 status | apply [--journald-volatile] [--split-threads] | revert"; exit 1 ;;
esac
