#!/bin/bash
# Board tuning for the KRM (two readout pipelines on the RFSoC's four A53
# cores). Specific to that board: it assumes two servers, readout_server_0 and
# readout_server_1, and the Ethernet interface end0 with two transmit queues.
# Leave it off on other platforms (e.g. the RFSoC 4x2); nothing else depends on
# it. Run through the console scripts, or directly as root:
#
#     souk-enable-tuning [--in-process] [--journald-volatile]   # = sudo rt_tuning.sh apply ...
#     souk-disable-tuning                                       # = sudo rt_tuning.sh revert
#     souk-tuning-status                                        # = sudo rt_tuning.sh status
#     sudo rt_tuning.sh kernel      # only the part a reboot undoes (see below)
#
# What 'apply' sets up:
#   CPU0-1  everything that is not a stream loop: systemd's default CPUAffinity
#           (ptp4l, tsu-strobe, tcpborphserver, chrony, journald, ssh, and each
#           readout server's own request side), kernel threads that can move,
#           unbound workqueues and new interrupts. The Ethernet and SD-card
#           interrupts already arrive on CPU0.
#   CPU2    pipeline 0's stream loop, alone; CPU3 pipeline 1's.
#           By default each stream loop runs in its own process
#           (SOUK_STREAM_PROCESS=1) at real-time priority (SOUK_STREAM_RT_PRIO=50),
#           and its server moves to CPU0-1 (SOUK_WORKER_CPUS). Real-time priority
#           is what keeps the kernel's periodic per-CPU work (the SLAB cache
#           reaper, every 2 s for 100-300 us) from delaying the loop, and it is
#           safe because the loop sleeps between accumulations instead of
#           spinning. --in-process instead keeps each whole server, stream loop
#           included, on its core at normal priority (the older layout).
#   Network: net.core.wmem_max / rmem_max 32 MiB, so each server's 1 MiB send
#           buffer is granted (the default cap is 208 KiB, ~13 ms at 2 kHz), and
#           one end0 transmit queue per stream core (tx-0 for CPU2, tx-1 for
#           CPU3). Without that, both streams can share a queue, and then one
#           stream core also sends the other's packets (~30 us a frame).
#
# The systemd and sysctl files persist across a reboot. The kernel-side settings
# (kernel threads, workqueue and interrupt masks, transmit queues) do not, so
# 'apply' also installs souk-rt-kernel.service, which runs 'kernel' at every
# boot from a copy of this script in /usr/local/sbin.
#
# --journald-volatile keeps the journal in RAM (it removed large stalls at
# 3.5-4 kHz); the journal is then lost on reboot. Options in any order.
# Background and measurements: doc/v712-notes.md.
set -u
[ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)"; exit 1; }
CMD=${1:-status}
SYSTEMD_CONF=/etc/systemd/system.conf.d/50-souk-cpu.conf
SYSCTL_CONF=/etc/sysctl.d/60-souk-net.conf
JOURNALD_CONF=/etc/systemd/journald.conf.d/60-souk-volatile.conf
SERVERS="readout_server_0:2 readout_server_1:3"
HOUSEKEEPING=0,1
BOOT_UNIT=/etc/systemd/system/souk-rt-kernel.service
BOOT_SCRIPT=/usr/local/sbin/souk-rt-tuning

kernel_affinity() {
    # kernel threads that allow it (per-CPU ones refuse, which is fine), unbound
    # workqueues, new IRQs: all to the housekeeping CPUs
    for p in $(ps -eo pid=,args= | awk '$2 ~ /^\[/ {print $1}'); do
        case "$(taskset -pc "$p" 2>/dev/null | awk '{print $NF}')" in
            0-3) taskset -pc $HOUSEKEEPING "$p" > /dev/null 2>&1 ;;
        esac
    done
    echo 3 > /sys/devices/virtual/workqueue/cpumask 2>/dev/null
    echo 3 > /proc/irq/default_smp_affinity
    # one transmit queue per stream core
    q=0
    for s in $SERVERS; do
        printf '%x' $((1 << ${s##*:})) > /sys/class/net/end0/queues/tx-$q/xps_cpus 2>/dev/null
        q=$((q + 1))
    done
}

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
    echo "== end0 transmit queue CPUs (XPS)"
    for f in /sys/class/net/end0/queues/tx-*/xps_cpus; do echo "  $(basename $(dirname $f)): $(cat $f)"; done
    echo "== interrupts per CPU (end0 = Ethernet, mmc1 = SD card)"
    grep -E "CPU0|end0|mmc1" /proc/interrupts
    echo "== sysctl"
    sysctl net.core.wmem_max net.core.rmem_max kernel.sched_rt_runtime_us
    echo "== files"
    for f in $SYSTEMD_CONF $SYSCTL_CONF $JOURNALD_CONF $BOOT_UNIT $BOOT_SCRIPT /etc/systemd/system/readout_server_{0,1}.service.d/50-cpu.conf; do
        [ -e "$f" ] && echo "  present: $f" || echo "  absent:  $f"
    done
    echo "== readout server environment (from all drop-ins)"
    for s in $SERVERS; do
        echo "  ${s%%:*}: $(systemctl show -p Environment --value ${s%%:*} 2>/dev/null)"
    done
}

apply() {
    mkdir -p "$(dirname $SYSTEMD_CONF)" "$(dirname $SYSCTL_CONF)"
    printf '[Manager]\nCPUAffinity=0 1\n' > $SYSTEMD_CONF
    printf 'net.core.wmem_max = 33554432\nnet.core.rmem_max = 33554432\n' > $SYSCTL_CONF
    volatile=no; process=yes
    for arg in "$@"; do
        case "$arg" in
            --journald-volatile) volatile=yes ;;
            --in-process) process=no ;;
            *) echo "unknown option $arg"; exit 1 ;;
        esac
    done
    for s in $SERVERS; do
        unit=${s%%:*}; cpu=${s##*:}
        mkdir -p /etc/systemd/system/$unit.service.d
        if [ $process = yes ]; then
            printf '[Service]\nCPUAffinity=%s\nEnvironment=SOUK_STREAM_CPUS=%s SOUK_WORKER_CPUS=%s SOUK_STREAM_PROCESS=1 SOUK_STREAM_RT_PRIO=50\n' \
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
    # move what is already running: userspace, then the kernel side
    for p in $(ps -eo pid=,args= | awk '$2 !~ /^\[/ {print $1}'); do
        taskset -a -pc $HOUSEKEEPING "$p" > /dev/null 2>&1
    done
    kernel_affinity
    # ... and the kernel side again at every boot, before the servers start
    install -m 755 "$0" $BOOT_SCRIPT
    printf '[Unit]\nDescription=SOUK: kernel threads, workqueues and IRQs to CPU0-1, XPS (rt_tuning.sh)\nAfter=sysinit.target\nBefore=readout_server_0.service readout_server_1.service\n\n[Service]\nType=oneshot\nExecStart=%s kernel\n\n[Install]\nWantedBy=multi-user.target\n' \
        $BOOT_SCRIPT > $BOOT_UNIT
    systemctl daemon-reload
    systemctl enable souk-rt-kernel.service
    systemctl restart readout_server_0 readout_server_1
    sleep 3
    status
}

revert() {
    had_journald=$([ -e $JOURNALD_CONF ] && echo yes)
    systemctl disable souk-rt-kernel.service 2>/dev/null
    rm -f $BOOT_UNIT $BOOT_SCRIPT
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
    for f in /sys/class/net/end0/queues/tx-*/xps_cpus; do echo 0 > $f; done
    systemctl restart readout_server_0 readout_server_1
    sleep 3
    status
}

case "$CMD" in
    status) status ;;
    apply)  shift; apply "$@" ;;
    kernel) kernel_affinity ;;
    revert) revert ;;
    *) echo "usage: sudo $0 status | apply [--in-process] [--journald-volatile] | kernel | revert"; exit 1 ;;
esac
