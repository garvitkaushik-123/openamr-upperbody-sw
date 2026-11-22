#!/bin/sh
# SPDX-License-Identifier: MIT
# Create the virtual CAN interfaces of the OpenArm software-in-the-loop path, one per arm, and report what was done.
#
#   setup_vcan.sh [--delete] [interface ...]        default interfaces: vcan_arm_l vcan_arm_r
#
# For each interface: load the vcan module if needed, add the link, try MTU 72 (CAN FD frames), bring it up.
# Needs root or passwordless sudo. On Ubuntu the vcan module is in linux-modules-extra-$(uname -r).
#
# Output: one line per interface, then CAN_FD=yes when every interface carries CAN FD frames, else CAN_FD=no.
# Exit status: 0 every interface is up; 3 BLOCKED, an interface could not be created (the reason is printed).
# vcan has no bit rate, no sample point and no bus-off. The emulator models frame time in software (README).
set -u

DELETE=0
if [ "${1:-}" = "--delete" ]; then DELETE=1; shift; fi
[ $# -eq 0 ] && set -- vcan_arm_l vcan_arm_r

SUDO=""
if [ "$(id -u)" != "0" ]; then SUDO="sudo -n"; fi

if ! command -v ip >/dev/null 2>&1; then
  echo "BLOCKED: ip (iproute2) is not installed"
  exit 3
fi

if [ "$DELETE" = "1" ]; then
  for IFACE in "$@"; do
    if [ -e "/sys/class/net/$IFACE" ]; then $SUDO ip link delete "$IFACE" && echo "$IFACE: deleted"; fi
  done
  exit 0
fi

FD=yes
for IFACE in "$@"; do
  if [ ! -e "/sys/class/net/$IFACE" ]; then
    $SUDO modprobe vcan 2>/dev/null
    if ! ERR=$($SUDO ip link add dev "$IFACE" type vcan 2>&1); then
      echo "BLOCKED: cannot create $IFACE: $ERR"
      echo "  kernel $(uname -r): is the vcan module available (linux-modules-extra-$(uname -r)), and is this root or passwordless sudo?"
      echo "CAN_FD=no"
      exit 3
    fi
    CREATED="created"
  else
    CREATED="already there"
  fi
  if [ "$(cat "/sys/class/net/$IFACE/mtu")" != "72" ]; then
    # The MTU can only change while the link is down.
    $SUDO ip link set "$IFACE" down 2>/dev/null
    $SUDO ip link set "$IFACE" mtu 72 2>/dev/null
  fi
  if ! ERR=$($SUDO ip link set "$IFACE" up 2>&1); then
    echo "BLOCKED: cannot bring $IFACE up: $ERR"
    echo "CAN_FD=no"
    exit 3
  fi
  MTU=$(cat "/sys/class/net/$IFACE/mtu")
  if [ "$MTU" = "72" ]; then
    echo "$IFACE: $CREATED, up, CAN FD yes (mtu 72)"
  else
    echo "$IFACE: $CREATED, up, CAN FD NO (mtu $MTU): this kernel's vcan carries classic CAN frames only"
    FD=no
  fi
done
echo "CAN_FD=$FD"
exit 0
