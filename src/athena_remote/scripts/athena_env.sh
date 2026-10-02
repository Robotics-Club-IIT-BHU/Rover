#!/usr/bin/env bash
# athena_env.sh — set up native ROS 2 across a network that blocks multicast.
#
# Source this on BOTH machines (Jetson and laptop):
#
#     source athena_env.sh <OTHER_MACHINE_IP>
#
# e.g. on the laptop:   source athena_env.sh 100.101.102.103   # Jetson
#      on the Jetson:   source athena_env.sh 100.101.102.104   # laptop
#
# Use the Tailscale/VPN address if the WiFi isolates clients from each
# other; the plain LAN address if it does not.
#
# Verified on this rover: with multicast off and a single unicast peer,
# topic discovery and data both work.

if [ -z "$1" ]; then
  echo "usage: source athena_env.sh <other-machine-ip>" >&2
  echo "  (the IP of the OTHER machine, not this one)" >&2
  return 1 2>/dev/null || exit 1
fi

_ATHENA_CFG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/cyclonedds_peers.xml"
if [ ! -f "$_ATHENA_CFG" ]; then
  # installed layout
  _ATHENA_CFG="$(ros2 pkg prefix athena_remote 2>/dev/null)/share/athena_remote/config/cyclonedds_peers.xml"
fi
if [ ! -f "$_ATHENA_CFG" ]; then
  echo "cyclonedds_peers.xml not found" >&2
  return 1 2>/dev/null || exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ATHENA_PEER="$1"
export CYCLONEDDS_URI="file://${_ATHENA_CFG}"
# 42 is what the rover runs on (set in the Jetson's ~/.bashrc).  This used to
# default to 0, so any shell that skips ~/.bashrc -- ssh with a command
# attached, a cron job, a script -- silently landed on a different domain and
# saw an empty graph.  That looks exactly like a network fault.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# The daemon caches the middleware config from whenever it started, so a
# stale one will keep reporting the old (multicast) view of the graph.
ros2 daemon stop >/dev/null 2>&1

echo "ROS networking configured:"
echo "  RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "  peer               = $ATHENA_PEER"
echo "  ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "  config             = $_ATHENA_CFG"
echo
echo "Both machines need: the same ROS_DOMAIN_ID, each pointing at the other."
echo "Test with:  ros2 topic list"
unset _ATHENA_CFG
