#!/usr/bin/env bash

# Source this file on every ROS 2 host participating in the same experiment.
# Example: source scripts/network_env.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This file must be sourced: source scripts/network_env.sh" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="0"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Optional fallback for networks where multicast discovery is blocked.
# Set ARM_O7_PEER_IP to the other host's IPv4 address before sourcing.
if [[ -n "${ARM_O7_PEER_IP:-}" ]]; then
  export ROS_STATIC_PEERS="$ARM_O7_PEER_IP"
fi

echo "[network] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[network] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[network] ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "[network] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[network] ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-none}"
