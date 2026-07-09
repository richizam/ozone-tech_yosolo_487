#!/usr/bin/env bash
# Stop the evidence chain cleanly: kill the chain process group, then remove
# any sim containers it left running (killing the docker CLIENT does not stop
# a container). The idle isaac-sim streaming container is never touched.
if [ -f /root/chain.pid ]; then
  kill -- -"$(cat /root/chain.pid)" 2>/dev/null
  rm -f /root/chain.pid
fi
sleep 1
docker ps --format '{{.Names}}' \
  | grep -E '^(isaacrun|isaacrec|isaacgate|probe)-' \
  | xargs -r docker rm -f
docker ps --format '{{.Names}} {{.Status}}'
