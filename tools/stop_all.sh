#!/usr/bin/env bash
# Stop everything this project started: ros2 launches, the framework/driver nodes, llama-server, dash/web_dash.
#   tools/stop_all.sh -n     only list what would be stopped
#   tools/stop_all.sh        Ctrl+C (SIGINT) everything, wait up to 8 s, then SIGKILL what is left
# Matches only this workspace's install, ~/ros2_ws drivers (kobuki, sllidar), `ros2 launch` and llama-server.
PAT='ros2-jazzy-robocup-fw/install|ros2_ws/install|bin/ros2 launch|llama-server'
list() { ps -eo pid,args | grep -E "$PAT" | grep -vE "grep|stop_all|bash -c" | awk '{print $1}'; }
show() { for p in $(list); do printf '%7s  %s\n' "$p" "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | cut -c1-110)"; done; }

if [ -z "$(list)" ]; then echo "nothing running"; exit 0; fi
show
[ "$1" = "-n" ] && exit 0
kill -INT $(list) 2>/dev/null
for _ in $(seq 1 16); do [ -z "$(list)" ] && break; sleep 0.5; done
if [ -n "$(list)" ]; then echo "still running after 8 s, sending SIGKILL:"; show; kill -KILL $(list) 2>/dev/null; fi
echo "stopped"
