#!/bin/sh
set -eu

mkdir -p "$XDG_RUNTIME_DIR"
chmod 0700 "$XDG_RUNTIME_DIR"

display_number=${DISPLAY#:}
display_number=${display_number%%.*}
display_socket="/tmp/.X11-unix/X${display_number}"

cleanup() {
    for pid in ${app_pid:-} ${websockify_pid:-} ${vnc_pid:-} ${openbox_pid:-} ${xvfb_pid:-}; do
        if [ -n "$pid" ]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

Xvfb "$DISPLAY" -screen 0 "$SCREEN_GEOMETRY" -ac -nolisten tcp >/tmp/xvfb.log 2>&1 &
xvfb_pid=$!

attempt=0
while [ ! -S "$display_socket" ]; do
    attempt=$((attempt + 1))
    if [ "$attempt" -ge 100 ]; then
        echo "Xvfb did not create $display_socket" >&2
        exit 1
    fi
    sleep 0.05
done

openbox-session >/tmp/openbox.log 2>&1 &
openbox_pid=$!

x11vnc \
    -display "$DISPLAY" \
    -forever \
    -localhost \
    -nopw \
    -quiet \
    -rfbport 5900 \
    -shared \
    >/tmp/x11vnc.log 2>&1 &
vnc_pid=$!

websockify --web=/usr/share/novnc/ 6080 localhost:5900 >/tmp/websockify.log 2>&1 &
websockify_pid=$!

./road_intersection &
app_pid=$!
wait "$app_pid"
