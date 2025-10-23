#!/bin/bash

# setterm blank 0
# setterm powerdown 0
# gsettings set org.mate.Marco.general compositing-manager false
# /usr/bin/mate-session > /dev/null 2>&1

setterm blank 0
setterm powerdown 0
gsettings set org.mate.Marco.general compositing-manager false

# Start a VNC server (x11vnc) against the current X display so a native VNC client can connect on 5901.
# We run it as the 'me485' user and in the background. If it's already running, the command is skipped.

# Ensure DISPLAY is set; KasmVNC typically sets :1
export DISPLAY="${DISPLAY:-:1}"

# Wait briefly for the X socket to be available
XNUM="${DISPLAY#:}"
for i in {1..30}; do
	if [ -S "/tmp/.X11-unix/X${XNUM}" ]; then
		break
	fi
	sleep 0.5
done

# Launch x11vnc if not already running
if ! pgrep -x x11vnc >/dev/null 2>&1; then
	# Start as me485; write a simple log; use provided options
	sudo -u me485 sh -c "x11vnc -display '${DISPLAY}' -rfbport 5901 -forever -shared -passwd 'me485' -noshm -noxdamage -bg -o /home/me485/.x11vnc.log" || true
fi

# Run the MATE session as me485 using sudo (without password)
exec sudo -u me485 dbus-launch --exit-with-session mate-session > /dev/null 2>&1
