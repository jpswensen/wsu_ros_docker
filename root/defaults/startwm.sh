#!/bin/bash

# setterm blank 0
# setterm powerdown 0
# gsettings set org.mate.Marco.general compositing-manager false
# /usr/bin/mate-session > /dev/null 2>&1

setterm blank 0
setterm powerdown 0
gsettings set org.mate.Marco.general compositing-manager false

# Run the MATE session as me485 using sudo (without password)
exec sudo -u me485 dbus-launch --exit-with-session mate-session > /dev/null 2>&1
