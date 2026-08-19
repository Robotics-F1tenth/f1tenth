#!/bin/bash
pkill -9 -f "[r]os2 bag play" 2>/dev/null
pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[r]viz2" 2>/dev/null
sleep 2
exit 0
