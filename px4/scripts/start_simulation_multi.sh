#!/bin/bash

cat <<EOF | bash >/dev/null &

cd /usr/local/gzweb/
sleep 1
npm start 

EOF

cd /home/user/Firmware/
Tools/gazebo_sitl_multiple_run.sh -m typhoon_h480 -n $NUMBER_OF_DRONES

# sleep 2
