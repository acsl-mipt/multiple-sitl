pkill gz
pkill gzclient
pkill gzserver
pkill gazebo
pkill roslaunch
pkill roscore
pkill micrortps_agent
pkill -f "xterm -T mavros"
pkill -f "xterm -T gazebo"
pkill -f "xterm -T gz"
pkill -f "xterm -T rtps"
pkill -f "settings.json"
`dirname $0`/kill_px4.sh
