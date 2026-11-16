#!/bin/bash
# Uso: session.sh stop | start <nombre_bag> | ps
source /opt/ros/humble/setup.bash
source ~/ivnafra/ws/install/setup.bash
export ROS_DOMAIN_ID=11
TP=~/ivnafra/TP_final
SESION=~/ivnafra/sesion_20261116
L=$SESION/logs
mkdir -p "$L" /tmp/slam

pids() {  # procesos de la sesion, sin contar shells
  ps -eo pid=,comm=,args= | awk '$2!="bash" && $2!="awk" && /lib\/slam\/slam_node|lib\/rviz2\/rviz2|bin\/ros2 (launch slam|bag record|topic echo \/slam)/ {print $1}'
}

case "$1" in
  ps)
    for p in $(pids); do ps -o pid=,args= -p "$p" | cut -c1-110; done ;;
  stop)
    for p in $(pids); do kill -INT "$p" 2>/dev/null; done
    sleep 8
    for p in $(pids); do kill -TERM "$p" 2>/dev/null; done
    sleep 1
    echo "restantes: $(pids | wc -l)" ;;
  start)
    bag=${2:?nombre del bag}
    cd "$SESION/bags" && setsid nohup ros2 bag record -o "$bag" /scan /odom /tf /tf_static /cmd_vel > "$L/$bag.log" 2>&1 &
    cd "$TP/slam" && setsid nohup ros2 launch slam slam.launch.py mode:=real scan_topic:=/scan \
      odom_source:=topic odom_topic:=/odom base_frame:=base_footprint output_path:=/tmp/slam/aula \
      rviz:=true config_file:=$TP/slam/config/parcial_bag_scan_match.yaml > "$L/slam_$bag.log" 2>&1 &
    sleep 12
    setsid nohup ros2 topic echo /slam/status --full-length > /tmp/slam/status.log 2>&1 &
    sleep 6
    "$0" ps ;;
esac
