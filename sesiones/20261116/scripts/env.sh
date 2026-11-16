# source ~/ivnafra/ws/env.sh [domain_id]
source /opt/ros/humble/setup.bash
source ~/ivnafra/ws/install/setup.bash
[ -n "$1" ] && export ROS_DOMAIN_ID=$1
export TP=~/ivnafra/TP_final
export SESION=~/ivnafra/sesion_20261116
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  TP=$TP  SESION=$SESION"
