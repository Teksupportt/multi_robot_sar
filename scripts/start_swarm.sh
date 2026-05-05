#!/bin/bash

PX4_DIR=~/PX4-Autopilot
BUILD=$PX4_DIR/build/px4_sitl_default

# Kill any existing instances
pkill -x px4 || true
pkill gzserver || true
pkill gzclient || true
sleep 2

# Setup Gazebo environment
source $PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  $PX4_DIR $BUILD

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export PX4_SIM_MODEL=gazebo-classic_iris

# Start Gazebo server
gzserver $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world \
  --verbose &
sleep 5

# Spawn 4 drones at corner positions
POSITIONS=("0 0" "10 0" "0 10" "10 10")
for i in 1 2 3 4; do
  N=$((i-1))
  X=$(echo ${POSITIONS[$N]} | cut -d' ' -f1)
  Y=$(echo ${POSITIONS[$N]} | cut -d' ' -f2)
  PORT=$((8888+i))

  mkdir -p $BUILD/rootfs/$N
  pushd $BUILD/rootfs/$N > /dev/null

  echo "Spawning drone $i at ($X, $Y) XRCE port $PORT"

  PX4_UXRCE_DDS_PORT=$PORT \
  PX4_UXRCE_DDS_NS=drone_$((i-1)) \
  $BUILD/bin/px4 -i $i \
    -d $BUILD/etc > out.log 2> err.log &

  # Generate and spawn SDF
  python3 $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
    $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja \
    $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
    --mavlink_tcp_port $((4560+i)) \
    --mavlink_udp_port $((14560+i)) \
    --mavlink_id $((1+i)) \
    --gst_udp_port $((5600+i)) \
    --video_uri $((5600+i)) \
    --mavlink_cam_udp_port $((14530+i)) \
    --output-file /tmp/iris_$i.sdf

  gz model --spawn-file=/tmp/iris_$i.sdf \
    --model-name=iris_$i -x $X -y $Y -z 0.83

  popd > /dev/null
  sleep 2
done

# Start Gazebo client
gzclient &

echo "Swarm ready. Ports: 8889-8892"
wait
