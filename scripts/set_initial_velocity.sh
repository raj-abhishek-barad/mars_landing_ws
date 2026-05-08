#!/bin/bash
# Set lander initial velocity after simulation starts
# Must be run after Gazebo is unpaused

CASE=${1:-3}

case $CASE in
  1)
    VX=26.78; VY=24.08; VZ=-15.36
    PX=-1639; PY=-919; PZ=1283
    ;;
  2)
    VX=10.0; VY=-28.5; VZ=-105.0
    PX=669; PY=-819; PZ=2883
    ;;
  3)
    VX=15.0; VY=25.0; VZ=-80.0
    PX=900; PY=800; PZ=2200
    ;;
  *)
    echo "Usage: $0 [1|2|3]"
    exit 1
    ;;
esac

echo "Setting Case $CASE: pos=[$PX,$PY,$PZ] vel=[$VX,$VY,$VZ]"

gz service -s /world/mars_world/set_entity_state \
  --reqtype gz.msgs.EntityState \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: \"lander::body\", \
         velocity: {linear: {x: $VX, y: $VY, z: $VZ}}"

echo "Done."
