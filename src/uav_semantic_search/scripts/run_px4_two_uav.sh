#!/usr/bin/env bash
# Launch PX4 Gazebo Classic multi-SITL with the Stage-2 corridor world.
# Required first terminal: roslaunch uav_semantic_search semantic_stage2.launch
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-$HOME/PX4_Firmware_13}"
[[ -d "$PX4_ROOT" ]] || { echo "PX4_ROOT not found: $PX4_ROOT" >&2; exit 1; }

source /opt/ros/noetic/setup.bash
if ! rosparam get /rosversion >/dev/null 2>&1; then
  echo "ROS master is unavailable. Start semantic_stage2.launch or stack.launch first." >&2
  exit 1
fi

if [[ -f "$PX4_ROOT/Tools/setup_gazebo.bash" ]]; then
  source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
elif [[ -f "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]]; then
  source "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
else
  echo "Gazebo Classic setup script not found under $PX4_ROOT/Tools" >&2
  exit 1
fi

# Required for gazebo_ros_api_plugin, RGB-D camera, and Velodyne point-cloud plugins.
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"

REAL_GZSERVER="$(command -v gzserver)"
[[ -n "$REAL_GZSERVER" ]] || { echo "gzserver was not found." >&2; exit 1; }
WRAP_DIR="${TMPDIR:-/tmp}/uav_semantic_search_gzserver_wrapper"
mkdir -p "$WRAP_DIR"
cat > "$WRAP_DIR/gzserver" <<WRAP
#!/usr/bin/env bash
exec "$REAL_GZSERVER" -s libgazebo_ros_api_plugin.so "\$@"
WRAP
chmod +x "$WRAP_DIR/gzserver"
export PATH="$WRAP_DIR:$PATH"

# Explicitly place both vehicles inside the 4 m wide corridor.
# exec "$PX4_ROOT/Tools/gazebo_sitl_multiple_run.sh" \
#   -w corridor_rooms \
#   -s "iris:1:2.0:-0.7,iris:1:2.0:0.7"

# corridor_rooms 
# h1_hospital_ugv 
# h2_industrial_ring_uav 
# h3_warehouse_shelves_ugv
# h4_multibranch_tunnel_uav
WORLD_NAME="$(rosparam get /experiment/world_name 2>/dev/null || true)"
UAV_SPAWN_SPEC="$(rosparam get /experiment/uav_spawn_spec 2>/dev/null || true)"

if [[ -z "$WORLD_NAME" ]]; then
  echo "Missing ROS parameter: /experiment/world_name" >&2
  echo "Start human_ai_vlm_stage5.launch before this script." >&2
  exit 1
fi

if [[ -z "$UAV_SPAWN_SPEC" ]]; then
  echo "Missing ROS parameter: /experiment/uav_spawn_spec" >&2
  exit 1
fi

echo "Starting Gazebo world: $WORLD_NAME"
echo "UAV spawn configuration: $UAV_SPAWN_SPEC"

exec "$PX4_ROOT/Tools/gazebo_sitl_multiple_run.sh" \
  -w "$WORLD_NAME" \
  -s "$UAV_SPAWN_SPEC"
