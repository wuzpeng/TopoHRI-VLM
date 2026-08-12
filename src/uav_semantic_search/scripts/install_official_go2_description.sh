#!/usr/bin/env bash
# Install the official Unitree Go2 URDF description package into a ROS1 catkin workspace.
# This revision intentionally avoids `git clone --sparse`, which is unreliable on some
# Ubuntu 20.04 / Git 2.25 installations.
set -euo pipefail

WS_ROOT="${1:-$HOME/harp_sar_ws}"
SRC_DIR="$WS_ROOT/src"
TARGET="$SRC_DIR/go2_description"
REPO_URL="${GO2_REPO_URL:-https://github.com/unitreerobotics/go2_urdf.git}"

[[ -d "$SRC_DIR" ]] || {
  echo "Catkin src directory not found: $SRC_DIR" >&2
  exit 1
}

command -v git >/dev/null 2>&1 || {
  echo "git is not installed. Install it first: sudo apt install git" >&2
  exit 1
}

if [[ -e "$TARGET" ]]; then
  echo "Target already exists: $TARGET" >&2
  echo "Move or remove it before installing a fresh official Go2 description package." >&2
  exit 1
fi

printf 'Downloading official Unitree Go2 URDF description package...\n'
printf 'Repository: %s\n' "$REPO_URL"

# Avoid inherited Git environment variables from another repository or launcher shell.
env -u GIT_DIR -u GIT_WORK_TREE -u GIT_COMMON_DIR \
  git clone --depth 1 "$REPO_URL" "$TARGET"

[[ -f "$TARGET/package.xml" ]] || {
  echo "Downloaded repository does not contain package.xml: $TARGET" >&2
  rm -rf "$TARGET"
  exit 1
}

if [[ ! -d "$TARGET/urdf" && ! -d "$TARGET/xacro" ]]; then
  echo "Downloaded repository does not contain expected Go2 URDF/Xacro resources." >&2
  rm -rf "$TARGET"
  exit 1
fi

printf 'Official Go2 description installed to: %s\n' "$TARGET"
printf 'Next run:\n'
printf '  cd %s && catkin_make && source devel/setup.bash\n' "$WS_ROOT"
printf '  rospack find go2_description\n'
