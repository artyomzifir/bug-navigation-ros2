#!/usr/bin/env bash
# Runs as the non-root user on every container start. Fixes ownership of
# bind-mounted directories that Docker may have created as root, then drops
# into the requested command.
set -e

USER_NAME="$(id -un)"
USER_UID="$(id -u)"
USER_GID="$(id -g)"

# Paths that must be writable by the non-root user.
fix_paths=(
    "/home/${USER_NAME}/.ros"
    "/ros2_ws/install"
    "/ros2_ws/build"
    "/ros2_ws/log"
)

for p in "${fix_paths[@]}"; do
    # Create if missing — Docker would otherwise make it root-owned on first
    # access from inside the container.
    if [ ! -e "$p" ]; then
        sudo mkdir -p "$p"
    fi
    # Re-chown only if needed; avoids touching every file on every start.
    owner_uid="$(stat -c '%u' "$p")"
    if [ "$owner_uid" != "$USER_UID" ]; then
        sudo chown -R "${USER_UID}:${USER_GID}" "$p"
    fi
done

exec "$@"