# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Humble + Gazebo Classic + RTAB-Map + Nav2 map_server
# Non-root development container for the path-planning homework.
#
# `colcon build` is intentionally NOT run here — do it inside the container
# so --symlink-install picks up live edits to the bind-mounted Python files.
# ─────────────────────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop-full

# Pass your host UID/GID via build args. Compose fills these from the .env
# file (which contains UID and GID lines).
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# ── 1. System + ROS dependencies ────────────────────────────────────────────
# `coreutils` brings `stat` which the entrypoint needs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        coreutils \
        bash-completion \
        nano vim less \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-numpy \
        ros-humble-rtabmap-ros \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-teleop-twist-keyboard \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf-transformations \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-plugins \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Non-root user with passwordless sudo ─────────────────────────────────
# Reuse an existing UID/GID if the base image already has one (some Ubuntu
# bases ship with a default `ubuntu` user at 1000:1000); otherwise create
# `ros`. Either way, the resulting account gets sudo with no password.
RUN set -eux; \
    if ! getent group "${USER_GID}" >/dev/null; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    if ! getent passwd "${USER_UID}" >/dev/null; then \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" \
                --create-home --shell /bin/bash "${USERNAME}"; \
    fi; \
    REAL_USER="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
    echo "${REAL_USER} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${REAL_USER}"; \
    chmod 0440 "/etc/sudoers.d/${REAL_USER}"; \
    usermod -aG video,dialout "${REAL_USER}" 2>/dev/null || true

# ── 3. Workspace skeleton (real packages bind-mounted at runtime) ──────────
RUN mkdir -p /ros2_ws/src \
 && chown -R "${USER_UID}:${USER_GID}" /ros2_ws

# ── 4. Bash environment ────────────────────────────────────────────────────
RUN REAL_USER="$(getent passwd "${USER_UID}" | cut -d: -f1)" \
 && BASHRC="/home/${REAL_USER}/.bashrc" \
 && { \
      echo ''; \
      echo '# ── ROS 2 Humble setup ─────────────────────────────────────'; \
      echo 'source /opt/ros/humble/setup.bash'; \
      echo ''; \
      echo '# Source the workspace overlay if it has been built.'; \
      echo 'if [ -f /ros2_ws/install/setup.bash ]; then'; \
      echo '    source /ros2_ws/install/setup.bash'; \
      echo 'fi'; \
      echo ''; \
      echo '# Gazebo Classic — model/resource paths from the tutorial pkg.'; \
      echo 'if [ -d /ros2_ws/install/rtabmap_diff_drive_tutorial/share ]; then'; \
      echo '    export GAZEBO_MODEL_PATH="/ros2_ws/install/rtabmap_diff_drive_tutorial/share:${GAZEBO_MODEL_PATH:-}"'; \
      echo '    export GAZEBO_RESOURCE_PATH="/ros2_ws/install/rtabmap_diff_drive_tutorial/share:${GAZEBO_RESOURCE_PATH:-}"'; \
      echo 'fi'; \
      echo '[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh'; \
      echo ''; \
      echo '# Pretty prompt — obvious we are in the container.'; \
      echo "export PS1='\\[\\033[01;32m\\]ros@planning\\[\\033[00m\\]:\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\$ '"; \
      echo ''; \
      echo '# Convenience aliases.'; \
      echo "alias build='cd /ros2_ws && colcon build --symlink-install && source install/setup.bash'"; \
      echo "alias build_one='cd /ros2_ws && colcon build --symlink-install --packages-select'"; \
      echo "alias clean='cd /ros2_ws && rm -rf build install log'"; \
      echo ''; \
      echo 'cd /ros2_ws'; \
    } >> "${BASHRC}" \
 && chown "${USER_UID}:${USER_GID}" "${BASHRC}"

# ── 5. Entrypoint that fixes mount ownership at runtime ────────────────────
# Bind-mounts created on the host before this container ran will sometimes
# be owned by root (especially if Docker had to create the directory).
# The entrypoint chowns them to the runtime user so writes from inside the
# container succeed without manual `sudo chown` on the host.
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# ── 6. Run as non-root, idle indefinitely ──────────────────────────────────
USER ${USER_UID}:${USER_GID}
WORKDIR /ros2_ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]