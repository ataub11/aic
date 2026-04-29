FROM docker.io/library/ros:kilted-ros-core AS build

# Build-time SHA injection — submit.sh passes the current git rev so every
# image is self-identifying.  ANT.py reads ANT_BUILD_VERSION at startup and
# logs it, so every eval log starts with the SHA that's actually running.
ARG BUILD_VERSION=unknown
ENV ANT_BUILD_VERSION=${BUILD_VERSION}

# Install pixi — no version pin needed; lock file is self-contained and
# the docker-specific pixi.toml/pixi.lock live alongside this Dockerfile.
RUN apt update && apt install -y git && \
    curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/root/.pixi/bin:$PATH"

# Add workspace source packages needed at inference time.
# aic_utils (lerobot_robot_aic, aic_teleoperation) is intentionally excluded —
# those packages caused ros2-distro-mutex/libprotobuf conflicts and are not
# needed to run the ANT policy.
COPY aic_example_policies /ws_aic/src/aic/aic_example_policies
COPY aic_model /ws_aic/src/aic/aic_model
COPY aic_interfaces /ws_aic/src/aic/aic_interfaces
COPY ant_policy_node /ws_aic/src/aic/ant_policy_node

# Use the minimal Docker-specific pixi.toml/pixi.lock (next to this Dockerfile).
# opencv is installed via pip (opencv-python-headless) to avoid the libprotobuf
# conflict between opencv conda and ros2-distro-mutex 0.14.0.
COPY docker/ant_policy_node/pixi.toml /ws_aic/src/aic/pixi.toml
COPY docker/ant_policy_node/pixi.lock /ws_aic/src/aic/pixi.lock
COPY pixi_env_setup.sh robostack_mapping.json /ws_aic/src/aic/

SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/root/.cache/rattler/cache --mount=type=cache,target=/ws_aic/src/aic/.pixi/build \
    cd /ws_aic/src/aic && pixi install --locked

WORKDIR /ws_aic/src/aic
COPY --chmod=755 <<"EOF" /entrypoint.sh
#!/bin/bash
set -e

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Competition sets AIC_MODEL_ROUTER_ADDR; local docker-compose uses AIC_ROUTER_ADDR.
ROUTER_ADDR="${AIC_MODEL_ROUTER_ADDR:-$AIC_ROUTER_ADDR}"
if [[ -z "$ROUTER_ADDR" ]]; then
  echo "Either AIC_MODEL_ROUTER_ADDR or AIC_ROUTER_ADDR must be provided"
  exit 1
fi

# Enable ACL only when AIC_ENABLE_ACL is explicitly set.
# This must match the eval container's should_enable_acl() exactly — if only one
# side configures usrpwd transport auth, Zenoh rejects the connection with
# "UsrPwd extension - Recv InitSyn. Decoding error."
should_enable_acl() {
  [[ "$AIC_ENABLE_ACL" == "true" || "$AIC_ENABLE_ACL" == "1" ]]
}

# prepare the credentials file
if should_enable_acl; then
  if [[ ! (-n "$AIC_MODEL_PASSWD" ) ]]; then
    echo "AIC_MODEL_PASSWD must be provided"
    exit 1
  fi
  echo "model:$AIC_MODEL_PASSWD" >> /credentials.txt
fi

ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/'"$ROUTER_ADDR"'"]'
ZENOH_CONFIG_OVERRIDE+=';listen/endpoints=["tcp/0.0.0.0:0"]'
ZENOH_CONFIG_OVERRIDE+=';transport/shared_memory/enabled=false'
if should_enable_acl; then
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="model"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_MODEL_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
fi
export ZENOH_CONFIG_OVERRIDE
echo "ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"
# Retry Zenoh router connection indefinitely — the competition evaluator and
# model containers start in parallel; the router may not be reachable on the
# first attempt.  Without this the default is 1 attempt, after which rmw_zenoh
# proceeds without a router connection and the evaluator can never reach us.
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
exec pixi run --as-is ros2 run aic_model aic_model "$@"
EOF

ENTRYPOINT ["/entrypoint.sh"]
CMD ["--ros-args", "-p", "policy:=ant_policy_node.ANT"]
