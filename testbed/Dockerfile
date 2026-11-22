ARG ZEPHYR_IMAGE=ghcr.io/jaster-prj/devcontainer-zephyr:latest
FROM ${ZEPHYR_IMAGE}

USER root
WORKDIR /workspace

COPY --from=relaismodule-src . /workspace/application
COPY --from=native-udp-src . /workspace/zephyr-native-linux-udp-can
RUN sed -i '/^CONFIG_CAN_MAX_FILTER=/d' /workspace/application/prj.conf
RUN mkdir -p /workspace/application/dts/bindings \
    && printf '%s\n' \
      'description: GPIO pin to switch a relay output' \
      'compatible: "power-switch"' \
      'properties:' \
      '  gpios:' \
      '    type: phandle-array' \
      '    required: true' \
      > /workspace/application/dts/bindings/power-switch.yaml
RUN /opt/venv/bin/west build -b native_sim /workspace/application \
      --build-dir /workspace/application/build/native_sim \
      -- -DZEPHYR_EXTRA_MODULES="/workspace/canopennodezephyr;/workspace/zephyr-native-linux-udp-can" \
          -DEXTRA_CONF_FILE=/workspace/application/boards/native_sim.conf \
          -DEXTRA_DTC_OVERLAY_FILE=/workspace/application/boards/native_sim.overlay

CMD ["/workspace/application/build/native_sim/zephyr/zephyr.exe"]
