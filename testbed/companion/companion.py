"""Forward native-sim GPIO snapshots to the dashboard-only MQTT broker."""

import logging
import os
import socket
import threading
import time

import paho.mqtt.client as mqtt

LOG = logging.getLogger("gpio-state-companion")


def receive(port: int, topic: str, client: mqtt.Client, stop: threading.Event) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    LOG.info("Forwarding native GPIO UDP port %d to %s", port, topic)
    while not stop.is_set():
        try:
            payload, _ = sock.recvfrom(64)
        except TimeoutError:
            continue
        if payload and all(value in (0, 1) for value in payload):
            client.publish(topic, payload, qos=1, retain=True)
        else:
            LOG.warning("Ignoring invalid GPIO snapshot on UDP port %d", port)
    sock.close()


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    garden = os.environ.get("GARDEN_ID", "garden-1")
    specs = os.environ["GPIO_TOPICS"].split(",")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=os.environ["MQTT_CLIENT_ID"],
                         clean_session=True)
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    client.connect(os.environ.get("MQTT_HOST", "dashboard-mqtt"),
                   int(os.environ.get("MQTT_PORT", "1883")), keepalive=30)
    client.loop_start()
    stop = threading.Event()
    workers = []
    try:
        for spec in specs:
            port_text, topic_path = spec.split("=", 1)
            topic = topic_path.replace("{garden}", garden)
            worker = threading.Thread(
                target=receive, args=(int(port_text), topic, client, stop), daemon=True
            )
            worker.start()
            workers.append(worker)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=1)
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
