#include "native_sim_gpio.h"
#include <string.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <zephyr/kernel.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

void native_sim_gpio_publish(uint16_t companion_port, const uint8_t *values,
			     size_t count)
{
	static int socket_fd = -1;
	struct snapshot {
		uint16_t port;
		uint8_t values[64];
		size_t count;
		int64_t sent_at;
		bool initialized;
	};
	static struct snapshot snapshots[4];
	struct snapshot *snapshot = NULL;
	struct sockaddr_in destination = {
		.sin_family = AF_INET,
		.sin_port = 0,
		.sin_addr.s_addr = htonl(INADDR_LOOPBACK),
	};

	if (values == NULL || count == 0U || count > 64U) {
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(snapshots); i++) {
		if (snapshots[i].port == companion_port || snapshots[i].port == 0U) {
			snapshot = &snapshots[i];
			if (snapshot->port == 0U) {
				snapshot->port = companion_port;
			}
			break;
		}
	}
	if (snapshot == NULL) {
		return;
	}
	int64_t now = k_uptime_get();
	bool changed = !snapshot->initialized || snapshot->count != count ||
		memcmp(snapshot->values, values, count) != 0;
	if (!changed && now - snapshot->sent_at < 1000) {
		return;
	}
	if (socket_fd < 0) {
		socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
	}
	if (socket_fd < 0) {
		return;
	}
	destination.sin_port = htons(companion_port);
	if (sendto(socket_fd, values, count, 0, (struct sockaddr *)&destination,
		   sizeof(destination)) == (ssize_t)count) {
		memcpy(snapshot->values, values, count);
		snapshot->count = count;
		snapshot->sent_at = now;
		snapshot->initialized = true;
	}
}
#else
void native_sim_gpio_publish(uint16_t companion_port, const uint8_t *values,
			     size_t count)
{
	ARG_UNUSED(companion_port);
	ARG_UNUSED(values);
	ARG_UNUSED(count);
}
#endif
