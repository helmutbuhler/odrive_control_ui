// Network communication with control_ui

#include "server.h"
#include <cerrno>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <vector>

#include "../common/common.h"
#include "../common/network.h"
#include "main.h"


SOCKET client = INVALID_SOCKET;
SOCKET server = INVALID_SOCKET;

// We send the monitor data every frame to the client. In case the wifi
// becomes busy, we do not want to wait until it becomes free, because that
// will also block the pid loop and cause a (literal) crash.
// So we buffer the data here:
std::vector<char> send_buffer;
char receive_buffer[sizeof(ControlData)];
int receive_buffer_pos = 0;

bool server_init(const Params& params)
{
	assert(server == INVALID_SOCKET);
	assert(client == INVALID_SOCKET);

	server = net_listen(params.port, true, &running);
	if (server == INVALID_SOCKET)
	{
		printf("Failed create server!\n");
		return false;
	}
	net_set_socket_non_blocking(server);
	return true;
}

void disconnect_client()
{
	if (client != INVALID_SOCKET)
	{
		net_close_socket(client);
		client = INVALID_SOCKET;
		printf("Closed\n");
		if (cd.stop_motors_on_disconnect)
		{
			for (int a = 0; a < monitor_axes; a++)
				cd.axes[a].enable_motor = false;
		}

	}
	send_buffer.clear();
	send_buffer.shrink_to_fit(); // free memory
	receive_buffer_pos = 0;
}

bool server_update()
{
	u32_micros start_time = time_micros();

	if (client == INVALID_SOCKET)
	{
		client = net_accept(server, true);
		if (client != INVALID_SOCKET)
		{
			int header[2] = {sizeof(MonitorData), sizeof(ControlData)};
			net_send_all(client, header, 8);
			net_send_all(client, &cd, sizeof(ControlData));
			net_set_socket_non_blocking(client);
		}
	}

	while (client != INVALID_SOCKET)
	{
		int r = net_recv(client, receive_buffer+receive_buffer_pos, sizeof(cd)-receive_buffer_pos);
		if (r == -1)
			break;
		if (r == 0)
		{
			disconnect_client();
			break;
		}
		receive_buffer_pos += r;
		assert(receive_buffer_pos <= sizeof(cd));
		if (receive_buffer_pos == sizeof(cd))
		{
			memcpy(&cd, receive_buffer, sizeof(cd));
			receive_buffer_pos = 0;
		}
	}

	int send_buffer_pos = (int)send_buffer.size();
	send_buffer.resize(send_buffer_pos+sizeof(md));
	memcpy(send_buffer.data()+send_buffer_pos, &md, sizeof(md));

	//send MonitorData if the client is connected
	if (client != INVALID_SOCKET)
	{
		while (send_buffer.size())
		{
			int r = net_send(client, send_buffer.data(), (int)send_buffer.size());
			if (r == -1)
				break;
			if (r == 0)
			{
				printf("send fail %d %d\n", r, (int)send_buffer.size());
				disconnect_client();
				break;
			}
			memmove(send_buffer.data(), send_buffer.data()+r, send_buffer.size()-r);
			send_buffer.resize(send_buffer.size()-r);
		}
	}

	md.delta_time_network = time_micros() - start_time;
	return true;
}

void server_close()
{
	disconnect_client();
	if (server != INVALID_SOCKET)
	{
		net_close_socket(server);
		server = INVALID_SOCKET;
	}
}
