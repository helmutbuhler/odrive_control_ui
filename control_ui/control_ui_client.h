#pragma once

#include "../common/helper.h"

bool client_update();
bool client_init();
void client_close();
void client_disconnect();

int client_get_connection_state();
void client_connect(const char* address, u16 port);

struct MonitorData;

void on_new_monitor_data(MonitorData& md);
