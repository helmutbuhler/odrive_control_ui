#pragma once

struct Params;
bool server_init(const Params& params);
bool server_update();
void server_close();
