#pragma once
struct Params;
bool odrive_control_init(const Params& params);
void odrive_control_close();
bool odrive_control_update();


