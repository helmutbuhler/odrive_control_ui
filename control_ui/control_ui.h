#pragma once
#include "../common/common.h"
struct Plot;

struct MonitorDataEx : MonitorData
{
	// Some additional variables for simulation and visualization stuff.
	// This is not used on the proxy, only in control ui.
	s64 display_time = 0;
};


extern float dpi_scaling;
extern int window_size_x, window_size_y;

extern Plot* main_plot;

void draw_ui_main_2();
