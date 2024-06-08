#ifndef COMPILING_CONTROL_UI
#error COMPILING_CONTROL_UI must be defined for the control_ui project
#endif

#define _CRT_SECURE_NO_WARNINGS
#include "control_ui.h"
#include "control_ui_client.h"
#include "../common/odrive/odrive_helper.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

#ifdef _MSC_VER
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <GL/glu.h>
#else
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <unistd.h>
#include <sys/stat.h>
#endif
#include "GLFW/glfw3.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "../3rdparty/implot/implot.h"
#include "../control_ui/plot.h"

// Fonts don't compile on VS2017 and below
#ifdef _MSC_VER
#define INCLUDE_ICON_FONTS _MSC_VER >= 1920
#else
#define INCLUDE_ICON_FONTS 1
#endif
#if INCLUDE_ICON_FONTS
#include "../3rdparty/fonts/IconsFontAwesome6.h"
#include "../3rdparty/fonts/DroidSans.h"
#include "../3rdparty/fonts/fa-solid-900.h"
#endif

using namespace std;

#ifdef _MSC_VER
// link to libraries
#pragma comment(lib, "../3rdparty/glfw/lib-vc2015/glfw3dll.lib")
#pragma comment (lib, "opengl32.lib") // link with Microsoft OpenGL lib
#pragma comment (lib, "Glu32.lib")
#else
#define MessageBeep(x)
#pragma GCC diagnostic ignored "-Wunused-result"
#endif

// window
GLFWwindow* window;
int window_size_x, window_size_y;
float dpi_scaling;
bool pressing_key[GLFW_KEY_LAST] = {0};

ControlData cd;

static std::vector<MonitorDataEx> history;

Plot* main_plot = plot_create();

Plot_Y_Axis unit_pos    {"rev", -5, 5};
Plot_Y_Axis unit_vel    {"rev/s", -10, 10};
Plot_Y_Axis unit_current{"A", -50, 50};

void do_emergency_stop()
{
	for (int a = 0; a < monitor_axes; a++)
		cd.axes[a].enable_motor = false;
	cd.counter++;
}

MonitorData& get_last_monitor_data()
{
	assert(history.size());
	return history.back();
}

void push_history(const MonitorData& md_)
{
	MonitorDataEx md;
	(MonitorData&)md = md_;
	const s64 odrive_delta_time = 1;
	if (history.size() == 0)
		md.display_time = 0;
	else
	{
		s64 delta = md.odrive_counter-history.back().odrive_counter;
		if (delta <= 0)
		{
			// If odrive is disabled, or we add this during simulation, the counter doesn't increase
			delta = cd.target_delta_time_ms * odrive_frequency / 1000;
		}
		if (delta > odrive_frequency)
		{
			delta = odrive_frequency;
		}
		md.display_time = history.back().display_time + delta;
	}
	history.push_back(md);
}

template<typename T>
void cd_change(T& old_value, T new_value)
{
	if (old_value != new_value)
	{
		old_value = new_value;
		cd.counter++;
	}
}

int oscilloscope_history_start = -1;
void on_new_monitor_data(MonitorData& md)
{
	if (history.size() == 1 && history[0].counter == 0)
	{
		// clear first dummy element
		history.clear();
	}
	if (md.oscilloscope_state == 0)
		oscilloscope_history_start = -1;
	if (md.oscilloscope_state == 1 || md.oscilloscope_state == 2)
	{
		if (md.oscilloscope_start == md.odrive_counter)
		{
			oscilloscope_history_start = (int)history.size();
		}
		else
		{
			MonitorData last_md = history.back();
			int count = md.odrive_counter-last_md.odrive_counter-1;
			for (int i = 0; i < count; i++)
			{
				last_md.odrive_counter++;
				push_history(last_md);
			}
		}
	}
	if (md.oscilloscope_state == 3)
	{
		assert(oscilloscope_history_start != -1);
		for (int i = md.oscilloscope_start; i < md.oscilloscope_end; i++)
		{
			// #osci
			float value = md.oscilloscope_transmitting[i-md.oscilloscope_start];
			const int oscilloscope_values_per_step = 8;
			int h = oscilloscope_history_start+i/oscilloscope_values_per_step;
			switch (i%oscilloscope_values_per_step)
			{
			case 0: history[h].axes[0].pos                  = value; break;
			case 1: history[h].axes[1].pos                  = value; break;
			case 2: history[h].axes[0].current_target       = value; break;
			case 3: history[h].axes[1].current_target       = value; break;
			case 4: history[h].axes[0].vel                  = value; break;
			case 5: history[h].axes[1].vel                  = value; break;
			case 6: history[h].axes[0].input_vel            = value; break;
			case 7: history[h].axes[1].input_vel            = value; break;
			}
		}
	}
	push_history(md);
}

void save_history(const char* filename)
{
#ifdef _MSC_VER
	CreateDirectory(L"../logs", 0);
#else
	mkdir("../logs", 0755);
#endif
	char buffer[128];
	sprintf_s(buffer, "../logs/%s", filename);
	FILE* log = fopen(buffer, "wb");
	if (!log)
	{
		printf("Cannot save file in %s\n", filename);
		MessageBeep(0);
		return;
	}
	fwrite(&monitor_data_version, 4, 1, log);
	int len = sizeof(MonitorData);
	fwrite(&len, 4, 1, log);
	for (int i = 0; i < history.size(); i++)
	{
		fwrite(&history[i], sizeof(MonitorData), 1, log);
	}
	fclose(log);
}

void clear_history()
{
	history.clear();
	plot_reset_display_range(main_plot);

	MonitorData md;
	md.counter = 0;
	on_new_monitor_data(md);
}

void load_history(const char* filename)
{
	clear_history();

	char buffer[128];
	sprintf_s(buffer, "../logs/%s", filename);
	FILE* log = fopen(buffer, "rb");
	if (!log)
	{
		printf("Cannot open %s for reading!\n", buffer);
		MessageBeep(0);
		return;
	}
	int version = 0;
	fread(&version, 4, 1, log);
	if (version != monitor_data_version)
	{
		printf("error reading history: unknown version %i\n", version);
		fclose(log);
		MessageBeep(0);
		return;
	}
	int len = 0;
	fread(&len, 4, 1, log);
	if (len <= 0)
	{
		printf("error reading history\n");
		fclose(log);
		MessageBeep(0);
		return;
	}
	int extra = 0;
	if (len > sizeof(MonitorData))
		extra = len - sizeof(MonitorData);
	MonitorData md;
	while (!feof(log))
	{
		size_t r = fread(&md, min(len, (int)sizeof(MonitorData)), 1, log);
		if (r == 0)
			break;
		assert(r == 1);
		push_history(md);
		if (extra)
			fseek(log, extra, SEEK_CUR);
	}
	fclose(log);

	if (history.size() == 0)
	{
		printf("history empty!\n");
		fclose(log);
		MessageBeep(0);
		return;
	}
	plot_reset_display_range(main_plot);
}



void draw_circle(float r, int num_segments)
{
    glBegin(GL_LINE_STRIP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = tau * float(ii) / float(num_segments);//get the current angle

        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component

        glVertex2f(x, y);//output vertex

    }
    glVertex2f(r, 0);
    glVertex2f(0, 0);
    glEnd();
}

void draw_joint_state(float ui_x_pos, float ui_y_pos)
{
	MonitorDataEx& md = history[plot_get_visual_selection_index(main_plot)];
	
	glPushMatrix();
	glTranslatef(ui_x_pos, ui_y_pos, 0);
	glScalef(0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);

	for (int a = 0; a < monitor_axes; a++)
	{
		glPushMatrix();
		glColor3f(0, 0, 0);
		glTranslatef(a*400.0f, 0, 0);
		glRotatef(to_degree(md.axes[a].pos*tau), 0, 0, 1);
		draw_circle(150, 5);
		draw_circle(100, 5);
		glPopMatrix();
	}

	glPopMatrix();
}

void draw_ui_monitoring(float monitor_height, float sidebar_width)
{
	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImVec2(sidebar_width, monitor_height));
	ImGui::Begin("Monitoring", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
	MonitorData& md = get_last_monitor_data();
	
	switch (client_get_connection_state())
	{
	case 0: ImGui::TextDisabled(" "); break;
	case 1: ImGui::TextDisabled("Connecting..."); break;
	case 2: ImGui::TextDisabled("Connected"); break;
	}
	if (md.oscilloscope_state)
	{
		ImGui::SameLine();
		ImGui::TextDisabled("OSC");
	}

	if (ImGui::Button("STOP (spacebar)"))
	{
		do_emergency_stop();
	}

	ImGui::End();
}

void start_exe(const char* command_line)
{
#ifdef _WIN32
    wchar_t path[1024] = { 0 };
    GetModuleFileNameW(NULL, path, 1024);
    std::filesystem::path p(path);
	p = p.parent_path();
#else
    char result[1024];
    ssize_t count = readlink("/proc/self/exe", result, 1024);
    std::filesystem::path p(std::string(result, count > 0 ? count : 0));
	p = p.parent_path();
#endif
	char buffer[1024];
#ifdef _WIN32
	sprintf_s(buffer, "START %s/%s", p.string().c_str(), command_line);
#else
	sprintf_s(buffer, "gnome-terminal -- %s/%s", p.string().c_str(), command_line);
#endif
	system(buffer);
}
void draw_ui_sidebar(float monitor_height, float sidebar_width)
{
	s64 plot_start_history = plot_get_visual_selection_index(main_plot);

	ImGui::SetNextWindowPos(ImVec2(0, monitor_height));
	ImGui::SetNextWindowSize(ImVec2(sidebar_width, (float)window_size_y-monitor_height));
	ImGui::Begin("UI", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	//static float width = 0.35f;
	//ImGui::DragFloat("width", &width, 0.01f, 0, 0, "%.3f");
	ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
	
#define PLOT_HISTORY(name, var) \
	{ \
		static bool show = false; \
		plot_history(main_plot, name, [&](s64 i){const MonitorDataEx& md = history[i]; return var;}, &show); \
	}
#define PLOT_HISTORY_AXIS(name, var) \
	{ \
		static bool show[monitor_axes]; \
		plot_history(main_plot, (std::string(axis_names[a]) + " " + name).c_str(), [&](s64 i){const MonitorDataEx& md = history[i]; return var;}, &show[a]); \
	}
#define PLOT_HISTORY_U(name, var, unit) \
	{ \
		static bool show = false; \
		plot_history(main_plot, name, [&](s64 i){const MonitorDataEx& md = history[i]; return var;}, &show, unit); \
	}
#define PLOT_HISTORY_AXIS_U(name, var, unit) \
	{ \
		static bool show[monitor_axes]; \
		plot_history(main_plot, (std::string(axis_names[a]) + " "  + name).c_str(), [&](s64 i){const MonitorDataEx& md = history[i]; return var;}, &show[a], unit); \
	}

	if (ImGui::CollapsingHeader("Connect", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::TextWrapped("This application cannot directly connect to ODrive. Instead you must start the proxy application on the device where ODrive is connected and then connect here to the proxy via TCP/IP.");

		ImGui::BeginDisabled(client_get_connection_state() == 0);
		if (ImGui::Button(client_get_connection_state() != 1 ? "Disconnect" : "Cancel connection"))
			client_disconnect();
		ImGui::EndDisabled();
		ImGui::NewLine();

		ImGui::TextDisabled("Connect remotely:");
		static char address[128] = "";
        ImGui::InputText("address", address, IM_ARRAYSIZE(address));
		static int port = ::port;
        ImGui::InputInt("port", &port);
		ImGui::BeginDisabled(client_get_connection_state() == 2 || address[0] == 0);
		if (ImGui::Button("Connect remotely"))
		{
			client_connect(address, port);
		}
		ImGui::EndDisabled();
		ImGui::NewLine();
		
		ImGui::TextDisabled("Connect locally:");
		ImGui::TextDisabled("Step 1: Start proxy");
		if (ImGui::Button("Start proxy to ODrive via USB"))
		{
			start_exe("proxy --usb -w");
		}
		ImGui::NewLine();

		static char uart_address[128] = "";
		static int baudrate = 115200;
        ImGui::InputInt("baudrate", &baudrate);
        ImGui::InputText("UART address", uart_address, IM_ARRAYSIZE(uart_address));
		ImGui::BeginDisabled(uart_address[0] == 0);
		if (ImGui::Button("Start proxy to ODrive via UART"))
		{
			char buffer[256];
			sprintf_s(buffer, "proxy --uart %s -b %d -w", uart_address, baudrate);
			start_exe(buffer);
		}
		ImGui::EndDisabled();
		ImGui::NewLine();

		ImGui::TextDisabled("Step 2: Connect to proxy");
		if (ImGui::Button("Connect to proxy"))
		{
			client_connect("127.0.0.1", ::port);
		}

	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("ODrive Config"))
	{
		if (ImGui::Button("save configuration"))
		{
			cd.odrive_save_configuration_trigger++;
			cd.counter++;
		}
		if (!history.back().odrive_fw_is_milana && ImGui::IsItemHovered()) ImGui::SetTooltip("The proxy needs to be restarted after this");
		if (ImGui::Button("reboot"))
		{
			cd.odrive_reboot_trigger++;
			cd.counter++;
		}
		if (ImGui::IsItemHovered()) ImGui::SetTooltip("The proxy needs to be restarted after this");
		ImGui::NewLine();

		if (ImGui::Checkbox("stop_motors_on_disconnect", &cd.stop_motors_on_disconnect)) { cd.counter++; }
		ImGui::NewLine();

		if (ImGui::DragFloat("max_regen_current", &cd.max_regen_current, 0.01f)) { cd.counter++; cd.odrive_set_control_counter++; }
		if (ImGui::DragFloat("brake_resistance", &cd.brake_resistance, 0.01f)) { cd.counter++; cd.odrive_set_control_counter++; }
		if (ImGui::DragFloat("dc_max_positive_current", &cd.dc_max_positive_current, 0.01f)) { cd.counter++; cd.odrive_set_control_counter++; }
		if (ImGui::DragFloat("dc_max_negative_current", &cd.dc_max_negative_current, 0.01f)) { cd.counter++; cd.odrive_set_control_counter++; }
		if (history.back().odrive_fw_is_milana)
			if (ImGui::DragInt("uart_baudrate", &cd.uart_baudrate)) { cd.counter++; cd.odrive_set_control_counter++; }
		if (ImGui::DragFloat("ibus_report_filter_k", &cd.ibus_report_filter_k, 0.01f)) { cd.counter++; cd.odrive_set_control_counter++; }
		if (history.back().odrive_fw_is_milana)
			if (ImGui::Checkbox("generate_error_on_filtered_ibus", &cd.generate_error_on_filtered_ibus)) { cd.counter++; cd.odrive_set_control_counter++; }
		ImGui::NewLine();
		
		PLOT_HISTORY("bus_voltage", md.odrive_bus_voltage);
		PLOT_HISTORY_U("bus_current", md.odrive_bus_current, &unit_current);
		ImGui::NewLine();
		
		ImGui::TextDisabled("serial_number: %llX ", history[plot_start_history].odrive_serial_number);
		ImGui::TextDisabled("hw_version: %d.%d-%dV",
				history[plot_start_history].odrive_hw_version_major,
				history[plot_start_history].odrive_hw_version_minor,
				history[plot_start_history].odrive_hw_version_variant);
		ImGui::TextDisabled("fw_version: %d.%d.%d%s",
				history[plot_start_history].odrive_fw_version / 1000000,
				history[plot_start_history].odrive_fw_version / 1000 % 1000,
				history[plot_start_history].odrive_fw_version % 1000,
				history[plot_start_history].odrive_fw_is_milana ? " (Milana)" : "");
	}
	ImGui::NewLine();

	for (int a = 0; a < monitor_axes; a++)
	{
		if (ImGui::CollapsingHeader(axis_names[a]))
		{
			ImGui::PushID(a);
			if (ImGui::Checkbox("enable axis", &cd.axes[a].enable_axis)) cd.counter++;

			{
				bool disabled = !history.back().axes[a].motor_is_calibrated || !history.back().axes[a].encoder_ready;
				if (cd.axes[a].enable_motor) disabled = false;
				ImGui::BeginDisabled(disabled);
				if (ImGui::Checkbox("enable motor", &cd.axes[a].enable_motor)) cd.counter++;
				ImGui::EndDisabled();
				if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !history.back().axes[a].encoder_ready)       ImGui::SetTooltip("encoder is not ready");
				if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !history.back().axes[a].motor_is_calibrated) ImGui::SetTooltip("motor is not calibrated");
			}

			ImGui::BeginDisabled(cd.axes[a].control_mode != CONTROL_MODE_TORQUE_CONTROL);
			if (ImGui::DragFloat("input_torque", &cd.axes[a].input_torque, 0.01f)) cd.counter++;
			ImGui::EndDisabled();

			ImGui::BeginDisabled(cd.axes[a].control_mode != CONTROL_MODE_VELOCITY_CONTROL);
			if (ImGui::DragFloat("input_vel", &cd.axes[a].input_vel, 0.01f)) cd.counter++;
			ImGui::EndDisabled();

			ImGui::BeginDisabled(cd.axes[a].control_mode != CONTROL_MODE_POSITION_CONTROL);
			if (ImGui::DragFloat("input_pos", &cd.axes[a].input_pos, 0.01f)) cd.counter++;
			ImGui::EndDisabled();

			ImGui::NewLine();
			
			if (ImGui::Button("trigger full calibration sequence"))
			{
				cd.axes[a].calibration_trigger++;
				cd.counter++;
			}
			ImGui::NewLine();

			ImGui::TextDisabled("Motor");
			if (ImGui::Checkbox("motor_pre_calibrated"    , &cd.axes[a].motor_pre_calibrated)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragInt("pole_pairs"               , &cd.axes[a].pole_pairs)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("torque_constant"        , &cd.axes[a].torque_constant, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("current_lim"            , &cd.axes[a].current_lim, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("current_lim_margin"     , &cd.axes[a].current_lim_margin, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("requested_current_range", &cd.axes[a].requested_current_range, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			ImGui::TextDisabled("motor_is_calibrated: %d", (int)history.back().axes[a].motor_is_calibrated);
			ImGui::NewLine();

			ImGui::TextDisabled("Controller");
			{
				ImGui::BeginDisabled(cd.axes[a].enable_motor);
				const char* items[] =
				{
					"VOLTAGE_CONTROL", 
					"TORQUE_CONTROL",  
					"VELOCITY_CONTROL",
					"POSITION_CONTROL",
				};
				if (ImGui::Combo("control mode", &cd.axes[a].control_mode, items, IM_ARRAYSIZE(items))) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
				ImGui::EndDisabled();
			}

			{
				const char* items[] =
				{
					"INACTIVE",     
					"PASSTHROUGH",
					"VEL_RAMP",
					"POS_FILTER",
					"MIX_CHANNELS",
					"TRAP_TRAJ",
					"TORQUE_RAMP",  
					"MIRROR",
					"TUNING",
				};
				if (ImGui::Combo("input mode", &cd.axes[a].input_mode, items, IM_ARRAYSIZE(items))) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			}
			
			if (ImGui::DragFloat("pos_gain"              , &cd.axes[a].pos_gain, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("vel_gain"              , &cd.axes[a].vel_gain, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("vel_integrator_gain"   , &cd.axes[a].vel_integrator_gain, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("vel_limit"             , &cd.axes[a].vel_limit, 0.1f, 0.1f, 100)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("vel_limit_tolerance"   , &cd.axes[a].vel_limit_tolerance, 0.1f, 0.1f, 100)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("input_filter_bandwidth", &cd.axes[a].input_filter_bandwidth, 0.1f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			{
				bool disabled = !history.back().axes[a].anticogging_valid;
				ImGui::BeginDisabled(disabled && !cd.axes[a].enable_anticogging);
				if (ImGui::Checkbox("enable_anticogging"     , &cd.axes[a].enable_anticogging)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
				ImGui::EndDisabled();
				if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && disabled) ImGui::SetTooltip("Anticogging must be calibrated first");
			}
			if (ImGui::Checkbox("enable_vel_limit"       , &cd.axes[a].enable_vel_limit)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::Checkbox("enable_overspeed_error" , &cd.axes[a].enable_overspeed_error)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			ImGui::NewLine();

			ImGui::TextDisabled("Encoder");
			{
				const char* items_text[] =
				{
					"INCREMENTAL",
					"HALL",        
					"SINCOS",      
					"SPI_ABS_CUI", 
					"SPI_ABS_AMS",
					"SPI_ABS_AEAT",
					"SPI_ABS_RLS",
					"SPI_ABS_MA732",
				};
				const int items_value[] = 
				{
					ENCODER_MODE_INCREMENTAL,
					ENCODER_MODE_HALL,        
					ENCODER_MODE_SINCOS,      
					ENCODER_MODE_SPI_ABS_CUI, 
					ENCODER_MODE_SPI_ABS_AMS, 
					ENCODER_MODE_SPI_ABS_AEAT,
					ENCODER_MODE_SPI_ABS_RLS,
					ENCODER_MODE_SPI_ABS_MA732,
				};
				static_assert(IM_ARRAYSIZE(items_text) == IM_ARRAYSIZE(items_value), "");
				const char* preview = "invalid";
				for (int i = 0; i < IM_ARRAYSIZE(items_text); i++)
				{
					if (cd.axes[a].encoder_mode == items_value[i])
					{
						preview = items_text[i];
					}
				}
				if (ImGui::BeginCombo("Encoder Mode", preview, 0))
				{
					for (int i = 0; i < IM_ARRAYSIZE(items_text); i++)
					{
						if (ImGui::Selectable(items_text[i], cd.axes[a].encoder_mode == items_value[i]))
						{
							cd.axes[a].encoder_mode = items_value[i];
							cd.axes[a].odrive_set_control_counter++;
							cd.counter++;
						}
					}
					ImGui::EndCombo();
				}
			}
			if (ImGui::Checkbox( "use_index"                , &cd.axes[a].encoder_use_index)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			{
				bool disabled = !cd.axes[a].encoder_pre_calibrated && !history.back().axes[a].encoder_ready;
				ImGui::BeginDisabled(disabled);
				if (ImGui::Checkbox("encoder_pre_calibrated", &cd.axes[a].encoder_pre_calibrated)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
				ImGui::EndDisabled();
				if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && disabled) ImGui::SetTooltip("encoder must be ready before pre_calibrated can be set to true");
			}
			if (ImGui::DragInt(  "cpr"                      , &cd.axes[a].encoder_cpr)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragFloat("bandwidth"                , &cd.axes[a].encoder_bandwidth, 1.0f)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (ImGui::DragInt(  "abs_spi_cs_gpio_pin"      , &cd.axes[a].encoder_abs_spi_cs_gpio_pin)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			if (history.back().odrive_fw_is_milana)
				if (ImGui::Checkbox( "ignore_abs_ams_error_flag", &cd.axes[a].encoder_ignore_abs_ams_error_flag)) { cd.counter++; cd.axes[a].odrive_set_control_counter++; }
			ImGui::NewLine();

			ImGui::TextDisabled("encoder_ready: %d", (int)history.back().axes[a].encoder_ready);
			if (ImGui::Button("trigger encoder z search"))
			{
				cd.axes[a].encoder_z_search_trigger++;
				cd.counter++;
			}
			ImGui::NewLine();

			ImGui::TextDisabled("Plotting Variables");
			PLOT_HISTORY_AXIS_U("pos", md.axes[a].pos, &unit_pos);
			PLOT_HISTORY_AXIS_U("input_pos", md.axes[a].input_pos, &unit_pos);
			PLOT_HISTORY_AXIS_U("vel", md.axes[a].vel, &unit_vel);
			PLOT_HISTORY_AXIS_U("vel_coarse", md.axes[a].vel_coarse, &unit_vel);
			PLOT_HISTORY_AXIS_U("input_vel", md.axes[a].input_vel, &unit_vel);
			PLOT_HISTORY_AXIS_U("integrator", md.axes[a].integrator, &unit_vel);
			PLOT_HISTORY_AXIS_U("input_torque", md.axes[a].input_torque, &unit_current);
			PLOT_HISTORY_AXIS_U("current_target", md.axes[a].current_target, &unit_current);
			ImGui::NewLine();

			PLOT_HISTORY_AXIS_U("encoder_shadow_count", float(md.axes[a].encoder_shadow_count)/cd.axes[a].encoder_cpr, &unit_pos);
			PLOT_HISTORY_AXIS("encoder_index_error", md.axes[a].encoder_index_error*0.0001f);
			PLOT_HISTORY_AXIS("encoder_index_count", md.axes[a].encoder_index_count*0.01f);
			
			ImGui::PopID();
		}
		ImGui::NewLine();
	}

	if (ImGui::CollapsingHeader("Oscilloscope"))
	{
		MonitorData& md = get_last_monitor_data();
		ImGui::TextWrapped("Right now, the oscilloscope is hardcoded to record position, velocity and current of both axes upon triggering. (It would be easy to make this configurable, but it's not done yet).");
		ImGui::TextWrapped("After triggering, the oscilloscope will record those values in 8000Hz until the RAM of ODrive is full. Then it will transmit that data (which will take a while) and the data will appear in the plots.");

		bool disabled = true;
		const char* hover_text = 0;
		if (client_get_connection_state() != 2)
		{
			hover_text = "The proxy must be connected";
		}
		else if (!history.back().odrive_fw_is_milana)
		{
			hover_text = "Oscilloscope only works with ODrive Milana firmware";
		}
		else if (!md.axes[0].is_running || !md.axes[1].is_running)
		{
			hover_text = "Both motors must be running to trigger the oscilloscope";
		}
		else if (md.oscilloscope_state != 0)
		{
		}
		else
			disabled = false;

		ImGui::BeginDisabled(disabled);
		if (ImGui::Button("trigger oscilloscope"))
		{
			cd.oscilloscope_force_trigger++;
			cd.counter++;
		}
		ImGui::EndDisabled();
		if (hover_text && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", hover_text);

		switch (md.oscilloscope_state)
		{
		case 0: ImGui::TextDisabled("oscilloscope_state: idle"); break;
		case 1: ImGui::TextDisabled("oscilloscope_state: recording"); break;
		case 2: ImGui::TextDisabled("oscilloscope_state: recording done"); break;
		case 3: ImGui::TextDisabled("oscilloscope_state: transmitting"); break;
		default: assert(0);
		}
		PLOT_HISTORY("oscilloscope_state", (float)md.oscilloscope_state);
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Timing"))
	{
		if (ImGui::SliderInt("target_delta_time_ms", &cd.target_delta_time_ms, 1, 100)) cd.counter++;

		PLOT_HISTORY("delta_time", md.delta_time * 1000.f);
		PLOT_HISTORY("delta_time_odrive", md.delta_time_odrive * 0.001f);
		PLOT_HISTORY("delta_time_sleep", md.delta_time_sleep * 0.001f);
		PLOT_HISTORY("delta_time_network", md.delta_time_network * 0.001f);
		PLOT_HISTORY("frame counter", (float)md.counter);
		PLOT_HISTORY("odrive counter", (float)md.odrive_counter);

		ImGui::TextDisabled("Uptime: %llus", history[plot_start_history].uptime_micros/1000000);
		ImGui::TextDisabled("Time: %s", ctime(&history[plot_start_history].local_time));
	}
	ImGui::NewLine();
	if (ImGui::Button("clear history"))
		clear_history();
	ImGui::NewLine();

	{
		ImGui::TextDisabled("Save/Load all plotting data");
		ImGui::TextDisabled("in the folder ../logs/");
		static char filename[128] = "monitor_log.bin";
		ImGui::InputText("filename", filename, IM_ARRAYSIZE(filename));
		
		ImGui::BeginDisabled(history.size() <= 1);
		if (ImGui::Button("save history"))
			save_history(filename);
		ImGui::EndDisabled();
		ImGui::BeginDisabled(client_get_connection_state() != 0);
		if (ImGui::Button("load history"))
			load_history(filename);
		ImGui::EndDisabled();
	}
	
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::PopItemWidth();
	ImGui::End();
}

void draw_ui_main_1(float monitor_height, float sidebar_width)
{
	ImGui::SetNextWindowPos(ImVec2(sidebar_width, 0));
	ImGui::SetNextWindowSize(ImVec2((float)window_size_x-sidebar_width+5, (float)window_size_y));
	ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

	if (ImGui::TreeNodeEx("State visualization", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImVec2 draw_robot_ui_pos = ImGui::GetCursorScreenPos();
		draw_joint_state(draw_robot_ui_pos.x+80*dpi_scaling, draw_robot_ui_pos.y+80*dpi_scaling);
		ImGui::Dummy(ImVec2(0, 200*dpi_scaling));
		ImGui::TreePop();
	}
	ImGui::End();
}

void draw_ui_main_2()
{
	s64 plot_start_history = plot_get_visual_selection_index(main_plot);

	ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

	if (ImGui::TreeNodeEx("Plot", ImGuiTreeNodeFlags_DefaultOpen))
	{
		plot_draw(main_plot, 400*dpi_scaling);
		ImGui::TreePop();
	}
		
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();

	ImGui::End();
}

void draw_ui()
{
	set_plot_history_elements(main_plot, history.size(),
		[](s64 i){ return history[i].display_time; },
		1.0 / odrive_frequency);

	float monitor_height = 70*dpi_scaling;
	float sidebar_width = 350*dpi_scaling;
	
	draw_ui_monitoring(monitor_height, sidebar_width);

	draw_ui_sidebar(monitor_height, sidebar_width);

	draw_ui_main_1(monitor_height, sidebar_width);
	draw_ui_main_2();

	//ImGui::ShowDemoWindow();
	//ImPlot::ShowDemoWindow();
}


void Display()
{
	glfwGetWindowSize(window, &window_size_x, &window_size_y);
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    glClearColor(0.95f, 0.98f, 0.99f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, window_size_x, window_size_y);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, window_size_x, window_size_y, 0, -1.0f, +1.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	draw_ui();

	ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
}

void OnKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	// space isn't used by imgui, so we handle that for emergency exit
	if (ImGui::GetIO().WantTextInput && key != GLFW_KEY_SPACE)
	{
		ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
		return;
	}

	if (key >= 0 && key < GLFW_KEY_LAST)
		pressing_key[key] = (action != GLFW_RELEASE);

	ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);

	if (action == GLFW_RELEASE)
		return;

	if (key >= '0' && key <= '9')
	{
		static s64 save[10];
		if (mods & GLFW_MOD_CONTROL)
			save[key-'0'] = plot_get_visual_selection_index(main_plot);
		else if (save[key-'0'] != 0)
			plot_set_visual_selection_index(main_plot, save[key-'0']);
	}

	switch (key)
    {
	case GLFW_KEY_SPACE:
		do_emergency_stop();
		break;
	
    case GLFW_KEY_ESCAPE:
	case GLFW_KEY_F11:
	{
		// Switch between fullscreen and windowed mode.
		static int xPosWindowed, yPosWindowed;
		static int xSizeWindowed, ySizeWindowed;
		if (glfwGetWindowMonitor(window))
		{
			glfwSetWindowMonitor(window, nullptr,
					xPosWindowed, yPosWindowed,
					xSizeWindowed, ySizeWindowed, GLFW_DONT_CARE);
		}
		else
		{
			if (key == GLFW_KEY_ESCAPE)
			{
				glfwSetWindowShouldClose(window, 1);
				break;
			}
			glfwGetWindowPos(window, &xPosWindowed, &yPosWindowed);
			glfwGetWindowSize(window, &xSizeWindowed, &ySizeWindowed);

			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
			const GLFWvidmode* v = glfwGetVideoMode(monitor);
			int xpos, ypos;
			glfwGetMonitorPos(monitor, &xpos, &ypos);
			glfwSetWindowMonitor(window, monitor, xpos, ypos, v->width, v->height, GLFW_DONT_CARE);
		}
		break;
	}
	}
}

void on_mouse_button(GLFWwindow* window, int button, int action, int mods)
{
	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
}

void on_mouse_wheel(GLFWwindow* window, double xOffset, double yOffset)
{
	ImGui_ImplGlfw_ScrollCallback(window, xOffset, yOffset);
}

void on_window_sized(GLFWwindow* window, int w, int h)
{
	Display();
}

void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
	assert(0);
}


#ifdef _MSC_VER
// Linker -> System -> SubSystem
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char* CmdLine, int CmdShow)
#else
int main(int argc, char** argv)
#endif
{
	time_init();
	{
		MonitorData md;
		md.counter = 0;
		on_new_monitor_data(md);
	}
	if (!client_init()) return EXIT_FAILURE;

	//convert_logs();

	if (0)
	{
		load_history("monitor_log.bin");
		//plot_set_x_limit_min(main_plot, 599);
		//plot_set_x_limit_width(main_plot, 2);
	}

	// Setup window
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        return EXIT_FAILURE;

	glfwWindowHint(GLFW_VISIBLE, 0);
	glfwWindowHint(GLFW_MAXIMIZED, 1);
    window = glfwCreateWindow(1280, 720, "BalanceBot Controller", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Disable vsync

    // register callbacks
    glfwSetMouseButtonCallback(window, on_mouse_button);
    glfwSetScrollCallback     (window, on_mouse_wheel);
    glfwSetKeyCallback        (window, OnKeyboard);
    glfwSetCharCallback       (window, ImGui_ImplGlfw_CharCallback);
	glfwSetWindowSizeCallback (window, on_window_sized);

	// get windows dpi scaling factor
#ifdef _MSC_VER
	HDC hdc = GetDC(0);
	dpi_scaling = (float)GetDeviceCaps(hdc, LOGPIXELSX) / 96.f;
	ReleaseDC(0, hdc);
#else
	dpi_scaling = 1;
#endif

	// setup ImGui
    ImGui::CreateContext();
    ImPlot::CreateContext();
	ImGui::StyleColorsLight();
	ImGui::GetStyle().ScaleAllSizes(dpi_scaling);
	ImGui::GetStyle().WindowRounding = 0;
    ImGuiIO& io = ImGui::GetIO();
	io.IniFilename = nullptr;

#if INCLUDE_ICON_FONTS
	io.Fonts->AddFontFromMemoryCompressedBase85TTF(DroidSans_compressed_data_base85, 18.0f*dpi_scaling);
	ImFontConfig config;
	config.MergeMode = true;
	//config.GlyphMinAdvanceX = 18.0f*dpi_scaling; // Use if you want to make the icon monospaced
	static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
	io.Fonts->AddFontFromMemoryCompressedBase85TTF(fa_solid_900_compressed_data_base85, 18.0f*dpi_scaling, &config, icon_ranges);
#else
	io.Fonts->AddFontDefault();
#endif

	ImGui_ImplGlfw_InitForOpenGL(window, false);
	ImGui_ImplOpenGL2_Init();

	glfwShowWindow(window);
	
	// main loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
		Display();

		if (!client_update())
			break;
		
		double current_time = glfwGetTime();

		if (!glfwGetWindowAttrib(window, GLFW_FOCUSED) || glfwGetWindowAttrib(window, GLFW_ICONIFIED))
			imprecise_sleep(.1);
		else
			imprecise_sleep(0.01);
    }

    // cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwTerminate();
	client_close();

	return EXIT_SUCCESS;
}

