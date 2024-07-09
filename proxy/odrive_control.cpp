// Here we communicate with ODrive with the ODrive class.
// We basically fill out MonitorData with the values we receive from ODrive
// and set ODrive parameters according to the values in ControlData

#include "odrive_control.h"
#include "../common/odrive/ODrive.h"
#include "../common/odrive/odrive_helper.h"
#include "main.h"

#include <string>
#include <algorithm>

void odrive_control_get_control_data();
void odrive_control_set_control_data();
void odrive_control_axis_get_control_data(int a);
void odrive_control_update_axis(int a);

ODrive odrive;
static int cd_counter;
static int cd_counter_axis[monitor_axes];

Endpoint& get_axis(int axis)
{
	switch (axis)
	{
	case 0: return odrive.root("axis0");
	case 1: return odrive.root("axis1");
	default: assert(0); break;
	}
	return *(Endpoint*)nullptr;
}

inline u32 as_uint(const float x)
{
    union { float f; u32 i; } val;
    val.f = x;
    return val.i;
}
inline float as_float(const u32 x)
{
    union { float f; u32 i; } val;
    val.i = x;
    return val.f;
}

float half_to_float(u16 x)
{
	// IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
    const u32 e = (x&0x7C00)>>10; // exponent
    const u32 m = (x&0x03FF)<<13; // mantissa
    const u32 v = as_uint((float)m)>>23; // evil log2 bit hack to count leading zeros in denormalized format
    return as_float((x&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23|((m<<(150-v))&0x007FE000))); // sign : normalized : denormalized
}

static bool check_errors_and_watchdog_feed()
{
	if (odrive.communication_error)
	{
		printf("ODrive communication error\n");
		return false;
	}
	bool any_errors = true;
	if (odrive.root.odrive_fw_is_milana())
	{
		// This is a function that combines these calls to make things a bit faster
		odrive.root("any_errors_and_watchdog_feed").get(any_errors);
	}
	else
	{
		for (int a = 0; a < monitor_axes; a++)
			get_axis(a)("watchdog_feed").call();
		//odrive.root("any_error").call(&any_errors);
	}
	if (!any_errors)
		return true;

	if (odrive.root.odrive_fw_is_milana())
		printf("\nodrive error!\n");
	int num_errors = 0;
	check_odrive_errors(&odrive.root, num_errors);
	for (int a = 0; a < monitor_axes; a++)
		check_axis_errors(&get_axis(a), axis_names[a], num_errors);
		
	return num_errors == 0;
}

bool odrive_control_init(const Params& params)
{
	if (params.connect_uart)
	{
		if (!odrive.connect_uart(params.uart_address.c_str(), params.uart_baud_rate, params.uart_stop_bits == 2)) return false;
	}
	else if (params.connect_usb)
	{
		if (!odrive.connect_usb()) return false;
	}
	else
		return false;

	// Temporarilly disable watchdog, so it won't immediately make errors
	for (int a = 0; a < monitor_axes; a++)
	{
		get_axis(a)("config")("enable_watchdog").set(false);
		md.axes[a].is_running = false;
		md.axes[a].encoder_ready = false;
		md.axes[a].motor_is_calibrated = false;
	}

	if (params.clear_errors_on_startup)
	{
		clear_odrive_errors(&odrive.root);
		for (int a = 0; a < monitor_axes; a++)
		{
			clear_axis_errors(&get_axis(a));
		}
	}
	else
	{
		// Clear potential watchdog errors from previous runs, in case it wasn't
		// properly shutdown.
		for (int a = 0; a < monitor_axes; a++)
		{
			if (get_axis(a)("error").get2<int>() == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
			{
				get_axis(a)("error").set(0);
				printf("clear watchdog error %s\n", axis_names[a]);
			}
		}
	}

	// Get data that will never change
	odrive.root("serial_number"      ).get(md.odrive_serial_number);
	odrive.root("hw_version_major"   ).get(md.odrive_hw_version_major);
	odrive.root("hw_version_minor"   ).get(md.odrive_hw_version_minor);
	odrive.root("hw_version_variant" ).get(md.odrive_hw_version_variant);
	md.odrive_fw_version = odrive.root.get_odrive_fw_version();
	md.odrive_fw_is_milana = odrive.root.odrive_fw_is_milana();

	odrive_control_get_control_data();
	cd_counter = cd.odrive_set_control_counter;

	for (int a = 0; a < monitor_axes; a++)
	{
		odrive_control_axis_get_control_data(a);

		// Retrieve initial sensor values so we fail early if odrive_control_update_axis fails for some reason.
		cd_counter_axis[a] = cd.axes[a].odrive_set_control_counter;
		odrive_control_update_axis(a);
	}

	// Enable watchdog
	for (int a = 0; a < monitor_axes; a++)
	{
		Endpoint& axis = get_axis(a);
		axis("config")("watchdog_timeout").set(1.0f);
		axis("watchdog_feed").call();
		axis("config")("enable_watchdog").set(true);
	}

	if (!check_errors_and_watchdog_feed())
		return false;
	return true;
}

void odrive_control_close()
{
	if (odrive.is_connected)
	{
		for (int a = 0; a < monitor_axes; a++)
		{
			get_axis(a)("requested_state").set(AXIS_STATE_IDLE);
			get_axis(a)("config")("enable_watchdog").set(false);
			md.axes[a].is_running = false;
		}
	}
	odrive.close();
}

void odrive_control_get_control_data()
{
	odrive.root("config")("max_regen_current").get(cd.max_regen_current);
	odrive.root("config")("brake_resistance").get(cd.brake_resistance);
	odrive.root("config")("dc_max_positive_current").get(cd.dc_max_positive_current);
	odrive.root("config")("dc_max_negative_current").get(cd.dc_max_negative_current);
	if (odrive.root.odrive_fw_is_milana())
		odrive.root("config")("uart_baudrate").get(cd.uart_baudrate);
	odrive.root("ibus_report_filter_k").get(cd.ibus_report_filter_k);
	if (odrive.root.odrive_fw_is_milana())
		odrive.root("generate_error_on_filtered_ibus").get(cd.generate_error_on_filtered_ibus);
}
void odrive_control_set_control_data()
{
	odrive.root("config")("max_regen_current").set(cd.max_regen_current);
	odrive.root("config")("brake_resistance").set(cd.brake_resistance);
	odrive.root("config")("dc_max_positive_current").set(cd.dc_max_positive_current);
	odrive.root("config")("dc_max_negative_current").set(cd.dc_max_negative_current);
	if (odrive.root.odrive_fw_is_milana())
		odrive.root("config")("uart_baudrate").set(cd.uart_baudrate);
	odrive.root("ibus_report_filter_k").set(cd.ibus_report_filter_k);
	if (odrive.root.odrive_fw_is_milana())
		odrive.root("generate_error_on_filtered_ibus").set(cd.generate_error_on_filtered_ibus);
}

void odrive_control_axis_get_control_data(int a)
{
	Endpoint& axis = get_axis(a);
	ControlDataAxis& acd = cd.axes[a];

	// motor
	Endpoint& motor_config = axis("motor")("config");
	motor_config("pre_calibrated").get(acd.motor_pre_calibrated);
	motor_config("pole_pairs").get(acd.pole_pairs);
	motor_config("torque_constant").get(acd.torque_constant);
	motor_config("current_lim").get(acd.current_lim);
	motor_config("current_lim_margin").get(acd.current_lim_margin);
	motor_config("requested_current_range").get(acd.requested_current_range);
	
	// controller
	Endpoint& controller_config = axis("controller")("config");
	controller_config("control_mode").get(acd.control_mode);
	controller_config("input_mode").get(acd.input_mode);
	controller_config("pos_gain"              ).get(acd.pos_gain);
	controller_config("vel_gain"              ).get(acd.vel_gain);
	controller_config("vel_integrator_gain"   ).get(acd.vel_integrator_gain);
	controller_config("vel_limit"             ).get(acd.vel_limit);
	controller_config("vel_limit_tolerance"   ).get(acd.vel_limit_tolerance);
	controller_config("input_filter_bandwidth").get(acd.input_filter_bandwidth);
	controller_config("anticogging")("anticogging_enabled").get(acd.enable_anticogging);
	controller_config("enable_vel_limit"      ).get(acd.enable_vel_limit);
	controller_config("enable_overspeed_error").get(acd.enable_overspeed_error);

	// encoder
	Endpoint& encoder_config = axis("encoder")("config");
	encoder_config("mode").get(acd.encoder_mode);
	encoder_config("use_index").get(acd.encoder_use_index);
	encoder_config("pre_calibrated").get(acd.encoder_pre_calibrated);
	encoder_config("cpr").get(acd.encoder_cpr);
	encoder_config("bandwidth").get(acd.encoder_bandwidth);
	encoder_config("abs_spi_cs_gpio_pin").get(acd.encoder_abs_spi_cs_gpio_pin);
	if (odrive.root.odrive_fw_is_milana())
		encoder_config("ignore_abs_ams_error_flag").get(acd.encoder_ignore_abs_ams_error_flag);
}

void odrive_control_axis_set_control_data(int a)
{
	Endpoint& axis = get_axis(a);
	ControlDataAxis& acd = cd.axes[a];

	// motor
	Endpoint& motor_config = axis("motor")("config");
	motor_config("pre_calibrated").set(acd.motor_pre_calibrated);
	motor_config("pole_pairs").set(acd.pole_pairs);
	motor_config("torque_constant").set(acd.torque_constant);
	motor_config("current_lim").set(acd.current_lim);
	motor_config("current_lim_margin").set(acd.current_lim_margin);
	motor_config("requested_current_range").set(acd.requested_current_range);
	
	// controller
	Endpoint& controller_config = axis("controller")("config");
	controller_config("control_mode").set(acd.control_mode);
	controller_config("input_mode").set(acd.input_mode);
	controller_config("pos_gain"              ).set(acd.pos_gain);
	controller_config("vel_gain"              ).set(acd.vel_gain);
	controller_config("vel_integrator_gain"   ).set(acd.vel_integrator_gain);
	controller_config("vel_limit"             ).set(acd.vel_limit);
	controller_config("vel_limit_tolerance"   ).set(acd.vel_limit_tolerance);
	controller_config("input_filter_bandwidth").set(acd.input_filter_bandwidth);
	controller_config("anticogging")("anticogging_enabled").set(acd.enable_anticogging);
	controller_config("enable_vel_limit"      ).set(acd.enable_vel_limit);
	controller_config("enable_overspeed_error").set(acd.enable_overspeed_error);

	// encoder
	Endpoint& encoder_config = axis("encoder")("config");
	encoder_config("mode").set(acd.encoder_mode);
	encoder_config("use_index").set(acd.encoder_use_index);
	encoder_config("pre_calibrated").set(acd.encoder_pre_calibrated);
	encoder_config("cpr").set(acd.encoder_cpr);
	encoder_config("bandwidth").set(acd.encoder_bandwidth);
	encoder_config("abs_spi_cs_gpio_pin").set(acd.encoder_abs_spi_cs_gpio_pin);
	if (odrive.root.odrive_fw_is_milana())
		encoder_config("ignore_abs_ams_error_flag").set(acd.encoder_ignore_abs_ams_error_flag);
}

void odrive_control_update_axis(int a)
{
	Endpoint& axis = get_axis(a);
	//axis("watchdog_feed").call(); // called by any_errors_and_watchdog_feed
	if (cd_counter_axis[a] != cd.axes[a].odrive_set_control_counter)
	{
		odrive_control_axis_set_control_data(a);
		cd_counter_axis[a] = cd.axes[a].odrive_set_control_counter;
	}

	bool should_run = cd.axes[a].enable_motor;
	if (should_run != md.axes[a].is_running)
	{
		axis("requested_state").set(should_run ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
		md.axes[a].is_running = should_run;
	}

	// Set target value based on control mode
	md.axes[a].input_torque = 0;
	md.axes[a].input_vel = 0;
	md.axes[a].input_pos = 0;
	switch (cd.axes[a].control_mode)
	{
	case CONTROL_MODE_TORQUE_CONTROL:
		axis("controller")("input_torque").set(cd.axes[a].input_torque);
		md.axes[a].input_torque = cd.axes[a].input_torque;
		break;
	case CONTROL_MODE_VELOCITY_CONTROL:
		axis("controller")("input_vel").set(cd.axes[a].input_vel);
		md.axes[a].input_vel = cd.axes[a].input_vel;
		break;
	case CONTROL_MODE_POSITION_CONTROL:
		axis("controller")("input_pos").set(cd.axes[a].input_pos);
		md.axes[a].input_pos = cd.axes[a].input_pos;
		break;
	}


	float old_pos = md.axes[a].pos;
	axis("encoder")("pos_estimate").get(md.axes[a].pos);
	md.axes[a].vel_coarse = (md.axes[a].pos-old_pos) / md.delta_time;

	axis("encoder")("vel_estimate").get(md.axes[a].vel);

	axis("motor")("current_control")("Iq_setpoint").get(md.axes[a].current_target);
}

static void odrive_control_handle_z_search(int a)
{
	Endpoint& axis = get_axis(a);
	if (!md.axes[a].encoder_ready)
	{
		if (axis("encoder")("is_ready").get2<bool>() &&
			(!cd.axes[a].encoder_use_index || axis("encoder")("index_found").get2<bool>()))
			md.axes[a].encoder_ready = true;
	}
	if (!md.axes[a].motor_is_calibrated)
	{
		if (axis("motor")("is_calibrated").get2<bool>())
			md.axes[a].motor_is_calibrated = true;
	}

	bool do_trigger_z_search = false;

	static int last_encoder_z_search_trigger[monitor_axes];
	if (cd.axes[a].encoder_z_search_trigger-last_encoder_z_search_trigger[a] == 1 && !md.axes[a].is_running)
	{
		do_trigger_z_search = true;
		md.axes[a].encoder_ready = false;
	}
	last_encoder_z_search_trigger[a] = cd.axes[a].encoder_z_search_trigger;

	if (do_trigger_z_search)
	{
		axis("requested_state").set(AXIS_STATE_ENCODER_INDEX_SEARCH);
	}
}

static void odrive_control_handle_calibration(int a)
{
	Endpoint& axis = get_axis(a);

	static int last_calibration_trigger[monitor_axes];
	if (cd.axes[a].calibration_trigger-last_calibration_trigger[a] == 1 && !md.axes[a].is_running)
	{
		axis("requested_state").set(AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
		md.axes[a].encoder_ready = false;
		md.axes[a].motor_is_calibrated = false;
		md.axes[a].anticogging_valid = false;
	}
	last_calibration_trigger[a] = cd.axes[a].calibration_trigger;
}

static void odrive_control_handle_oscilloscope()
{
	bool trigger_oscilloscope = false;

	static int oscilloscope_size = 0;
		
	static int last_oscilloscope_force_trigger;
	if (cd.oscilloscope_force_trigger == last_oscilloscope_force_trigger+1)
		trigger_oscilloscope = true;
	last_oscilloscope_force_trigger = cd.oscilloscope_force_trigger;
	
	if (trigger_oscilloscope && md.oscilloscope_state == 0)
	{
		get_axis(0)("motor")("oscilloscope_force_trigger").set(true);
		s64 oscilloscope_counter = 0;
		while (oscilloscope_counter == 0 && running)
		{
			get_axis(0)("motor")("oscilloscope_counter").get(oscilloscope_counter);
		}

		if ((oscilloscope_counter & ((s64)1<<62)) == 0)
		{
			printf("oscilloscope doesn't seem to work!\n");
			get_axis(0)("motor")("oscilloscope_force_trigger").set(false);
		}
		else
		{
			md.odrive_counter = (int)oscilloscope_counter;
			md.oscilloscope_start = md.odrive_counter;
			md.oscilloscope_end = md.odrive_counter;
			md.oscilloscope_state = 1;
		}
	}
	else if (md.oscilloscope_state == 1)
	{
		// oscilloscope is recording on ODrive right now.
		// Check if it's done.
		s64 oscilloscope_counter = get_axis(0)("motor")("oscilloscope_counter").get2<s64>();
		if ((oscilloscope_counter & ((s64)1<<61)) != 0)
		{
			// oscilloscope recording is done
			md.odrive_counter = (int)oscilloscope_counter;
			md.oscilloscope_end = md.odrive_counter;
			md.oscilloscope_state = 2;
			get_axis(0)("motor")("oscilloscope_counter").set((s64)0);
		}
	}
	else if (md.oscilloscope_state == 2)
	{
		// oscilloscope recording just finished, get number of recorded samples
		odrive.root("oscilloscope_size").get(oscilloscope_size);
		md.oscilloscope_state = 3;
		md.oscilloscope_start = 0;
		md.oscilloscope_end = 0;
	}
	if (md.oscilloscope_state == 3)
	{
		// Retrieve recorded values from ODrive and send them to control_ui.
		// We don't retrieve all values at once, because that would be very slow
		// and MonitorData cannot handle a variable amount anyway.
		md.oscilloscope_start = md.oscilloscope_end;
		while (md.oscilloscope_end < oscilloscope_size &&
			    md.oscilloscope_end-md.oscilloscope_start < oscilloscope_transmitting_size)
		{
#if 0
			float value = -1;
			odrive.root("get_oscilloscope_val").call(md.oscilloscope_end, &value);
			md.oscilloscope_transmitting[md.oscilloscope_end-md.oscilloscope_start] = value;
			md.oscilloscope_end++;
#else
			// speed up oscilloscope retrieval by sending 4 values, packed into 64bit, at once.
			u64 value = 0;
			odrive.root("get_oscilloscope_val_4").call(md.oscilloscope_end, &value);
			int i = 0;
			while (md.oscilloscope_end < oscilloscope_size &&
					md.oscilloscope_end-md.oscilloscope_start < oscilloscope_transmitting_size &&
					i < 4)
			{
				md.oscilloscope_transmitting[md.oscilloscope_end-md.oscilloscope_start] = half_to_float((u16)(value>>(i*16)));
				md.oscilloscope_end++;
				i++;
			}
#endif
		}
		if (md.oscilloscope_start == md.oscilloscope_end)
			md.oscilloscope_state = 0;
	}
}

bool odrive_control_update()
{
	u32_micros start_time = time_micros();
	
	// Setting all the ODrive values is quite slow, so we do it only when control_ui actually changes something.
	// control_ui indicates this by changing cd.odrive_set_control_counter
	if (cd_counter != cd.odrive_set_control_counter)
	{
		odrive_control_set_control_data();
		cd_counter = cd.odrive_set_control_counter;
	}

	if (odrive.root.odrive_fw_is_milana())
	{
		// Get ODrive counter (Increased on ODrive with 8kHz)
		// Sadly this variable is missing since fw version 0.5.2
		// but it's ok, control_ui will calculate the time according to the target framerate.
		int new_counter = get_axis(0)("loop_counter").get2<int>();
		if (md.odrive_counter == new_counter)
			printf("odrive_counter did not change! %d\n", new_counter);
		if (md.odrive_counter > new_counter)
			printf("odrive_counter overflow! %d %d\n", md.odrive_counter, new_counter);
		md.odrive_counter = new_counter;
	}

	odrive_control_handle_oscilloscope();

	for (int a = 0; a < monitor_axes; a++)
	{
		Endpoint& axis = get_axis(a);
		if (cd.axes[a].enable_axis)
		{
			odrive_control_handle_z_search(a);
			odrive_control_handle_calibration(a);
			odrive_control_update_axis(a);

			// We want these variables for debugging purposes, but don't want to waste too much time
			// retrieving them. So we retrieve only one variable per timestep.
			switch (md.counter%6)
			{
			case 0: odrive.root("vbus_voltage").get(md.odrive_bus_voltage); break;
			case 1: odrive.root("ibus").get(md.odrive_bus_current); break;
			case 2: if (odrive.root.odrive_fw_is_milana()) axis("encoder")("index_check_cumulative_error").get(md.axes[a].encoder_index_error); break;
			case 3: if (odrive.root.odrive_fw_is_milana()) axis("encoder")("index_check_index_count").get(md.axes[a].encoder_index_count); break;
			case 4: axis("encoder")("shadow_count").get(md.axes[a].encoder_shadow_count); break;
			case 5: axis("controller")("anticogging_valid").get(md.axes[a].anticogging_valid); break;
			}
		}
		else
		{
			if (md.axes[a].is_running)
			{
				axis("requested_state").set(AXIS_STATE_IDLE);
				md.axes[a].is_running = false;
			}
			md.axes[a].pos = 0;
			md.axes[a].vel = 0;
			md.axes[a].vel_coarse = 0;
			md.axes[a].integrator = 0;
			md.axes[a].current_target = 0;
		}
	}
	if (!check_errors_and_watchdog_feed())
		return false;
	
	static int last_odrive_save_configuration = 0;
	if (cd.odrive_save_configuration_trigger-last_odrive_save_configuration == 1)
	{
		printf("save_configuration()\n");
		odrive.root("save_configuration").call();
	}
	last_odrive_save_configuration = cd.odrive_save_configuration_trigger;

	static int last_odrive_reboot = 0;
	if (cd.odrive_reboot_trigger-last_odrive_reboot == 1)
	{
		printf("reboot()\n");
		odrive.root("reboot").call();
	}
	last_odrive_reboot = cd.odrive_reboot_trigger;

	
	md.delta_time_odrive = time_micros() - start_time;

	return true;
}
