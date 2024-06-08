
#ifdef _MSC_VER
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#else
#include <unistd.h>
#include <sys/resource.h>
#include <sys/param.h>
#endif
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <stdexcept>
#include "main.h"
#include "server.h"
#include "odrive_control.h"
#include "../common/network.h"
#include "../common/time_helper.h"

bool running = true;
MonitorData md;
ControlData cd;

#ifdef _MSC_VER
static BOOL WINAPI ctrl_c_handler(DWORD signal)
{
	if (signal == CTRL_C_EVENT)
	{
		running = false;
	}
	return TRUE;
}
#else
static void ctrl_c_handler(int signum)
{
	running = false;
}
#endif

static void setup_handlers()
{
#ifdef _MSC_VER
    SetConsoleCtrlHandler(ctrl_c_handler, TRUE);
#else
	struct sigaction sa = { 0 };
	sa.sa_handler = ctrl_c_handler;

	// If we don't set SA_RESTART, the recv call to the uart file handle in ODrive will fail.
	sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
#endif
}



void print_usage(int /*argc*/, char** argv, const Params& params)
{
    printf("Connects to ODrive (via USB or UART) and monitors it. Also creates a server control_ui can connect to in order to visualize that data.\n");
    printf("\n");
    printf("usage: %s [options]\n", argv[0]);
    printf("\n");
    printf("options:\n");
    printf("  -h, --help            show this help message and exit\n");
    printf("  --usb                 connect with ODrive via USB\n");
    printf("  --uart ADDRESS        connect with ODrive via UART\n");
    printf("  -b N, --baudrate N    specify uart baudrate (default: %d)\n", params.uart_baud_rate);
    printf("  -s N, --stop-bits N   specify number of uart stop bits (1 or 2) (default: %d)\n", params.uart_stop_bits);
    printf("  -p N, --port N        port to listen to for control_ui connections (default: %d)\n", params.port);
    printf("  -w, --wait-input      wait for input after exit\n");
    printf("  -nc, --no-clear       do not clear ODrive errors on startup\n");
    printf("\n");
}

bool params_parse_ex(int argc, char** argv, Params& params)
{
    bool invalid_param = false;
    std::string arg;
    const std::string arg_prefix = "--";

    for (int i = 1; i < argc; i++)
    {
        arg = argv[i];
        if (arg.compare(0, arg_prefix.size(), arg_prefix) == 0)
        {
            std::replace(arg.begin(), arg.end(), '_', '-');
        }
        
        if (arg == "-h" || arg == "--help")
        {
            return false;
        }
        else if (arg == "--uart")
        {
            params.connect_uart = true;
            if (++i >= argc)
            {
                invalid_param = true;
                break;
            }
            params.uart_address = argv[i];
            if (params.uart_address.size() == 0)
            {
                invalid_param = true;
                break;
            }
        }
        else if (arg == "-b" || arg == "--baudrate")
        {
            if (++i >= argc)
            {
                invalid_param = true;
                break;
            }
            params.uart_baud_rate = std::stoi(argv[i]);
        }
        else if (arg == "-s" || arg == "--stop-bits")
        {
            if (++i >= argc)
            {
                invalid_param = true;
                break;
            }
            params.uart_stop_bits = std::stoi(argv[i]);
            if (params.uart_stop_bits != 1 && params.uart_stop_bits != 2)
            {
                invalid_param = true;
                break;
            }
        }
        else if (arg == "--usb")
        {
            params.connect_usb = true;
        }
        else if (arg == "-p" || arg == "--port")
        {
            if (++i >= argc)
            {
                invalid_param = true;
                break;
            }
            params.port = std::stoi(argv[i]);
        }
        else if (arg == "-w" || arg == "--wait-input")
        {
            params.wait_for_input_after_exit = true;
        }
        else if (arg == "-nc" || arg == "--no-clear")
        {
            params.clear_errors_on_startup = false;
        }
        else
            throw std::invalid_argument("error: unknown argument: " + arg);
    }
    if (invalid_param)
    {
        throw std::invalid_argument("error: invalid parameter for argument: " + arg);
    }
    if (!params.connect_usb && !params.connect_uart)
    {
        return false;
    }
    if (params.connect_usb && params.connect_uart)
    {
        throw std::invalid_argument("error: invalid arguments\n");
    }

    return true;
}

void params_parse(int argc, char ** argv, Params& params)
{
    try
    {
        if (!params_parse_ex(argc, argv, params))
        {
            print_usage(argc, argv, Params());
            exit(0);
        }
    }
    catch (const std::invalid_argument & ex)
    {
        fprintf(stderr, "%s\n", ex.what());
        print_usage(argc, argv, Params());
        exit(1);
    }
}


int main(int argc, char *argv[])
{
	int result = EXIT_FAILURE;
	u64_micros last_time;
	setup_handlers();

    Params params;
    params_parse(argc, argv, params);

	time_init();
	net_startup();

	if (!odrive_control_init(params)) goto fail;
	if (!server_init(params))         goto fail;
	
	last_time = time_micros_64();
	md.delta_time = 0.004f;
	
	while (running)
	{
		if (!odrive_control_update()) goto fail;
		if (!server_update())         goto fail;
		
		// calculate delta time
		u64_micros time_before_sleep = time_micros_64();
		{
			double target_delta = cd.target_delta_time_ms * 0.001;
			double delta = double(time_before_sleep - last_time) * .000001;
			double remaining = target_delta - delta;
			precise_sleep(remaining);
		}

		u64_micros current_time = time_micros_64();
		md.delta_time_sleep = (u32_micros)(current_time - time_before_sleep);

		md.delta_time = float(current_time - last_time) * .000001f;
		last_time = current_time;

		md.uptime_micros = current_time;
		time(&md.local_time);

		md.counter++;
	}
	printf("\n");
	result = EXIT_SUCCESS;
fail:
	server_close();
	odrive_control_close();
	net_shutdown();

    if (params.wait_for_input_after_exit)
    {
	    // Allow the user to see the error message
        printf("Press Enter to Exit\n");
        char ch;
        int dummy = scanf("%c", &ch);
    }
	return result;
}
