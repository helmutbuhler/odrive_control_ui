
#include "../common/common.h"
#include <string>

struct Params
{
    bool connect_usb = false;
    bool connect_uart = false;
    std::string uart_address;
    int uart_baud_rate = 115200;
    int uart_stop_bits = 2;
    u16 port = ::port;
    bool wait_for_input_after_exit = false;
    bool clear_errors_on_startup = true;
};

extern bool running;
extern MonitorData md;
extern ControlData cd;

