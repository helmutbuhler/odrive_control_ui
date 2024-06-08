# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)


project(control_ui)
include_directories(3rdparty/imgui)
add_executable(control_ui
	3rdparty/imgui/imgui.cpp
	3rdparty/imgui/imgui_demo.cpp
	3rdparty/imgui/imgui_draw.cpp
	3rdparty/imgui/imgui_impl_glfw.cpp
	3rdparty/imgui/imgui_impl_opengl2.cpp
	3rdparty/imgui/imgui_tables.cpp
	3rdparty/imgui/imgui_widgets.cpp

	3rdparty/implot/implot.cpp
	3rdparty/implot/implot_demo.cpp
	3rdparty/implot/implot_items.cpp

	common/network.cpp
	common/time_helper.cpp

	control_ui/control_ui.cpp
	control_ui/control_ui_client.cpp
	control_ui/plot.cpp
	)
target_compile_definitions(control_ui PUBLIC COMPILING_CONTROL_UI)
if (CMAKE_COMPILER_IS_GNUCC)
	target_compile_options(control_ui PRIVATE -Wfloat-conversion)
endif()
target_link_libraries(control_ui glfw GL)


project(proxy)
add_executable(proxy
	common/odrive/ODrive.cpp
	common/odrive/endpoint.cpp

	common/helper.cpp
	common/network.cpp
	common/time_helper.cpp

	proxy/odrive_control.cpp
	proxy/main.cpp
	proxy/server.cpp
	)
if (CMAKE_COMPILER_IS_GNUCC)
	target_compile_options(proxy PRIVATE -Wfloat-conversion)
endif()
target_link_libraries(proxy pthread usb-1.0)


