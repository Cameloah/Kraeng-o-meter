//
// Created by koorj on 19.01.2022.
//

#pragma once


extern bool enable_serial_stream;
extern bool enable_serial_verbose;
extern bool enable_measurements;

/// \brief writes out info such as firmware version
String ui_info();

/// \brief checks for user input via serial comm and offers commands for accessing internal features
void ui_serial_comm_handler();