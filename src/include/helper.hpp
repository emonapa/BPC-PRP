#pragma once

#include <iostream>
#include <string>

static const int MAIN_LOOP_PERIOD_MS = 50;

namespace Topic {
   const std::string buttons = "/bpc_prp_robot/buttons";
   const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
   const std::string set_motor_speed = "/bpc_prp_robot/set_motor_speeds";
   const std::string get_motor_speed = "/bpc_prp_robot/encoders";
   const std::string get_line = "/bpc_prp_robot/line_sensors";
   const std::string get_lidar="/bpc_prp_robot/lidar";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};
