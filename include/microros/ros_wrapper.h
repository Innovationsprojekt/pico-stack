/**
 * @file ros_wrapper.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_ROS_WRAPPER_H_
#define PICO_MOTORS_ROS_WRAPPER_H_

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"

void ROSWrapper();

#endif //PICO_MOTORS_ROS_WRAPPER_H_
