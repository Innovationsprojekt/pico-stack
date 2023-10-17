/**
 * @file ros_wrapper.cpp
 * @author Noa Sendlhofer
 */

#include "ros_wrapper.h"
#include "pico_udp_transport.h"

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_pub;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_publish(&publisher, &msg_pub, nullptr);
    msg_pub.data++;
}

ROSWrapper::ROSWrapper()
{
    set_microros_wifi_transports("WN-8EFE60", "47ec527533", "PC IP ADDRESS", 4444);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
        throw std::runtime_error("Agent unreachable");

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "Pico Node", "", &support);

    rclc_publisher_init_best_effort(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pico/temperature");

    rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(500),
            timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    msg_pub.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}
