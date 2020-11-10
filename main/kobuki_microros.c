#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>

#include <geometry_msgs/msg/twist.h>
#include <drive_base_msgs/msg/base_info.h>
#include <std_msgs/msg/int32.h>

#include <kobuki.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t base_info_pub;
rcl_subscription_t cmd_vel_sub;

drive_base_msgs__msg__BaseInfo base_info;
geometry_msgs__msg__Twist cmd_vel;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	kobuki_status_t robot_status = kobuki_get_status();

	base_info.hw_id = robot_status.hw_id.UDID_0;
	base_info.hw_timestamp = robot_status.last_basic_sensor_data.timestamp;
	// base_info.stamp = 0;
	base_info.x = robot_status.odometry.x;
	base_info.y = robot_status.odometry.y;
	base_info.orientation = robot_status.odometry.theta;
	base_info.forward_velocity = robot_status.odometry.linear_velocity;
	base_info.rotational_velocity = robot_status.odometry.angular_velocity;
	base_info.battery_voltage_pct = 0;
	base_info.power_supply = robot_status.last_basic_sensor_data.charger;
	base_info.overcurrent = robot_status.last_basic_sensor_data.overcurrent;
	base_info.blocked = robot_status.last_basic_sensor_data.bumper;
	base_info.in_collision = robot_status.last_basic_sensor_data.bumper;
	base_info.at_cliff = robot_status.last_basic_sensor_data.cliff;
	base_info.safety_state = 0;

	RCSOFTCHECK(rcl_publish(&base_info_pub, &base_info, NULL));
}

bool em_stop = false;

void cmd_vel_callback(const void * msgin)
{
	geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
	if (!em_stop){
		kobuki_set_speed_command(msg->linear.x/3, msg->angular.z);
	}
	
}

void kobuki_emergency(kobuki_subpayload_t * msg, bool emergency){
	if (emergency){
		kobuki_set_speed_command(0.0, 0.0);
    	printf("EMERGENCY STOP\n");
		em_stop = true;
	}else{
		em_stop = false;
	}
}

void appMain(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	// RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_kobuki", "", &support));

	// create base info publisher
	RCCHECK(rclc_publisher_init_best_effort(
		&base_info_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drive_base_msgs, msg, BaseInfo),
		"base_info"));

	// create cmd_vel subscriber
	RCCHECK(rclc_subscription_init_best_effort(
		&cmd_vel_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// create timer,
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 200;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel, cmd_vel_callback, ON_NEW_DATA));

	// init Kobuki
	kobuki_init_serial();
    kobuki_set_emergency_callback(kobuki_emergency);

	TaskHandle_t kobuki_handle = NULL;
    xTaskCreate(kobuki_loop, "kobuki_thread", 2000, NULL,  5, &kobuki_handle); 

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&cmd_vel_sub, &node))
	RCCHECK(rcl_publisher_fini(&base_info_pub, &node))
	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}
