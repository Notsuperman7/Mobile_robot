#include <Arduino.h>
#include <ESP32Servo.h> // REQUIRED: Add this to your platformio.ini dependencies
#include "pid.h"
#include "config_wheels.h"
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int8.h> // Added for servo topics
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>

// --- ROS Entities ---
rcl_subscription_t wheel_cmds_subscriber;
rcl_subscription_t gripper_subscriber;
rcl_subscription_t arm_subscriber;
rcl_publisher_t encoder_feedback_publisher;
rcl_publisher_t battery_publisher;

std_msgs__msg__Float32 battery_msg;
std_msgs__msg__Int16MultiArray wheel_cmds_msg;
std_msgs__msg__Int16MultiArray encoder_feedback_msg;
std_msgs__msg__UInt8 gripper_msg;
std_msgs__msg__UInt8 arm_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

volatile bool micro_ros_connected = false;
static int16_t wheel_cmds_buffer[4];
static int16_t encoder_feedback_buffer[4];

// --- Task Handles ---
TaskHandle_t ros_executor_handle = NULL;
TaskHandle_t encoder_pub_handle = NULL;
TaskHandle_t battery_pub_handle = NULL;

// --- Motor & Servo Objects ---
PIController pi_BL((Kp_BL), (Ki_BL), (Kd_BL), PWM_MIN, PWM_MAX);
PIController pi_FL((Kp_FL), (Ki_FL), (Kd_FL), PWM_MIN, PWM_MAX);
PIController pi_BR((Kp_BR), (Ki_BR), (Kd_BR), PWM_MIN, PWM_MAX);
PIController pi_FR((Kp_FR), (Ki_FR), (Kd_FR), PWM_MIN, PWM_MAX);

Servo gripper_servo;
Servo arm_servo;

QueueHandle_t BR_target_queue;
QueueHandle_t FR_target_queue;
QueueHandle_t BL_target_queue;
QueueHandle_t FL_target_queue;

typedef struct MotorConfig MotorConfig;

MotorConfig BL_Motor = { BL_EN, BL_IN1, BL_IN2, "BL", pi_BL, 0, NULL, 0.0f };
MotorConfig FL_Motor = { FL_EN, FL_IN1, FL_IN2, "FL", pi_FL, 0, NULL, 0.0f };
MotorConfig BR_Motor = { BR_EN, BR_IN1, BR_IN2, "BR", pi_BR, 0, NULL, 0.0f };
MotorConfig FR_Motor = { FR_EN, FR_IN1, FR_IN2, "FR", pi_FR, 0, NULL, 0.0f };

// --- Callbacks ---
void wheel_cmds_callback(const void *msg_in) {
    const std_msgs__msg__Int16MultiArray *msg = (const std_msgs__msg__Int16MultiArray *)msg_in;
    if (msg == NULL || msg->data.data == NULL || msg->data.size < 4) return;

    int32_t t_FL = msg->data.data[0];
    int32_t t_FR = msg->data.data[1];
    int32_t t_BL = msg->data.data[2];
    int32_t t_BR = msg->data.data[3];

    xQueueOverwrite(FL_target_queue, &t_FL);
    xQueueOverwrite(FR_target_queue, &t_FR);
    xQueueOverwrite(BL_target_queue, &t_BL);
    xQueueOverwrite(BR_target_queue, &t_BR);
}

void gripper_callback(const void *msg_in) {
    const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msg_in;
    // Constrain to safe servo angles
    int angle = constrain(msg->data, 0, 180);
    gripper_servo.write(angle);
}

void arm_callback(const void *msg_in) {
    const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msg_in;
    // Constrain to safe servo angles
    int angle = constrain(msg->data, 0, 180);
    arm_servo.write(angle);
}

void batteryPublisherTask(void *pvprm) {
    (void)pvprm;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000); // Publish at 1 Hz

    // Ensure 12-bit resolution (0-4095)
    analogReadResolution(12);

    while (true) {
        // Read the raw ADC value
        int raw_adc = analogRead(BATTERY_PIN);
        
        // Calculate voltage: (raw / max_resolution) * reference_voltage * external_multiplier
        float voltage = (raw_adc / 4095.0f) * 3.3f * 5.0f + 1.0f;

        battery_msg.data = voltage;

        // Publish to micro-ROS
        rcl_publish(&battery_publisher, &battery_msg, NULL);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

// --- Tasks ---
void rosExecutorTask(void *pvprm) {
    (void)pvprm;
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void encoderPublisherTask(void *pvprm) {
    (void)pvprm;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);
    int consecutive_errors = 0;

    while (true) {
        encoder_feedback_buffer[0] = (int16_t)FL_Motor.currentRPM;
        encoder_feedback_buffer[1] = (int16_t)FR_Motor.currentRPM;
        encoder_feedback_buffer[2] = (int16_t)BL_Motor.currentRPM;
        encoder_feedback_buffer[3] = (int16_t)BR_Motor.currentRPM;

        encoder_feedback_msg.data.data = encoder_feedback_buffer;
        encoder_feedback_msg.data.size = 4;
        encoder_feedback_msg.data.capacity = 4;

        rcl_ret_t ret = rcl_publish(&encoder_feedback_publisher, &encoder_feedback_msg, NULL);
        
        if (ret != RCL_RET_OK) {
            consecutive_errors++;
            if (consecutive_errors >= 10) ESP.restart();
        } else {
            consecutive_errors = 0;
        }
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

bool createMicroRosEntities() {
    rcl_ret_t ret;

    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_node_init_default(&node, "mobile_robot_node", "", &support);
    if (ret != RCL_RET_OK) return false;

    // --- Subscriptions ---
    ret = rclc_subscription_init_best_effort(
        &wheel_cmds_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/wheel_cmds");
    if (ret != RCL_RET_OK) return false;

    ret = rclc_subscription_init_best_effort(
        &gripper_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/gripper_angle");
    if (ret != RCL_RET_OK) return false;

    ret = rclc_subscription_init_best_effort(
        &arm_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/arm_angle");
    if (ret != RCL_RET_OK) return false;

    // --- Publishers ---
    ret = rclc_publisher_init_best_effort(
        &encoder_feedback_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/encoder_feedback");
    if (ret != RCL_RET_OK) return false;

    // --- Battery Publisher ---
    ret = rclc_publisher_init_best_effort(
        &battery_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery_volt");
    if (ret != RCL_RET_OK) return false;

    // --- Init Messages ---
    std_msgs__msg__Int16MultiArray__init(&wheel_cmds_msg);
    std_msgs__msg__Int16MultiArray__init(&encoder_feedback_msg);
    std_msgs__msg__Float32__init(&battery_msg);

    wheel_cmds_msg.data.data = wheel_cmds_buffer;
    wheel_cmds_msg.data.size = 0;
    wheel_cmds_msg.data.capacity = 4;

    // --- Executor ---
    // Note: Capacity increased to 3 to handle wheel_cmds, gripper, and arm!
    ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_executor_add_subscription(
        &executor, &wheel_cmds_subscriber, &wheel_cmds_msg, &wheel_cmds_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_executor_add_subscription(
        &executor, &gripper_subscriber, &gripper_msg, &gripper_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_executor_add_subscription(
        &executor, &arm_subscriber, &arm_msg, &arm_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return false;

    return true;
}

void microRosConnectionTask(void *pvprm) {
    (void)pvprm;
    while (true) {
        if (!micro_ros_connected) {
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (createMicroRosEntities()) {
                    micro_ros_connected = true;
                    if (ros_executor_handle == NULL) xTaskCreate(rosExecutorTask, "ROS_Executor", 8192, NULL, 4, &ros_executor_handle);
                    if (encoder_pub_handle == NULL) xTaskCreate(encoderPublisherTask, "Encoder_Publisher", 4096, NULL, 4, &encoder_pub_handle);
                    
                    if (battery_pub_handle == NULL) xTaskCreate(batteryPublisherTask, "Battery_Publisher", 2048, NULL, 3, &battery_pub_handle);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(921600);
    delay(100);

    // Servos
    // Ensure ESP32 timers are allocated for PWM
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    //input pin for battery voltage measurement
    pinMode(BATTERY_PIN, INPUT);
    
    // Attach servos using the pins defined in config_wheels.h
    // The default min/max pulse widths are typically 544/2400 for standard servos
    gripper_servo.setPeriodHertz(50);
    gripper_servo.attach(GRIPPER_PIN, 500, 2500); 
    
    arm_servo.setPeriodHertz(50);
    arm_servo.attach(ARM_PIN, 500, 2500);

    // Motor pins
    pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT); pinMode(BL_EN, OUTPUT);
    pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
    pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT); pinMode(BR_EN, OUTPUT);
    pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_EN, OUTPUT);

    // Encoder pins
    pinMode(BL_ENC_A, INPUT_PULLUP); pinMode(BL_ENC_B, INPUT_PULLUP);
    pinMode(BR_ENC_A, INPUT_PULLUP); pinMode(BR_ENC_B, INPUT_PULLUP);
    pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
    pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);

    // Stop motors at startup
    setMotor(BL_EN, BL_IN1, BL_IN2, 0);
    setMotor(FL_EN, FL_IN1, FL_IN2, 0);
    setMotor(BR_EN, BR_IN1, BR_IN2, 0);
    setMotor(FR_EN, FR_IN1, FR_IN2, 0);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A), encoderISR_BL, RISING);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A), encoderISR_BR, RISING);
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), encoderISR_FL, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), encoderISR_FR, RISING);

    // Queues
    BR_target_queue = xQueueCreate(1, sizeof(int32_t));
    FR_target_queue = xQueueCreate(1, sizeof(int32_t));
    BL_target_queue = xQueueCreate(1, sizeof(int32_t));
    FL_target_queue = xQueueCreate(1, sizeof(int32_t));

    BL_Motor.M_queue = BL_target_queue;
    FL_Motor.M_queue = FL_target_queue;
    BR_Motor.M_queue = BR_target_queue;
    FR_Motor.M_queue = FR_target_queue;

    // micro-ROS transport
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();

    xTaskCreate(microRosConnectionTask, "MicroROS_Conn", 8192, NULL, 2, NULL);
    
    // Create PID tasks
    xTaskCreate(apply_pid, "BL_Task", 4096, (void *)&BL_Motor, 4, NULL);
    xTaskCreate(apply_pid, "FL_Task", 4096, (void *)&FL_Motor, 4, NULL);
    xTaskCreate(apply_pid, "BR_Task", 4096, (void *)&BR_Motor, 4, NULL);
    xTaskCreate(apply_pid, "FR_Task", 4096, (void *)&FR_Motor, 4, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10));
}