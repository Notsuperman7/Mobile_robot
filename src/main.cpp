#include <Arduino.h>
#include "pid.h"
#include "config_wheels.h"
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

rcl_subscription_t fl_subscriber, fr_subscriber, bl_subscriber, br_subscriber;
std_msgs__msg__Float32 fl_msg, fr_msg, bl_msg, br_msg;
rcl_publisher_t fl_encoder_publisher, fr_encoder_publisher, bl_encoder_publisher, br_encoder_publisher;
std_msgs__msg__Float32 fl_encoder_msg, fr_encoder_msg, bl_encoder_msg, br_encoder_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Shared variable for motor target speeds (updated by ROS callback) - one per wheel
volatile float wheel_targets[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // FL, FR, BL, BR
SemaphoreHandle_t targets_mutex;

// Shared encoder speeds - protected by mutex for multi-task access
volatile float encoder_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // FL, FR, BL, BR
SemaphoreHandle_t encoder_mutex;

PIController pi_BL((Kp_BL) , (Ki_BL) , (Kd_BL), PWM_MIN, PWM_MAX);
PIController pi_FL((Kp_FL) , (Ki_FL) , (Kd_FL), PWM_MIN, PWM_MAX);
PIController pi_BR((Kp_BR) , (Ki_BR) , (Kd_BR), PWM_MIN, PWM_MAX);
PIController pi_FR((Kp_FR) , (Ki_FR) , (Kd_FR), PWM_MIN, PWM_MAX);


QueueHandle_t BR_target_queue;
QueueHandle_t FR_target_queue;
QueueHandle_t BL_target_queue;
QueueHandle_t FL_target_queue;

typedef struct MotorConfig MotorConfig;

MotorConfig BL_Motor = {
    .EN_pin = BL_EN,
    .IN1_pin = BL_IN1,
    .IN2_pin = BL_IN2,
    .name = "BL",
    .pi = pi_BL,
    .encoderCount = 0,
    .M_queue = NULL
};

MotorConfig FL_Motor = {
    .EN_pin = FL_EN,
    .IN1_pin = FL_IN1,
    .IN2_pin = FL_IN2,
    .name = "FL",
    .pi = pi_FL,
    .encoderCount = 0,
    .M_queue = NULL
};

MotorConfig BR_Motor = {
    .EN_pin = BR_EN,
    .IN1_pin = BR_IN1,
    .IN2_pin = BR_IN2,
    .name = "BR",
    .pi = pi_BR,
    .encoderCount = 0,
    .M_queue = NULL
};

MotorConfig FR_Motor = {
    .EN_pin = FR_EN,
    .IN1_pin = FR_IN1,
    .IN2_pin = FR_IN2,
    .name = "FR",
    .pi = pi_FR,
    .encoderCount = 0,
    .M_queue = NULL
};

// ROS callback functions for individual wheel speed commands
void fl_speed_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    xSemaphoreTake(targets_mutex, portMAX_DELAY);
    wheel_targets[0] = msg->data; // FL
    xSemaphoreGive(targets_mutex);
}

void fr_speed_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    xSemaphoreTake(targets_mutex, portMAX_DELAY);
    wheel_targets[1] = msg->data; // FR
    xSemaphoreGive(targets_mutex);
}

void bl_speed_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    xSemaphoreTake(targets_mutex, portMAX_DELAY);
    wheel_targets[2] = msg->data; // BL
    xSemaphoreGive(targets_mutex);
}

void br_speed_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    xSemaphoreTake(targets_mutex, portMAX_DELAY);
    wheel_targets[3] = msg->data; // BR
    xSemaphoreGive(targets_mutex);
}

void rosExecutorTask(void* pvprm) {
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void startMotors(void* pvprm) {
    while (true) {
        xSemaphoreTake(targets_mutex, portMAX_DELAY);
        float targets[4];
        memcpy(targets, (float*)wheel_targets, sizeof(targets));
        xSemaphoreGive(targets_mutex);
        
        // Send individual wheel targets to motor queues
        xQueueSend(FL_target_queue, &targets[0], 0); // FL
        xQueueSend(FR_target_queue, &targets[1], 0); // FR
        xQueueSend(BL_target_queue, &targets[2], 0); // BL
        xQueueSend(BR_target_queue, &targets[3], 0); // BR

        // Update shared encoder speeds with mutex protection
        xSemaphoreTake(encoder_mutex, portMAX_DELAY);
        encoder_speeds[0] = FL_Motor.currentRPM;
        encoder_speeds[1] = FR_Motor.currentRPM;
        encoder_speeds[2] = BL_Motor.currentRPM;
        encoder_speeds[3] = BR_Motor.currentRPM;
        xSemaphoreGive(encoder_mutex);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void encoderPublisherTask(void* pvprm) {
    while (true) {
        // Read encoder speeds with mutex protection
        xSemaphoreTake(encoder_mutex, portMAX_DELAY);
        float fl_rpm = encoder_speeds[0];
        float fr_rpm = encoder_speeds[1];
        float bl_rpm = encoder_speeds[2];
        float br_rpm = encoder_speeds[3];
        xSemaphoreGive(encoder_mutex);

        // Set message data with current RPM values
        fl_encoder_msg.data = fl_rpm;
        fr_encoder_msg.data = fr_rpm;
        bl_encoder_msg.data = bl_rpm;
        br_encoder_msg.data = br_rpm;

        // Publish to ROS
        rcl_publish(&fl_encoder_publisher, &fl_encoder_msg, NULL);
        rcl_publish(&fr_encoder_publisher, &fr_encoder_msg, NULL);
        rcl_publish(&bl_encoder_publisher, &bl_encoder_msg, NULL);
        rcl_publish(&br_encoder_publisher, &br_encoder_msg, NULL);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(9600);

    delay(3000);

    // Motor pins
    pinMode(BL_IN1, OUTPUT);
    pinMode(BL_IN2, OUTPUT);
    pinMode(BL_EN, OUTPUT);

    pinMode(FL_IN1, OUTPUT);
    pinMode(FL_IN2, OUTPUT);
    pinMode(FL_EN, OUTPUT);

    pinMode(BR_IN1, OUTPUT);
    pinMode(BR_IN2, OUTPUT);
    pinMode(BR_EN, OUTPUT);

    pinMode(FR_IN1, OUTPUT);
    pinMode(FR_IN2, OUTPUT);
    pinMode(FR_EN, OUTPUT);

    // Encoder pins
    pinMode(BL_ENC_A, INPUT_PULLUP);
    pinMode(BL_ENC_B, INPUT_PULLUP);

    pinMode(BR_ENC_A, INPUT_PULLUP);
    pinMode(BR_ENC_B, INPUT_PULLUP);

    pinMode(FL_ENC_A, INPUT_PULLUP);
    pinMode(FL_ENC_B, INPUT_PULLUP);

    pinMode(FR_ENC_A, INPUT_PULLUP);
    pinMode(FR_ENC_B, INPUT_PULLUP);

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

    // Initial targets
    BR_target_queue = xQueueCreate(10, sizeof(float));
    FR_target_queue = xQueueCreate(10, sizeof(float));
    BL_target_queue = xQueueCreate(10, sizeof(float));
    FL_target_queue = xQueueCreate(10, sizeof(float));

    BL_Motor.M_queue = BL_target_queue;
    FL_Motor.M_queue = FL_target_queue;
    BR_Motor.M_queue = BR_target_queue;
    FR_Motor.M_queue = FR_target_queue;

    // Initialize targets mutex
    targets_mutex = xSemaphoreCreateMutex();
    
    // Initialize encoder mutex
    encoder_mutex = xSemaphoreCreateMutex();

    // Initialize micro-ROS
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    
    // Create init_options
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        // Support initialization failed, halt
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    // Create node
    ret = rclc_node_init_default(&node, "mobile_robot_node", "", &support);
    if (ret != RCL_RET_OK) {
        // Node initialization failed, halt
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    // Create subscribers for each wheel
    rclc_subscription_init_default(
        &fl_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/wheel_front_left_speed");
    
    rclc_subscription_init_default(
        &fr_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/wheel_front_right_speed");
    
    rclc_subscription_init_default(
        &bl_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/wheel_rear_left_speed");
    
    rclc_subscription_init_default(
        &br_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/wheel_rear_right_speed");
    
   // Create publishers for individual encoder RPMs
    rclc_publisher_init_default(
         &fl_encoder_publisher,
        &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
         "/fl_encoder_rpm");
    
    rclc_publisher_init_default(
         &fr_encoder_publisher,
        &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
         "/fr_encoder_rpm");
    
    rclc_publisher_init_default(
         &bl_encoder_publisher,
        &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
         "/bl_encoder_rpm");
    
    rclc_publisher_init_default(
         &br_encoder_publisher,
        &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
         "/br_encoder_rpm");
    
    // Initialize individual encoder messages
    std_msgs__msg__Float32__init(&fl_encoder_msg);
    std_msgs__msg__Float32__init(&fr_encoder_msg);
    std_msgs__msg__Float32__init(&bl_encoder_msg);
    std_msgs__msg__Float32__init(&br_encoder_msg);
     //Create executor with 4 subscriptions
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_subscription(&executor, &fl_subscriber, &fl_msg, &fl_speed_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &fr_subscriber, &fr_msg, &fr_speed_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &bl_subscriber, &bl_msg, &bl_speed_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &br_subscriber, &br_msg, &br_speed_callback, ON_NEW_DATA);

    // Create PID task
    xTaskCreate(
        apply_pid,
        "BL_Task",
        4096,
        (void *)&BL_Motor,
        1,
        NULL
    );
        xTaskCreate(
        apply_pid,
        "FL_Task",
        4096,
        (void *)&FL_Motor,
        1,
        NULL
    );
        xTaskCreate(
        apply_pid,
        "BR_Task",
        4096,
        (void *)&BR_Motor,
        1,
        NULL
    );
        xTaskCreate(
        apply_pid,
        "FR_Task",
        4096,
        (void *)&FR_Motor,
        1,
        NULL
    );
        xTaskCreate(
        startMotors,
        "Start_Motors",
        4096,
        NULL,
        2,
        NULL
    );
    
    xTaskCreate(
        rosExecutorTask,
        "ROS_Executor",
        4096,
        NULL,
        1,
        NULL
    );
    
    xTaskCreate(
        encoderPublisherTask,
        "Encoder_Publisher",
        4096,
        NULL,
        2,
        NULL
    );
}

void loop() {
    // ROS executor now runs in its own FreeRTOS task
    vTaskDelay(pdMS_TO_TICKS(100));
}