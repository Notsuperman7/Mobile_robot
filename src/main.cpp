#include <Arduino.h>
#include "pid.h"
#include "config_wheels.h"
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h>

rcl_subscription_t wheel_cmds_subscriber;
rcl_publisher_t encoder_feedback_publisher;

std_msgs__msg__Float32MultiArray wheel_cmds_msg;
std_msgs__msg__Float32MultiArray encoder_feedback_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;


volatile bool micro_ros_connected = false;
static float wheel_cmds_buffer[4];

// Order: FL, FR, BL, BR
volatile float wheel_targets[4] = {0.0f, 0.0f, 0.0f, 0.0f};
SemaphoreHandle_t targets_mutex;// Shared variable for motor target speeds (updated by ROS callback)


// Shared encoder speeds
// Order: FL, FR, BL, BR
volatile float encoder_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
SemaphoreHandle_t encoder_mutex;

PIController pi_BL((Kp_BL), (Ki_BL), (Kd_BL), PWM_MIN, PWM_MAX);
PIController pi_FL((Kp_FL), (Ki_FL), (Kd_FL), PWM_MIN, PWM_MAX);
PIController pi_BR((Kp_BR), (Ki_BR), (Kd_BR), PWM_MIN, PWM_MAX);
PIController pi_FR((Kp_FR), (Ki_FR), (Kd_FR), PWM_MIN, PWM_MAX);

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
    .M_queue = NULL,
    .currentRPM = 0.0f
};

MotorConfig FL_Motor = {
    .EN_pin = FL_EN,
    .IN1_pin = FL_IN1,
    .IN2_pin = FL_IN2,
    .name = "FL",
    .pi = pi_FL,
    .encoderCount = 0,
    .M_queue = NULL,
    .currentRPM = 0.0f
};

MotorConfig BR_Motor = {
    .EN_pin = BR_EN,
    .IN1_pin = BR_IN1,
    .IN2_pin = BR_IN2,
    .name = "BR",
    .pi = pi_BR,
    .encoderCount = 0,
    .M_queue = NULL,
    .currentRPM = 0.0f
};

MotorConfig FR_Motor = {
    .EN_pin = FR_EN,
    .IN1_pin = FR_IN1,
    .IN2_pin = FR_IN2,
    .name = "FR",
    .pi = pi_FR,
    .encoderCount = 0,
    .M_queue = NULL,
    .currentRPM = 0.0f
};

// Static buffer for outgoing encoder feedback
static float encoder_feedback_buffer[4];

void wheel_cmds_callback(const void *msg_in) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_in;

    if (msg == NULL || msg->data.data == NULL || msg->data.size < 4) {
        return;
    }

    xSemaphoreTake(targets_mutex, portMAX_DELAY);
    wheel_targets[0] = msg->data.data[0]; // FL
    wheel_targets[1] = msg->data.data[1]; // FR
    wheel_targets[2] = msg->data.data[2]; // BL
    wheel_targets[3] = msg->data.data[3]; // BR
    xSemaphoreGive(targets_mutex);
}

void rosExecutorTask(void *pvprm) {
    (void)pvprm;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void startMotors(void *pvprm) {
    (void)pvprm;
    while (true) {
        float targets[4];

        xSemaphoreTake(targets_mutex, portMAX_DELAY);
        memcpy(targets, (const void *)wheel_targets, sizeof(targets));
        xSemaphoreGive(targets_mutex);

        // Send individual wheel targets to motor queues
        xQueueSend(FL_target_queue, &targets[0], 0); // FL
        xQueueSend(FR_target_queue, &targets[1], 0); // FR
        xQueueSend(BL_target_queue, &targets[2], 0); // BL
        xQueueSend(BR_target_queue, &targets[3], 0); // BR

        // Update shared encoder speeds
        xSemaphoreTake(encoder_mutex, portMAX_DELAY);
        encoder_speeds[0] = FL_Motor.currentRPM;
        encoder_speeds[1] = FR_Motor.currentRPM;
        encoder_speeds[2] = BL_Motor.currentRPM;
        encoder_speeds[3] = BR_Motor.currentRPM;
        xSemaphoreGive(encoder_mutex);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void encoderPublisherTask(void *pvprm) {
    (void)pvprm;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz

    while (true) {
        xSemaphoreTake(encoder_mutex, portMAX_DELAY);
        encoder_feedback_buffer[0] = encoder_speeds[0];
        encoder_feedback_buffer[1] = encoder_speeds[1];
        encoder_feedback_buffer[2] = encoder_speeds[2];
        encoder_feedback_buffer[3] = encoder_speeds[3];
        xSemaphoreGive(encoder_mutex);

        encoder_feedback_msg.data.data = encoder_feedback_buffer;
        encoder_feedback_msg.data.size = 4;
        encoder_feedback_msg.data.capacity = 4;

        rcl_ret_t ret = rcl_publish(&encoder_feedback_publisher, &encoder_feedback_msg, NULL);
        if (ret != RCL_RET_OK) {
            ESP.restart();
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

    ret = rclc_subscription_init_default(
        &wheel_cmds_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_cmds");
    if (ret != RCL_RET_OK) return false;

    ret = rclc_publisher_init_default(
        &encoder_feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/encoder_feedback");
    if (ret != RCL_RET_OK) return false;

    std_msgs__msg__Float32MultiArray__init(&wheel_cmds_msg);
    std_msgs__msg__Float32MultiArray__init(&encoder_feedback_msg);

    wheel_cmds_msg.data.data = wheel_cmds_buffer;
    wheel_cmds_msg.data.size = 0;
    wheel_cmds_msg.data.capacity = 4;
    wheel_cmds_msg.layout.dim.data = NULL;
    wheel_cmds_msg.layout.dim.size = 0;
    wheel_cmds_msg.layout.dim.capacity = 0;
    wheel_cmds_msg.layout.data_offset = 0;

    encoder_feedback_msg.data.data = encoder_feedback_buffer;
    encoder_feedback_msg.data.size = 4;
    encoder_feedback_msg.data.capacity = 4;
    encoder_feedback_msg.layout.dim.data = NULL;
    encoder_feedback_msg.layout.dim.size = 0;
    encoder_feedback_msg.layout.dim.capacity = 0;
    encoder_feedback_msg.layout.data_offset = 0;

    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_executor_add_subscription(
        &executor,
        &wheel_cmds_subscriber,
        &wheel_cmds_msg,
        &wheel_cmds_callback,
        ON_NEW_DATA);
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

                    xTaskCreate(rosExecutorTask, "ROS_Executor", 8192, NULL, 1, NULL);
                    xTaskCreate(encoderPublisherTask, "Encoder_Publisher", 4096, NULL, 4, NULL);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void setup() {
    Serial.begin(921600);
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

    // Queues
    BR_target_queue = xQueueCreate(10, sizeof(float));
    FR_target_queue = xQueueCreate(10, sizeof(float));
    BL_target_queue = xQueueCreate(10, sizeof(float));
    FL_target_queue = xQueueCreate(10, sizeof(float));

    BL_Motor.M_queue = BL_target_queue;
    FL_Motor.M_queue = FL_target_queue;
    BR_Motor.M_queue = BR_target_queue;
    FR_Motor.M_queue = FR_target_queue;

    // Mutexes
    targets_mutex = xSemaphoreCreateMutex();
    encoder_mutex = xSemaphoreCreateMutex();

    // micro-ROS transport
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();

    xTaskCreate(
    microRosConnectionTask,
    "MicroROS_Conn",
    8192,
    NULL,
    2,
    NULL
    );
    // Create PID tasks
    xTaskCreate(
        apply_pid,
        "BL_Task",
        4096,
        (void *)&BL_Motor,
        4,
        NULL
    );

    xTaskCreate(
        apply_pid,
        "FL_Task",
        4096,
        (void *)&FL_Motor,
        4,
        NULL
    );

    xTaskCreate(
        apply_pid,
        "BR_Task",
        4096,
        (void *)&BR_Motor,
        4,
        NULL
    );

    xTaskCreate(
        apply_pid,
        "FR_Task",
        4096,
        (void *)&FR_Motor,
        4,
        NULL
    );

    xTaskCreate(
        startMotors,
        "Start_Motors",
        4096,
        NULL,
        3,
        NULL
    );

}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10));
}