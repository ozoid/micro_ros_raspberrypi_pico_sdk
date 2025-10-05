#include <stdio.h>
#include <math.h>
// cd ..
// rm -rf build
// mkdir build && cd cuild
// cmake .. -DPICO_SDK_PATH=/home/pi/pico-sdk -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/home/pi/pico-sdk/cmake/preload/toolchains/pico_arm_cortex_m0plus_gcc.cmake
// make -j$(nproc)

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
//#include "time_override.c"
#include "task.h"

#include "pico_uart_transports.h"


#define I2C_PORT i2c1
#define SDA_PIN  14
#define SCL_PIN  15
#define BNO055_ADDR 0x28
#define BMP280_ADDR 0x77

// DRV8833 pins
#define LEFT_FWD 10
#define LEFT_REV 11
#define RIGHT_FWD 12
#define RIGHT_REV 13

// Encoder pins
#define LEFT_ENC_A 18
#define LEFT_ENC_B 19
#define RIGHT_ENC_A 20
#define RIGHT_ENC_B 21

// Robot params
#define WHEEL_RADIUS 0.045f  // 9 cm
#define WHEEL_BASE   0.09f  // 9 cm centre of front axle to centre of back axle
#define TICKS_PER_REV 80

SemaphoreHandle_t g_rcl_mutex;   // global
#define RCL_LOCK()   xSemaphoreTake(g_rcl_mutex, portMAX_DELAY)
#define RCL_UNLOCK() xSemaphoreGive(g_rcl_mutex)

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif
//-----------------------------------------------------------------------------
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;

// ROS interfaces
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_sub;

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;

float linear_vel = 0.0, angular_vel = 0.0;
// Last commanded speeds
float cmd_left_speed = 0.0f;
float cmd_right_speed = 0.0f;
int32_t leftdir = 0;
int32_t rightdir = 0;
//-----------------------------------------------------------------------------
void emergency_blink(int code, bool ret = false){
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1) {
        for (volatile uint32_t t = 0; t < code; t++){
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            for (volatile uint32_t i = 0; i < 5500000; ++i) { __asm volatile("nop"); }
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            for (volatile uint32_t i = 0; i < 5500000; ++i) { __asm volatile("nop"); }
        }
        for (volatile uint32_t i = 0; i < 10500000; ++i) { __asm volatile("nop"); }
        if(ret){
            return;
        }
    }

}
//-----------------------------------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;

    // Keep it simple: no logging, no malloc, no FreeRTOS calls.
    // Optionally mask interrupts so nothing else runs after a fatal error.
    // __disable_irq();

    emergency_blink(5);
   
}
//-----------------------------------------------------------------------------
void executor_task(void *arg) {
    while (true) {
        RCL_LOCK();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        RCL_UNLOCK();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===== I2C / BNO055 functions =====
void bno055_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, BNO055_ADDR, buf, 2, false);
}
//-----------------------------------------------------------------------------
void bno055_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(I2C_PORT, BNO055_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BNO055_ADDR, buf, len, false);
}
//-----------------------------------------------------------------------------
void bno055_init() {
    uint8_t id;
    do {
        bno055_read(0x00, &id, 1);
        sleep_ms(10);
    } while (id != 0xA0);
    sleep_ms(50);
    bno055_write(0x3D, 0x0C); // NDOF mode
    sleep_ms(50);
}
//-----------------------------------------------------------------------------
// ===== Encoder Interrupts =====
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENC_A) {
        bool b = gpio_get(LEFT_ENC_B);
        left_encoder_count += (events & GPIO_IRQ_EDGE_RISE) ? (b ? -1 : 1) : 0;
    }
    if (gpio == RIGHT_ENC_A) {
        bool b = gpio_get(RIGHT_ENC_B);
        right_encoder_count += (events & GPIO_IRQ_EDGE_RISE) ? (b ? -1 : 1) : 0;
    }
}
//-----------------------------------------------------------------------------
// ===== Motor Control =====
void set_motor_pwm(uint pin, float duty) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, chan, (uint16_t)(duty * 65535));
}
//-----------------------------------------------------------------------------
void set_motor_speeds(float left, float right) {
    // clamp between -1 and 1
    if (left > 1) left = 1; if (left < -1) left = -1;
    if (right > 1) right = 1; if (right < -1) right = -1;

    if (left > 0) {
        leftdir = 1;
        set_motor_pwm(LEFT_FWD, left);
        set_motor_pwm(LEFT_REV, 0);
    } else if (left < 0) {
        leftdir = -1;
        set_motor_pwm(LEFT_FWD, 0);
        set_motor_pwm(LEFT_REV, -left);
    }else{
        set_motor_pwm(LEFT_FWD, 0);
        set_motor_pwm(LEFT_REV, 0);
    }

    if (right > 0) {
        rightdir = 1;
        set_motor_pwm(RIGHT_FWD, right);
        set_motor_pwm(RIGHT_REV, 0);
    } else if (right < 0){
        rightdir = -1;
        set_motor_pwm(RIGHT_FWD, 0);
        set_motor_pwm(RIGHT_REV, -right);
    }else{
        set_motor_pwm(RIGHT_FWD, 0);
        set_motor_pwm(RIGHT_REV, 0);
    }
}
//-----------------------------------------------------------------------------
// ===== ROS Callbacks =====
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    linear_vel = msg->linear.x;
    angular_vel = msg->angular.z;

    cmd_left_speed = linear_vel - (angular_vel * WHEEL_BASE / 2);
    cmd_right_speed = linear_vel + (angular_vel * WHEEL_BASE / 2);

    // Convert to [-1,1] for PWM control
    set_motor_speeds(cmd_left_speed, cmd_right_speed);
}
//-----------------------------------------------------------------------------
static inline void euler_to_quat_d(double roll, double pitch, double yaw,
                                   double* qx, double* qy, double* qz, double* qw)
{
    const double cr = cos(roll * 0.5), sr = sin(roll * 0.5);
    const double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    const double cy = cos(yaw * 0.5),   sy = sin(yaw * 0.5);

    *qw = cr*cp*cy + sr*sp*sy;
    *qx = sr*cp*cy - cr*sp*sy;
    *qy = cr*sp*cy + sr*cp*sy;
    *qz = cr*cp*sy - sr*sp*cy;
}
//-----------------------------------------------------------------------------
void imu_task(void *arg) {
    while (true) {
        // Skip sensor read for testing
        imu_msg.orientation.w = 1.0;
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        
        uint64_t ms = rmw_uros_epoch_millis();
        imu_msg.header.stamp.sec = (int32_t)(ms / 1000);
        imu_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000u);
        
        RCL_LOCK();
        rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
        RCL_UNLOCK();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// ===== IMU Task =====
void _imu_task(void *arg) {
    uint8_t buf[6];
    while (true) {
        bno055_read(0x1A, buf, 6);
        int16_t heading = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t roll    = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t pitch   = (int16_t)((buf[5] << 8) | buf[4]);
        
        uint64_t ms = rmw_uros_epoch_millis();
        imu_msg.header.stamp.sec     = (int32_t)(ms / 1000);
        imu_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000u);

        double r = (roll    / 16.0f) * (3.14159f / 180.0f);
        double p = (pitch   / 16.0f) * (3.14159f / 180.0f);
        double y = (heading / 16.0f) * (3.14159f / 180.0f);
        euler_to_quat_d(r, p, y,
            &imu_msg.orientation.x,
            &imu_msg.orientation.y,
            &imu_msg.orientation.z,
            &imu_msg.orientation.w);
        
        RCL_LOCK();
        rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
        RCL_UNLOCK();
        if (rc != RCL_RET_OK) {
            emergency_blink(7);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}
//-----------------------------------------------------------------------------
// ===== Odometry Task =====
void odom_task(void *arg) {
    static int32_t last_left = 0, last_right = 0;
    float x = 0, y = 0, theta = 0;
    absolute_time_t last = get_absolute_time();

    while (true) {
        int32_t l = left_encoder_count;
        int32_t r = right_encoder_count;

        int32_t dl = l - last_left;
        int32_t dr = r - last_right;
        last_left = l;
        last_right = r;

        float dist_l = (dl / (float)TICKS_PER_REV) * 2 * 3.14159f * WHEEL_RADIUS;
        float dist_r = (dr / (float)TICKS_PER_REV) * 2 * 3.14159f * WHEEL_RADIUS;
        float dist = (dist_l + dist_r) / 2.0f;
        float dtheta = (dist_r - dist_l) / WHEEL_BASE;

        x += dist * cosf(theta + dtheta / 2);
        y += dist * sinf(theta + dtheta / 2);
        theta += dtheta;

        uint64_t ms = rmw_uros_epoch_millis();
        odom_msg.header.stamp.sec     = (int32_t)(ms / 1000);
        odom_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000u);

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;

        double qx, qy, qz, qw;
        euler_to_quat_d(0.0f, 0.0f, theta, &qx, &qy, &qz, &qw);
        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;
        RCL_LOCK();
        rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);
        RCL_UNLOCK();
        if (rc != RCL_RET_OK) {
            emergency_blink(6);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
    }
    vTaskDelete(NULL);
}
//-----------------------------------------------------------------------------
void init_i2c(){
    // Init I2C for IMU
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    bno055_init();
}
//-----------------------------------------------------------------------------
void init_motors(){
    // Init PWM for motors
    gpio_set_function(LEFT_FWD, GPIO_FUNC_PWM);    gpio_set_function(LEFT_REV, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_FWD, GPIO_FUNC_PWM);   gpio_set_function(RIGHT_REV, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(LEFT_FWD), 65535);    pwm_set_enabled(pwm_gpio_to_slice_num(LEFT_FWD), true);
    pwm_set_wrap(pwm_gpio_to_slice_num(RIGHT_FWD), 65535);   pwm_set_enabled(pwm_gpio_to_slice_num(RIGHT_FWD), true);
}
//-----------------------------------------------------------------------------
void init_encoders(){
    // Init encoders
    gpio_init(LEFT_ENC_A); 
    gpio_init(RIGHT_ENC_A); 
    gpio_set_dir(LEFT_ENC_A, false);   
    gpio_set_dir(RIGHT_ENC_A, false);  
    gpio_set_irq_enabled_with_callback(LEFT_ENC_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(RIGHT_ENC_A, GPIO_IRQ_EDGE_RISE, true);
}
//-----------------------------------------------------------------------------
// ===== App Main =====
int main(void) {
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    sleep_ms(10);
    g_rcl_mutex = xSemaphoreCreateMutex();
    configASSERT(g_rcl_mutex != NULL);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    //init_i2c();
    init_motors();
    init_encoders();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    // Make sure the agent is up before proceeding
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        emergency_blink(1);
    }
    

    // micro-ROS init
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) emergency_blink(10);

    ret = rclc_node_init_default(&node, "pico", "", &support);
    if (ret != RCL_RET_OK) emergency_blink(11);

    // Publishers
    rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "pico/odom");

    rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "pico/imu");

    // Subscriber
    rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "pico/cmd_vel");


    sensor_msgs__msg__Imu__init(&imu_msg);
    nav_msgs__msg__Odometry__init(&odom_msg);

    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id,  "base_link");
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    // Odometry also commonly sets child_frame_id
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

    // Executor with 1 handle (the subscription)
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

    int s1 = configMINIMAL_STACK_SIZE;
    // Create your app tasks
    BaseType_t xRet = xTaskCreate(imu_task,  "imu",  4096,  NULL, 2, NULL);
    if( xRet != pdPASS )
    {
         emergency_blink(2);
    }

    xRet = xTaskCreate(odom_task, "odom", 4096,  NULL, 2, NULL);
    if( xRet != pdPASS )
    {
        emergency_blink(3);
    }

    // Create an executor task
    xRet = xTaskCreate(executor_task, "executor", 4096, NULL, 3, NULL);
    if( xRet != pdPASS )
    {
        emergency_blink(4);
    }
    // *** Start the scheduler ***
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    vTaskStartScheduler();
    // Should never get here
    emergency_blink(8);
}