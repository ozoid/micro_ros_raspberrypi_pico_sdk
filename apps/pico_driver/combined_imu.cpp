#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "time_override.c"
#include "task.h"
#include <math.h>


#define I2C_PORT i2c1
#define SDA_PIN  14
#define SCL_PIN  15
#define BNO055_ADDR 0x28

// DRV8833 pins
#define LEFT_FWD 10
#define LEFT_REV 11
#define RIGHT_FWD 12
#define RIGHT_REV 13

// Encoder pins
#define LEFT_ENC_A 18
//#define LEFT_ENC_B 19
#define RIGHT_ENC_A 20
//#define RIGHT_ENC_B 21

// Robot params
#define WHEEL_RADIUS 0.045f  // 9 cm
#define WHEEL_BASE   0.09f  // 9 cm centre of front axle to centre of back axle
#define TICKS_PER_REV 80
//-----------------------------------------------------------------------------
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;

// ROS interfaces
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t cmd_vel_sub;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
float linear_vel = 0.0, angular_vel = 0.0;
// Last commanded speeds
float cmd_left_speed = 0.0f;
float cmd_right_speed = 0.0f;
int32_t leftdir = 0;
int32_t rightdir = 0;
//-----------------------------------------------------------------------------
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
        left_encoder_count += leftdir;
    }
    if (gpio == RIGHT_ENC_A) {
        right_encoder_count += rightdir;
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
        rightdir = 1
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
// ===== IMU Task =====
void imu_task(void *arg) {
    uint8_t buf[6];
    while (true) {
        bno055_read(0x1A, buf, 6);
        int16_t heading = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t roll    = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t pitch   = (int16_t)((buf[5] << 8) | buf[4]);
        
        imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        imu_msg.header.frame_id.data = "base_link";

        imu_msg.orientation.x = roll / 16.0f * (3.14159f / 180.0f);
        imu_msg.orientation.y = pitch / 16.0f * (3.14159f / 180.0f);
        imu_msg.orientation.z = heading / 16.0f * (3.14159f / 180.0f);
        imu_msg.orientation.w = 0.0f;

        rcl_publish(&imu_pub, &imu_msg, NULL);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
//-----------------------------------------------------------------------------
// ===== Odometry Task =====
void odom_task(void *arg) {
    static int32_t last_left = 0, last_right = 0;
    float x = 0, y = 0, theta = 0;

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

        odom_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        odom_msg.header.frame_id.data = "odom";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation.z = theta;

        rcl_publish(&odom_pub, &odom_msg, NULL);
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
    }
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
int main(void *arg) {
    stdio_init_all();
    init_i2c();
    init_motors();
    init_encoders();

    // micro-ROS setup
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico", "", &support);

    // Publishers
    rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "pico/odom"
    );
    rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "pico/imu"
    );

    // Subscriber
    rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "pico/cmd_vel"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

    // Create tasks
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 1, NULL);
    xTaskCreate(odom_task, "odom_task", 4096, NULL, 1, NULL);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
