#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>

// Definitions
#define DIST_SENSOR_COUNT 8
#define LIGHT_SENSOR_COUNT 8
#define MOTOR_MAX_SPEED 6.28
#define MOTOR_BASE_SPEED 3.0
#define WALL_DETECT_LIMIT 200
#define TRACK_WALL_THRESHOLD 150
#define AHEAD_OBSTACLE_LIMIT 150
#define MOTOR_TURN_SPEED 2.0
#define GPS_POSITION_MARGIN 0.3
#define LIGHT_INTENSITY_LIMIT 0.22

// Robot operational states
typedef enum {
    LOCATE_WALL,
    FOLLOW_WALL,
    SEARCH_LIGHT,
    TASK_COMPLETE
} BotMode;

// Devices for the robot
static WbDeviceTag distance_devices[DIST_SENSOR_COUNT];
static WbDeviceTag light_devices[LIGHT_SENSOR_COUNT];
static WbDeviceTag motor_left, motor_right;
static WbDeviceTag gps_sensor;

// Starting coordinates
static struct {
    double x, y;
} init_position;

// Brightest light info
static double brightest_light = -INFINITY;
static double bright_location[3] = {0.0, 0.0, 0.0};
static BotMode active_mode = LOCATE_WALL;
static int position_log_count = 0;
static double logged_positions[100][2]; // Tracks explored positions

// Robot initialization
void robot_initialize() {
    wb_robot_init();
    int step_duration = wb_robot_get_basic_time_step();
    
    // Distance sensors setup
    for (int i = 0; i < DIST_SENSOR_COUNT; i++) {
        char device_name[4];
        sprintf(device_name, "ps%d", i);
        distance_devices[i] = wb_robot_get_device(device_name);
        wb_distance_sensor_enable(distance_devices[i], step_duration);
    }
    
    // Light sensors setup
    for (int i = 0; i < LIGHT_SENSOR_COUNT; i++) {
        char device_name[4];
        sprintf(device_name, "ls%d", i);
        light_devices[i] = wb_robot_get_device(device_name);
        wb_light_sensor_enable(light_devices[i], step_duration);
    }
    
    // Motor setup
    motor_left = wb_robot_get_device("left wheel motor");
    motor_right = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(motor_left, INFINITY);
    wb_motor_set_position(motor_right, INFINITY);
    
    // GPS sensor setup
    gps_sensor = wb_robot_get_device("gps");
    wb_gps_enable(gps_sensor, step_duration);
}

// Store initial position
void log_start_position() {
    const double *gps_values = wb_gps_get_values(gps_sensor);
    init_position.x = gps_values[0];
    init_position.y = gps_values[1];
    printf("Initial Position: (%.2f, %.2f)\n", init_position.x, init_position.y);
}

// Update brightest light detection
void find_brightest_spot() {
    double total_light = 0.0;
    for (int i = 0; i < LIGHT_SENSOR_COUNT; i++) {
        total_light += wb_light_sensor_get_value(light_devices[i]);
    }
    total_light /= LIGHT_SENSOR_COUNT;
    
    if (total_light > brightest_light) {
        brightest_light = total_light;
        const double *gps_values = wb_gps_get_values(gps_sensor);
        memcpy(bright_location, gps_values, 3 * sizeof(double));
        printf("New Bright Spot: %.2f at (%.2f, %.2f)\n", brightest_light, bright_location[0], bright_location[1]);
    }
}

// Check if at brightest spot
int at_bright_spot() {
    const double *gps_values = wb_gps_get_values(gps_sensor);
    double dx = gps_values[0] - bright_location[0];
    double dy = gps_values[1] - bright_location[1];
    return sqrt(dx * dx + dy * dy) < LIGHT_INTENSITY_LIMIT;
}

// Gather sensor data
void collect_sensor_readings(double *distance_values) {
    for (int i = 0; i < DIST_SENSOR_COUNT; i++) {
        distance_values[i] = wb_distance_sensor_get_value(distance_devices[i]);
    }
}
// Adjust motor speed
void adjust_motor_speeds(double left, double right) {
    left = fmax(-MOTOR_MAX_SPEED, fmin(MOTOR_MAX_SPEED, left));
    right = fmax(-MOTOR_MAX_SPEED, fmin(MOTOR_MAX_SPEED, right));
    
    wb_motor_set_velocity(motor_left, left);
    wb_motor_set_velocity(motor_right, right);
}
// Follow walls using the left-hand rule
void handle_wall_following(double *distances, double *left_motor_speed, double *right_motor_speed) {
    double left_front = distances[5];   // Front-left sensor
    double front_center = distances[0]; // Front sensor
    double left_side = distances[6];    // Left-side sensor
    
    if (front_center > AHEAD_OBSTACLE_LIMIT || left_front > AHEAD_OBSTACLE_LIMIT) {
        // Obstacle ahead, turn right
        *left_motor_speed = MOTOR_TURN_SPEED;
        *right_motor_speed = -MOTOR_TURN_SPEED;
        return;
    }
    
    if (left_side < TRACK_WALL_THRESHOLD) {
        // No wall on the left, turn left
        *left_motor_speed = MOTOR_BASE_SPEED * 0.5;
        *right_motor_speed = MOTOR_BASE_SPEED;
    } else {
        // Wall on the left, move forward
        double correction = (TRACK_WALL_THRESHOLD - left_side) * 0.01;
        *left_motor_speed = MOTOR_BASE_SPEED - correction;
        *right_motor_speed = MOTOR_BASE_SPEED + correction;
    }
}
// Track visited locations
int check_visited(const double *current_pos) {
    for (int i = 0; i < position_log_count; i++) {
        double dx = current_pos[0] - logged_positions[i][0];
        double dy = current_pos[1] - logged_positions[i][1];
        if (sqrt(dx * dx + dy * dy) < GPS_POSITION_MARGIN) {
            return 1;
        }
    }
    logged_positions[position_log_count][0] = current_pos[0];
    logged_positions[position_log_count][1] = current_pos[1];
    position_log_count++;
    return 0;
}

int main() {
    robot_initialize();
    int step_duration = wb_robot_get_basic_time_step();
    log_start_position();
    
    printf("Exploration Started...\n");
    
    while (wb_robot_step(step_duration) != -1) {
        double left_motor_speed = 0, right_motor_speed = 0;
        double distances[DIST_SENSOR_COUNT];
        collect_sensor_readings(distances);
        const double *gps_values = wb_gps_get_values(gps_sensor);
        
        switch (active_mode) {
            case LOCATE_WALL:
            case FOLLOW_WALL:
                handle_wall_following(distances, &left_motor_speed, &right_motor_speed);
                find_brightest_spot();
                if (!check_visited(gps_values)) {
                    printf("Exploring New Area at (%.2f, %.2f)\n", gps_values[0], gps_values[1]);
                }
                if (position_log_count > 27) {
                    printf("Map Traversal Done. Searching Brightest Spot...\n");
                    active_mode = SEARCH_LIGHT;
                }
                break;
            
            case SEARCH_LIGHT:
                if (at_bright_spot()) {
                    printf("Reached Bright Spot at (%.2f, %.2f)! Task Complete.\n", gps_values[0], gps_values[1]);
                    active_mode = TASK_COMPLETE;
                } else {
                    handle_wall_following(distances, &left_motor_speed, &right_motor_speed);
                }
                break;
            
            case TASK_COMPLETE:
                left_motor_speed = 0;
                right_motor_speed = 0;
                break;
        }
        
        adjust_motor_speeds(left_motor_speed, right_motor_speed);
    }
    
    wb_robot_cleanup();
    return 0;
}
