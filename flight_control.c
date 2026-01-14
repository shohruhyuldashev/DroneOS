// flight_control.c - DroneOS Flight Controller
// PID controller, stabilization, navigation

// ============================================================================
// MATEMATIK KUTUBXONA
// ============================================================================

float sqrt_fast(float x) {
    // Newton-Raphson method
    if (x <= 0) return 0;
    float guess = x;
    for (int i = 0; i < 10; i++) {
        guess = (guess + x / guess) / 2.0f;
    }
    return guess;
}

float atan2_approx(float y, float x) {
    // Fast atan2 approximation
    float abs_y = y < 0 ? -y : y;
    float r, angle;
    
    if (x >= 0) {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.7853981634f - 0.7853981634f * r;  // pi/4
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 2.356194490f - 0.7853981634f * r;  // 3*pi/4
    }
    
    return y < 0 ? -angle : angle;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integrator;   // Integral sum
    float prev_error;   // Previous error
    float max_integrator;
    float max_output;
} pid_controller_t;

void pid_init(pid_controller_t* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0;
    pid->prev_error = 0;
    pid->max_integrator = 100.0f;
    pid->max_output = 500.0f;
}

float pid_update(pid_controller_t* pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    
    // Proportional
    float p_term = pid->kp * error;
    
    // Integral (with anti-windup)
    pid->integrator += error * dt;
    if (pid->integrator > pid->max_integrator) pid->integrator = pid->max_integrator;
    if (pid->integrator < -pid->max_integrator) pid->integrator = -pid->max_integrator;
    float i_term = pid->ki * pid->integrator;
    
    // Derivative
    float d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Output
    float output = p_term + i_term + d_term;
    
    // Limit output
    if (output > pid->max_output) output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;
    
    return output;
}

// ============================================================================
// ATTITUDE ESTIMATION (Complementary Filter)
// ============================================================================

typedef struct {
    float roll;   // degrees
    float pitch;  // degrees
    float yaw;    // degrees
} attitude_t;

typedef struct {
    float roll_rate;   // deg/s
    float pitch_rate;  // deg/s
    float yaw_rate;    // deg/s
} rates_t;

static attitude_t current_attitude = {0, 0, 0};
static rates_t current_rates = {0, 0, 0};

void attitude_update(imu_data_t* imu, float dt) {
    // Gyroscope data ni deg/s ga konvert (MPU6050: 65.5 LSB/(deg/s) @ ±500)
    float gyro_x = imu->gyro_x / 65.5f;
    float gyro_y = imu->gyro_y / 65.5f;
    float gyro_z = imu->gyro_z / 65.5f;
    
    current_rates.roll_rate = gyro_x;
    current_rates.pitch_rate = gyro_y;
    current_rates.yaw_rate = gyro_z;
    
    // Accelerometer dan burchak hisoblash
    float accel_x = imu->accel_x / 8192.0f;  // ±4g
    float accel_y = imu->accel_y / 8192.0f;
    float accel_z = imu->accel_z / 8192.0f;
    
    float accel_roll = atan2_approx(accel_y, accel_z) * 57.2958f;  // rad to deg
    float accel_pitch = atan2_approx(-accel_x, sqrt_fast(accel_y*accel_y + accel_z*accel_z)) * 57.2958f;
    
    // Complementary filter (98% gyro, 2% accel)
    current_attitude.roll = 0.98f * (current_attitude.roll + gyro_x * dt) + 0.02f * accel_roll;
    current_attitude.pitch = 0.98f * (current_attitude.pitch + gyro_y * dt) + 0.02f * accel_pitch;
    current_attitude.yaw += gyro_z * dt;  // Yaw faqat gyrodan (magnetometer kerak)
}

// ============================================================================
// FLIGHT MODES
// ============================================================================

typedef enum {
    MODE_DISARMED,
    MODE_STABILIZE,  // Angle mode
    MODE_ACRO,       // Rate mode
    MODE_ALTITUDE_HOLD,
    MODE_GPS_HOLD
} flight_mode_t;

static flight_mode_t current_mode = MODE_DISARMED;

// ============================================================================
// RC INPUT
// ============================================================================

typedef struct {
    int throttle;  // 0-1000
    int roll;      // -500 to +500
    int pitch;     // -500 to +500
    int yaw;       // -500 to +500
    int aux1;      // Mode switch
    int aux2;      // Arm switch
} rc_input_t;

static rc_input_t rc = {0, 0, 0, 0, 0, 0};

// ============================================================================
// MIXER (Motor mixing for quadcopter X configuration)
// ============================================================================

void mixer_update(int throttle, float roll, float pitch, float yaw) {
    // X configuration:
    //   M0(FR)  M1(BL)
    //       \ /
    //        X
    //       / \
    //   M2(FL)  M3(BR)
    
    int motor[4];
    
    // Mix commands
    motor[0] = throttle + roll - pitch + yaw;  // Front Right
    motor[1] = throttle - roll + pitch + yaw;  // Back Left
    motor[2] = throttle - roll - pitch - yaw;  // Front Left
    motor[3] = throttle + roll + pitch - yaw;  // Back Right
    
    // Apply to motors
    for (int i = 0; i < 4; i++) {
        if (motor[i] < 0) motor[i] = 0;
        if (motor[i] > 1000) motor[i] = 1000;
        motor_set(i, motor[i]);
    }
}

// ============================================================================
// FLIGHT CONTROLLER MAIN LOOP
// ============================================================================

static pid_controller_t pid_roll_rate, pid_pitch_rate, pid_yaw_rate;
static pid_controller_t pid_roll_angle, pid_pitch_angle;

void flight_controller_init(void) {
    // Rate PIDs (inner loop)
    pid_init(&pid_roll_rate, 0.5f, 0.05f, 0.01f);
    pid_init(&pid_pitch_rate, 0.5f, 0.05f, 0.01f);
    pid_init(&pid_yaw_rate, 0.8f, 0.02f, 0.0f);
    
    // Angle PIDs (outer loop)
    pid_init(&pid_roll_angle, 4.0f, 0.0f, 0.0f);
    pid_init(&pid_pitch_angle, 4.0f, 0.0f, 0.0f);
    
    motors_init();
}

void flight_controller_update(float dt) {
    // IMU o'qish
    imu_data_t imu;
    mpu6050_read(&imu);
    
    // Attitude yangilash
    attitude_update(&imu, dt);
    
    float roll_output = 0, pitch_output = 0, yaw_output = 0;
    
    if (current_mode == MODE_STABILIZE) {
        // Angle mode: outer loop + inner loop
        
        // RC inputdan desired angle
        float desired_roll = rc.roll * 0.06f;   // ±30 degrees
        float desired_pitch = rc.pitch * 0.06f;
        
        // Outer loop (angle to rate)
        float roll_rate_sp = pid_update(&pid_roll_angle, desired_roll, current_attitude.roll, dt);
        float pitch_rate_sp = pid_update(&pid_pitch_angle, desired_pitch, current_attitude.pitch, dt);
        
        // Inner loop (rate control)
        roll_output = pid_update(&pid_roll_rate, roll_rate_sp, current_rates.roll_rate, dt);
        pitch_output = pid_update(&pid_pitch_rate, pitch_rate_sp, current_rates.pitch_rate, dt);
        yaw_output = pid_update(&pid_yaw_rate, rc.yaw * 0.5f, current_rates.yaw_rate, dt);
        
    } else if (current_mode == MODE_ACRO) {
        // Rate mode: faqat inner loop
        
        float desired_roll_rate = rc.roll * 1.0f;   // ±500 deg/s
        float desired_pitch_rate = rc.pitch * 1.0f;
        float desired_yaw_rate = rc.yaw * 0.5f;
        
        roll_output = pid_update(&pid_roll_rate, desired_roll_rate, current_rates.roll_rate, dt);
        pitch_output = pid_update(&pid_pitch_rate, desired_pitch_rate, current_rates.pitch_rate, dt);
        yaw_output = pid_update(&pid_yaw_rate, desired_yaw_rate, current_rates.yaw_rate, dt);
    }
    
    // Mixer ga yuborish
    if (current_mode != MODE_DISARMED) {
        mixer_update(rc.throttle, roll_output, pitch_output, yaw_output);
    } else {
        motors_disarm();
    }
}

// ============================================================================
// SAFETY CHECKS
// ============================================================================

int safety_check(void) {
    // Angle too extreme?
    if (current_attitude.roll > 60.0f || current_attitude.roll < -60.0f) return 0;
    if (current_attitude.pitch > 60.0f || current_attitude.pitch < -60.0f) return 0;
    
    // RC signal lost?
    // TODO: implement failsafe
    
    return 1;
}

void failsafe_activate(void) {
    current_mode = MODE_DISARMED;
    motors_disarm();
    // TODO: log event, trigger buzzer
}

// ============================================================================
// TELEMETRY
// ============================================================================

typedef struct {
    float battery_voltage;
    attitude_t attitude;
    rates_t rates;
    int motor_values[4];
    flight_mode_t mode;
    unsigned int flight_time_ms;
} telemetry_t;

void telemetry_send(telemetry_t* telem) {
    // Send via UART in custom protocol
    uart_puts("TELEM:");
    
    // Simple ASCII format for debugging
    uart_puts(" Roll=");
    // uart_put_float(telem->attitude.roll);
    uart_puts(" Pitch=");
    // uart_put_float(telem->attitude.pitch);
    uart_puts("\n");
    
    // Binary mavlink protocol implementation would go here
}