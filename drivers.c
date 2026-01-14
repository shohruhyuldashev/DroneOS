// drivers.c - DroneOS Hardware Drivers
// I2C, SPI, PWM, GPIO drayverlari

// ============================================================================
// I2C DRIVER (IMU uchun - MPU6050/MPU9250)
// ============================================================================

#define I2C_BASE (PERIPHERAL_BASE + 0x804000)  // BSC1
#define I2C_C    ((volatile unsigned int*)(I2C_BASE + 0x00))
#define I2C_S    ((volatile unsigned int*)(I2C_BASE + 0x04))
#define I2C_DLEN ((volatile unsigned int*)(I2C_BASE + 0x08))
#define I2C_A    ((volatile unsigned int*)(I2C_BASE + 0x0C))
#define I2C_FIFO ((volatile unsigned int*)(I2C_BASE + 0x10))
#define I2C_DIV  ((volatile unsigned int*)(I2C_BASE + 0x14))

void i2c_init(void) {
    // GPIO 2,3 ni I2C ga ulash
    unsigned int sel = *GPFSEL0;
    sel &= ~(7 << 6);   // GPIO 2
    sel |= 4 << 6;      // ALT0
    sel &= ~(7 << 9);   // GPIO 3
    sel |= 4 << 9;      // ALT0
    *GPFSEL0 = sel;
    
    // 400kHz clock
    *I2C_DIV = 2500;
}

int i2c_write(unsigned char addr, unsigned char reg, unsigned char data) {
    *I2C_A = addr;
    *I2C_DLEN = 2;
    *I2C_C = 0x8080;  // I2C enable, start
    
    while (!(*I2C_S & 0x10));  // Wait for TXD
    *I2C_FIFO = reg;
    while (!(*I2C_S & 0x10));
    *I2C_FIFO = data;
    
    while (!(*I2C_S & 0x02));  // Wait for DONE
    *I2C_S = 0x02;
    
    return 0;
}

int i2c_read(unsigned char addr, unsigned char reg, unsigned char* data, int len) {
    // Write register address
    *I2C_A = addr;
    *I2C_DLEN = 1;
    *I2C_C = 0x8080;
    while (!(*I2C_S & 0x10));
    *I2C_FIFO = reg;
    while (!(*I2C_S & 0x02));
    *I2C_S = 0x02;
    
    // Read data
    *I2C_DLEN = len;
    *I2C_C = 0x8081;  // Read mode
    
    for (int i = 0; i < len; i++) {
        while (!(*I2C_S & 0x20));  // Wait for RXD
        data[i] = *I2C_FIFO & 0xFF;
    }
    
    while (!(*I2C_S & 0x02));
    *I2C_S = 0x02;
    
    return len;
}

// ============================================================================
// MPU6050 IMU DRIVER
// ============================================================================

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

typedef struct {
    short accel_x, accel_y, accel_z;
    short temp;
    short gyro_x, gyro_y, gyro_z;
} imu_data_t;

void mpu6050_init(void) {
    // Wake up MPU6050
    i2c_write(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    delay_ms(100);
    
    // Gyro: ±500 deg/s
    i2c_write(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x08);
    
    // Accel: ±4g
    i2c_write(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x08);
}

void mpu6050_read(imu_data_t* data) {
    unsigned char buffer[14];
    i2c_read(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14);
    
    data->accel_x = (buffer[0] << 8) | buffer[1];
    data->accel_y = (buffer[2] << 8) | buffer[3];
    data->accel_z = (buffer[4] << 8) | buffer[5];
    data->temp = (buffer[6] << 8) | buffer[7];
    data->gyro_x = (buffer[8] << 8) | buffer[9];
    data->gyro_y = (buffer[10] << 8) | buffer[11];
    data->gyro_z = (buffer[12] << 8) | buffer[13];
}

// ============================================================================
// PWM DRIVER (Motor control)
// ============================================================================

#define PWM_BASE (PERIPHERAL_BASE + 0x20C000)
#define PWM_CTL  ((volatile unsigned int*)(PWM_BASE + 0x00))
#define PWM_STA  ((volatile unsigned int*)(PWM_BASE + 0x04))
#define PWM_RNG1 ((volatile unsigned int*)(PWM_BASE + 0x10))
#define PWM_DAT1 ((volatile unsigned int*)(PWM_BASE + 0x14))
#define PWM_RNG2 ((volatile unsigned int*)(PWM_BASE + 0x20))
#define PWM_DAT2 ((volatile unsigned int*)(PWM_BASE + 0x24))

#define CLOCK_BASE (PERIPHERAL_BASE + 0x101000)
#define PWMCLK_CNTL ((volatile unsigned int*)(CLOCK_BASE + 0xA0))
#define PWMCLK_DIV  ((volatile unsigned int*)(CLOCK_BASE + 0xA4))

void pwm_init(void) {
    // GPIO 12, 13 ni PWM ga ulash
    unsigned int sel = *((volatile unsigned int*)(GPIO_BASE + 0x04));
    sel &= ~(7 << 6);   // GPIO 12
    sel |= 4 << 6;      // ALT0
    sel &= ~(7 << 9);   // GPIO 13
    sel |= 4 << 9;      // ALT0
    *((volatile unsigned int*)(GPIO_BASE + 0x04)) = sel;
    
    // PWM clock: 19.2MHz / 192 = 100kHz
    *PWMCLK_CNTL = 0x5A000006;  // Stop clock
    delay_us(10);
    *PWMCLK_DIV = 0x5A000000 | (192 << 12);
    *PWMCLK_CNTL = 0x5A000011;  // Start clock, PLLD source
    
    // PWM setup: 50Hz (20ms period) for ESC
    *PWM_RNG1 = 2000;  // 100kHz / 2000 = 50Hz
    *PWM_RNG2 = 2000;
    
    *PWM_CTL = 0x81;   // Enable PWM1, use FIFO
}

void pwm_set_duty(int channel, int duty) {
    // duty: 0-2000 (corresponds to pulse width)
    // For ESC: 1000 = 1ms (min), 2000 = 2ms (max)
    if (channel == 1) {
        *PWM_DAT1 = duty;
    } else {
        *PWM_DAT2 = duty;
    }
}

// ============================================================================
// GPIO DRIVER (General Purpose)
// ============================================================================

void gpio_set_output(int pin) {
    int reg = pin / 10;
    int shift = (pin % 10) * 3;
    unsigned int* fsel = (unsigned int*)(GPIO_BASE + reg * 4);
    
    unsigned int val = *fsel;
    val &= ~(7 << shift);
    val |= (1 << shift);  // Output
    *fsel = val;
}

void gpio_set_input(int pin) {
    int reg = pin / 10;
    int shift = (pin % 10) * 3;
    unsigned int* fsel = (unsigned int*)(GPIO_BASE + reg * 4);
    
    unsigned int val = *fsel;
    val &= ~(7 << shift);  // Input
    *fsel = val;
}

void gpio_write(int pin, int value) {
    if (value) {
        *GPSET0 = (1 << pin);
    } else {
        *GPCLR0 = (1 << pin);
    }
}

int gpio_read(int pin) {
    unsigned int* gplev = (unsigned int*)(GPIO_BASE + 0x34);
    return (*gplev & (1 << pin)) ? 1 : 0;
}

// ============================================================================
// SPI DRIVER (Additional sensors)
// ============================================================================

#define SPI0_BASE (PERIPHERAL_BASE + 0x204000)
#define SPI0_CS   ((volatile unsigned int*)(SPI0_BASE + 0x00))
#define SPI0_FIFO ((volatile unsigned int*)(SPI0_BASE + 0x04))
#define SPI0_CLK  ((volatile unsigned int*)(SPI0_BASE + 0x08))

void spi_init(void) {
    // GPIO 7-11 ni SPI ga ulash
    // CS=8, MISO=9, MOSI=10, SCLK=11
    
    // Clock: 250MHz / 256 = ~1MHz
    *SPI0_CLK = 256;
}

unsigned char spi_transfer(unsigned char data) {
    *SPI0_FIFO = data;
    while (!(*SPI0_CS & 0x10000));  // Wait for DONE
    return *SPI0_FIFO & 0xFF;
}

// ============================================================================
// MOTOR CONTROLLER (4 motors for quadcopter)
// ============================================================================

#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_IDLE 1100

static int motor_values[4] = {MOTOR_IDLE, MOTOR_IDLE, MOTOR_IDLE, MOTOR_IDLE};

void motors_init(void) {
    pwm_init();
    
    // Additional 2 motors via GPIO bit-banging PWM
    gpio_set_output(18);  // Motor 3
    gpio_set_output(19);  // Motor 4
}

void motor_set(int motor, int throttle) {
    // throttle: 0-1000
    int value = MOTOR_MIN + throttle;
    if (value > MOTOR_MAX) value = MOTOR_MAX;
    if (value < MOTOR_MIN) value = MOTOR_MIN;
    
    motor_values[motor] = value;
    
    if (motor == 0) {
        pwm_set_duty(1, value);
    } else if (motor == 1) {
        pwm_set_duty(2, value);
    }
    // Motors 2,3: software PWM (simplified, real impl needs timer)
}

void motors_arm(void) {
    for (int i = 0; i < 4; i++) {
        motor_set(i, 0);
    }
    delay_ms(2000);
}

void motors_disarm(void) {
    for (int i = 0; i < 4; i++) {
        motor_set(i, 0);
    }
}