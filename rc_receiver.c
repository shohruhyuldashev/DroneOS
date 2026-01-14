// rc_receiver.c - DroneOS Radio Control Receiver
// PPM, SBUS, va WiFi/Bluetooth masofadan boshqaruv

// ============================================================================
// PPM (Pulse Position Modulation) DECODER
// ============================================================================

#define RC_CHANNELS 8
#define PPM_PIN 17
#define PPM_SYNC_MIN 4000   // 4ms sync pulse
#define PPM_PULSE_MIN 900   // 0.9ms
#define PPM_PULSE_MAX 2100  // 2.1ms

static volatile unsigned int ppm_values[RC_CHANNELS];
static volatile int ppm_channel = 0;
static volatile unsigned int last_pulse_time = 0;
static volatile int rc_signal_valid = 0;
static volatile unsigned int last_update_time = 0;

// PPM Interrupt Handler (GPIO falling edge)
void ppm_interrupt_handler(void) {
    unsigned int now = get_time_us();
    unsigned int pulse_width = now - last_pulse_time;
    last_pulse_time = now;
    
    if (pulse_width > PPM_SYNC_MIN) {
        // Sync pulse - yangi frame
        ppm_channel = 0;
        rc_signal_valid = 1;
        last_update_time = now;
    } else if (ppm_channel < RC_CHANNELS) {
        // Channel pulse
        if (pulse_width >= PPM_PULSE_MIN && pulse_width <= PPM_PULSE_MAX) {
            ppm_values[ppm_channel] = pulse_width;
            ppm_channel++;
        }
    }
}

void ppm_init(void) {
    // GPIO ni input qilish
    gpio_set_input(PPM_PIN);
    
    // Falling edge interrupt
    // RPi GPIO interrupt setup
    unsigned int* gpren = (unsigned int*)(GPIO_BASE + 0x4C);  // Falling edge
    *gpren |= (1 << PPM_PIN);
    
    uart_puts("[RC] PPM decoder initialized on GPIO 17\n");
}

// PPM dan RC qiymatlarini olish (900-2100 -> -500 to +500)
void ppm_get_values(rc_input_t* rc) {
    disable_irq();
    
    // Channel mapping (AETR - Aileron, Elevator, Throttle, Rudder)
    unsigned int roll_raw = ppm_values[0];      // Aileron
    unsigned int pitch_raw = ppm_values[1];     // Elevator
    unsigned int throttle_raw = ppm_values[2];  // Throttle
    unsigned int yaw_raw = ppm_values[3];       // Rudder
    unsigned int aux1_raw = ppm_values[4];      // Mode switch
    unsigned int aux2_raw = ppm_values[5];      // Arm switch
    
    enable_irq();
    
    // Konvert: 900-2100 -> -500 to +500 (roll/pitch/yaw)
    rc->roll = (int)roll_raw - 1500;
    rc->pitch = (int)pitch_raw - 1500;
    rc->yaw = (int)yaw_raw - 1500;
    
    // Throttle: 900-2100 -> 0-1000
    rc->throttle = (int)throttle_raw - 1000;
    if (rc->throttle < 0) rc->throttle = 0;
    if (rc->throttle > 1000) rc->throttle = 1000;
    
    // AUX channels
    rc->aux1 = (int)aux1_raw;
    rc->aux2 = (int)aux2_raw;
}

// ============================================================================
// SBUS DECODER (Futaba/FrSky)
// ============================================================================

#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

static unsigned char sbus_buffer[SBUS_FRAME_SIZE];
static int sbus_index = 0;
static unsigned int sbus_channels[16];

void sbus_init(void) {
    // UART setup for SBUS (100k baud, 8E2)
    // RPi UART1 (mini UART)
    volatile unsigned int* aux_enables = (unsigned int*)(PERIPHERAL_BASE + 0x215004);
    *aux_enables |= 1;  // Enable mini UART
    
    volatile unsigned int* mu_baud = (unsigned int*)(PERIPHERAL_BASE + 0x215068);
    *mu_baud = 270;  // 100000 baud (system clock 250MHz)
    
    uart_puts("[RC] SBUS decoder initialized\n");
}

void sbus_parse_byte(unsigned char byte) {
    if (sbus_index == 0 && byte == SBUS_HEADER) {
        sbus_buffer[sbus_index++] = byte;
    } else if (sbus_index > 0 && sbus_index < SBUS_FRAME_SIZE) {
        sbus_buffer[sbus_index++] = byte;
        
        if (sbus_index == SBUS_FRAME_SIZE && sbus_buffer[24] == SBUS_FOOTER) {
            // Valid frame - decode 16 channels (11-bit each)
            sbus_channels[0]  = ((sbus_buffer[1]    | sbus_buffer[2]<<8)                 & 0x07FF);
            sbus_channels[1]  = ((sbus_buffer[2]>>3 | sbus_buffer[3]<<5)                 & 0x07FF);
            sbus_channels[2]  = ((sbus_buffer[3]>>6 | sbus_buffer[4]<<2 | sbus_buffer[5]<<10) & 0x07FF);
            sbus_channels[3]  = ((sbus_buffer[5]>>1 | sbus_buffer[6]<<7)                 & 0x07FF);
            // ... 16 channelgacha
            
            rc_signal_valid = 1;
            last_update_time = get_time_us();
        }
        
        if (sbus_index >= SBUS_FRAME_SIZE) sbus_index = 0;
    } else {
        sbus_index = 0;  // Resync
    }
}

void sbus_get_values(rc_input_t* rc) {
    // SBUS: 172-1811 range -> -500 to +500
    rc->roll = ((int)sbus_channels[0] - 992) / 2;
    rc->pitch = ((int)sbus_channels[1] - 992) / 2;
    rc->throttle = ((int)sbus_channels[2] - 172) * 1000 / 1639;
    rc->yaw = ((int)sbus_channels[3] - 992) / 2;
    rc->aux1 = sbus_channels[4];
    rc->aux2 = sbus_channels[5];
}

// ============================================================================
// WiFi CONTROL (ESP8266/ESP32 module orqali)
// ============================================================================

#define WIFI_UART UART0_BASE
#define WIFI_BUFFER_SIZE 128

static char wifi_rx_buffer[WIFI_BUFFER_SIZE];
static int wifi_rx_index = 0;

void wifi_init(void) {
    // ESP8266/ESP32 modem AT komanda orqali setup
    uart_puts("[RC] Initializing WiFi control...\n");
    
    // ESP8266 reset
    gpio_set_output(23);  // ESP8266 RST pin
    gpio_write(23, 0);
    delay_ms(100);
    gpio_write(23, 1);
    delay_ms(1000);
    
    // AT commands
    uart_puts("AT\r\n");
    delay_ms(100);
    
    // WiFi Access Point mode
    uart_puts("AT+CWMODE=2\r\n");  // AP mode
    delay_ms(100);
    
    // AP sozlamalari
    uart_puts("AT+CWSAP=\"DroneOS_AP\",\"12345678\",5,3\r\n");
    delay_ms(500);
    
    // TCP server (port 5760 - mavlink standart)
    uart_puts("AT+CIPMUX=1\r\n");
    delay_ms(100);
    uart_puts("AT+CIPSERVER=1,5760\r\n");
    delay_ms(100);
    
    uart_puts("[RC] WiFi AP: DroneOS_AP, Password: 12345678\n");
    uart_puts("[RC] TCP Server: 192.168.4.1:5760\n");
}

// WiFi dan RC komandalarini qabul qilish
void wifi_parse_command(char* cmd) {
    // Custom protocol yoki MAVLink
    // Format: "RC:throttle,roll,pitch,yaw,aux1,aux2\n"
    
    if (cmd[0] == 'R' && cmd[1] == 'C' && cmd[2] == ':') {
        int values[6];
        // Simple parsing
        // sscanf(cmd + 3, "%d,%d,%d,%d,%d,%d", 
        //        &values[0], &values[1], &values[2], 
        //        &values[3], &values[4], &values[5]);
        
        extern rc_input_t rc;
        rc.throttle = values[0];
        rc.roll = values[1];
        rc.pitch = values[2];
        rc.yaw = values[3];
        rc.aux1 = values[4];
        rc.aux2 = values[5];
        
        rc_signal_valid = 1;
        last_update_time = get_time_us();
    }
}

// ============================================================================
// BLUETOOTH CONTROL (HC-05/HC-06 yoki built-in Bluetooth)
// ============================================================================

void bluetooth_init(void) {
    uart_puts("[RC] Initializing Bluetooth...\n");
    
    // RPi Zero 2W built-in Bluetooth (BCM43438)
    // HCI setup orqali
    
    // HC-05 module ishlatilsa (UART orqali):
    // AT+NAME=DroneOS
    // AT+PSWD=1234
    // AT+UART=115200,0,0
    
    uart_puts("[RC] Bluetooth: DroneOS (PIN: 1234)\n");
}

// ============================================================================
// MAVLINK PROTOCOL (Ground Station uchun)
// ============================================================================

typedef struct __attribute__((packed)) {
    unsigned char stx;          // 0xFE
    unsigned char len;
    unsigned char seq;
    unsigned char sysid;
    unsigned char compid;
    unsigned char msgid;
    unsigned char payload[255];
    unsigned short checksum;
} mavlink_message_t;

void mavlink_send_heartbeat(void) {
    mavlink_message_t msg;
    msg.stx = 0xFE;
    msg.len = 9;
    msg.seq = 0;
    msg.sysid = 1;
    msg.compid = 1;
    msg.msgid = 0;  // HEARTBEAT
    
    // Payload: type, autopilot, base_mode, custom_mode, system_status
    msg.payload[0] = 2;  // MAV_TYPE_QUADROTOR
    msg.payload[1] = 12; // MAV_AUTOPILOT_GENERIC
    msg.payload[2] = 0;  // base_mode
    msg.payload[3] = 0;
    msg.payload[4] = 0;
    msg.payload[5] = 0;
    msg.payload[6] = 4;  // MAV_STATE_ACTIVE
    
    // Send via UART/WiFi
    // uart_send_bytes((unsigned char*)&msg, msg.len + 8);
}

void mavlink_send_attitude(attitude_t* att) {
    mavlink_message_t msg;
    msg.stx = 0xFE;
    msg.len = 28;
    msg.msgid = 30;  // ATTITUDE
    
    // Payload: time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
    // Float packing...
    
    uart_puts("[TELEM] Sending attitude via MAVLink\n");
}

// ============================================================================
// JOYSTICK CONTROL (Smartphone App)
// ============================================================================

// Android/iOS app orqali WiFi/Bluetooth
// JSON format:
// {"throttle": 500, "roll": -100, "pitch": 50, "yaw": 0}

void parse_json_command(char* json) {
    // Simple JSON parser
    // Production uchun cJSON library ishlatiladi
    
    extern rc_input_t rc;
    
    // Throttle topish
    char* ptr = strstr(json, "\"throttle\":");
    if (ptr) {
        // rc.throttle = atoi(ptr + 11);
    }
    
    // Roll, pitch, yaw xuddi shunday...
    
    rc_signal_valid = 1;
    last_update_time = get_time_us();
}

// ============================================================================
// FAILSAFE
// ============================================================================

int rc_signal_lost(void) {
    unsigned int now = get_time_us();
    // 100ms dan ko'p signal yo'q bo'lsa
    if (now - last_update_time > 100000) {
        return 1;
    }
    return 0;
}

void rc_failsafe_handler(void) {
    if (rc_signal_lost()) {
        uart_puts("[RC] FAILSAFE: Signal lost!\n");
        
        // Emergency action
        extern flight_mode_t current_mode;
        extern rc_input_t rc;
        
        // Throttle pasaytirish
        rc.throttle = rc.throttle * 0.9f;
        
        // 2 sekunddan keyin landing
        static unsigned int failsafe_start = 0;
        if (failsafe_start == 0) {
            failsafe_start = get_time_us();
        }
        
        if (get_time_us() - failsafe_start > 2000000) {
            // Land mode
            current_mode = MODE_DISARMED;
            motors_disarm();
            uart_puts("[RC] FAILSAFE: Motors disarmed\n");
        }
    }
}

// ============================================================================
// RC TASK (Main receiver task)
// ============================================================================

typedef enum {
    RC_MODE_PPM,
    RC_MODE_SBUS,
    RC_MODE_WIFI,
    RC_MODE_BLUETOOTH
} rc_mode_t;

static rc_mode_t active_rc_mode = RC_MODE_WIFI;  // Default WiFi

void rc_receiver_init(void) {
    uart_puts("[RC] Initializing receiver...\n");
    
    // PPM init
    ppm_init();
    
    // WiFi init (default)
    wifi_init();
    
    // Bluetooth standby
    // bluetooth_init();
    
    uart_puts("[RC] Receiver ready. Mode: WiFi\n");
}

void rc_receiver_update(void) {
    extern rc_input_t rc;
    
    switch (active_rc_mode) {
        case RC_MODE_PPM:
            ppm_get_values(&rc);
            break;
            
        case RC_MODE_SBUS:
            sbus_get_values(&rc);
            break;
            
        case RC_MODE_WIFI:
            // WiFi UART dan o'qish
            // wifi_parse_command();
            break;
            
        case RC_MODE_BLUETOOTH:
            // Bluetooth dan o'qish
            break;
    }
    
    // Failsafe check
    rc_failsafe_handler();
    
    // Signal strength LED
    if (rc_signal_valid) {
        gpio_write(22, 1);  // LED on
    } else {
        gpio_write(22, 0);  // LED off
    }
}

// ============================================================================
// WEB INTERFACE (WiFi control)
// ============================================================================

const char* html_page = 
"HTTP/1.1 200 OK\r\n"
"Content-Type: text/html\r\n"
"\r\n"
"<!DOCTYPE html>"
"<html>"
"<head><title>DroneOS Control</title></head>"
"<body style='text-align:center; font-family:Arial;'>"
"<h1>DroneOS Remote Control</h1>"
"<div id='joystick' style='width:300px; height:300px; border:2px solid #000; margin:20px auto;'></div>"
"<p>Throttle: <span id='throttle'>0</span></p>"
"<p>Roll: <span id='roll'>0</span></p>"
"<p>Pitch: <span id='pitch'>0</span></p>"
"<p>Battery: <span id='battery'>11.1V</span></p>"
"<button onclick='arm()'>ARM</button>"
"<button onclick='disarm()'>DISARM</button>"
"<script>"
"function sendRC(t,r,p,y){"
"  fetch('/rc?t='+t+'&r='+r+'&p='+p+'&y='+y);"
"}"
"setInterval(()=>{"
"  fetch('/telemetry').then(r=>r.json()).then(d=>{"
"    document.getElementById('battery').innerText=d.battery+'V';"
"  });"
"}, 100);"
"</script>"
"</body>"
"</html>";

void web_server_send_page(void) {
    // ESP8266 orqali HTTP response
    uart_puts("AT+CIPSEND=0,");
    // uart_puts(strlen(html_page));
    uart_puts("\r\n");
    delay_ms(10);
    uart_puts(html_page);
}