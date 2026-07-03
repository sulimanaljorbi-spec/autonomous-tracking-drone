/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Experiments 1 + 2 + 3 — Complete Flight Controller
  * STM32F4 — Autonomous GPS Mission Planning
  *
  * Experiment 1: IMU sensor fusion, PID stabilization, motor mixing
  * Experiment 2: Jetson vision tracking via UART protocol
  * Experiment 3: Waypoint navigation, RTH, Geofencing, Battery monitor
  *
  * RC Switch Mapping:
  *   AUX1 (ch5) > 1700  → Start mission (if uploaded) or RTH
  *   AUX2 (ch6) < 1200  → Kill switch (motors off immediately)
  *
  * Battery ADC:
  *   PA0 → voltage divider (10kΩ + 3.3kΩ) from LiPo
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
// ── Protocol state machine ───────────────────────────────────────────────
typedef enum {
    WAIT_H1, WAIT_H2, WAIT_ID, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM
} ProtoRxState_t;

// ── Waypoint action ──────────────────────────────────────────────────────
typedef enum {
    WP_ACTION_FLYTHROUGH = 0,
    WP_ACTION_HOVER      = 1,
    WP_ACTION_LAND       = 2,
    WP_ACTION_RTH        = 3
} WP_Action_t;

// ── Waypoint ─────────────────────────────────────────────────────────────
typedef struct {
    float       lat;
    float       lon;
    float       alt;
    float       speed;
    float       accept_radius;
    WP_Action_t action;
    uint16_t    hover_time;
} Waypoint_t;

// ── Mission ──────────────────────────────────────────────────────────────
typedef struct {
    Waypoint_t waypoints[20];
    uint8_t    count;
    uint8_t    current_idx;
    uint8_t    is_active;
    Waypoint_t home;
} Mission_t;

// ── Navigation states ─────────────────────────────────────────────────────
typedef enum {
    NAV_IDLE     = 0,
    NAV_TAKEOFF  = 1,
    NAV_NAVIGATE = 2,
    NAV_HOVER    = 3,
    NAV_ADVANCE  = 4,
    NAV_RTH      = 5,
    NAV_LAND     = 6
} NavState_t;

// ── Geofence ─────────────────────────────────────────────────────────────
typedef struct {
    float   center_lat;
    float   center_lon;
    float   radius;
    float   max_altitude;
    float   min_altitude;
    uint8_t enabled;
} Geofence_t;

typedef enum {
    GEO_OK = 0, GEO_WARNING = 1, GEO_BREACH = 2
} GeoStatus_t;

// ── Navigation PID ────────────────────────────────────────────────────────
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
} NavPID_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
// I2C addresses
#define MPU6050_ADDR     (0x68 << 1)
#define QMC5883L_ADDR    (0x0D << 1)

// Protocol bytes
#define PROTO_H1         0xAA
#define PROTO_H2         0x55
#define MSG_HEARTBEAT    0x01
#define MSG_TRACK_CMD    0x02
#define MSG_VELOCITY     0x03
#define MSG_ATTITUDE     0x10
#define MSG_GPS          0x11
#define MSG_BATTERY      0x12
#define MSG_WP_UPLOAD    0x20
#define MSG_MISSION_CMD  0x21
#define MSG_NAV_STATUS   0x30
#define MSG_GEO_STATUS   0x31
#define MAX_PAYLOAD      32

// GPS math
#define R_EARTH          6371000.0f
#define DEG2RAD          0.017453293f

// RTH parameters
#define RTH_ALTITUDE     30.0f
#define RTH_SPEED        3.0f
#define LAND_THRESHOLD   0.5f

// Battery thresholds (3S LiPo)
#define VDIV_RATIO       (13.3f / 3.3f)
#define ADC_MAX          4095.0f
#define VREF             3.3f
#define BATT_WARNING     10.8f
#define BATT_CRITICAL    10.2f
/* USER CODE END PD */

/* Peripheral handles */
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart1, huart2, huart3, huart4;
DMA_HandleTypeDef  hdma_uart4_rx;
ADC_HandleTypeDef  hadc1;

/* USER CODE BEGIN PV */
// ── IMU raw data ─────────────────────────────────────────────────────────
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW,  Gyro_Y_RAW,  Gyro_Z_RAW;
int16_t Mag_X_RAW,   Mag_Y_RAW,   Mag_Z_RAW;

// ── Sensor calibration ───────────────────────────────────────────────────
float Mag_X_Offset  = -1581.0f;
float Mag_Y_Offset  =    70.0f;
float Roll_Offset   =  -3.99461436f;
float Pitch_Offset  =  -2.35308671f;
float Yaw_Offset    = 174.983521f;

// ── Attitude ─────────────────────────────────────────────────────────────
float Roll = 0, Pitch = 0, Yaw = 0, dt = 0.01f;
float Roll_Setpoint = 0, Pitch_Setpoint = 0, Yaw_Setpoint = 0;
float Kp = 1.0f, Ki = 0.01f, Kd = 0.5f;
float PID_Roll = 0, PID_Pitch = 0, PID_Yaw = 0;
float Int_Roll = 0, Int_Pitch = 0, Int_Yaw = 0;
float Prev_Roll = 0, Prev_Pitch = 0, Prev_Yaw = 0;

// ── RC input ─────────────────────────────────────────────────────────────
uint8_t  ibus_data[64];
uint16_t roll_in = 1500, pitch_in = 1500;
uint16_t throttle = 1000, yaw_in = 1500;
uint16_t aux1 = 1000, aux2 = 1000;
uint32_t last_rc_tick = 0;

// ── GPS ──────────────────────────────────────────────────────────────────
uint8_t  gps_rx_data;
char     gps_buffer[100];
uint8_t  gps_index = 0;
uint8_t  gps_sentence_ready = 0;
float    GPS_Lat = 0.0f, GPS_Lon = 0.0f, GPS_Alt = 0.0f;
uint8_t  GPS_Fix = 0, GPS_Sats = 0;
uint32_t last_gps_fix_tick = 0;

// ── Experiment 2 — Vision ────────────────────────────────────────────────
uint8_t           vision_rx_data      = 0;
volatile float    vision_offset_x     = 0.0f;
volatile float    vision_offset_y     = 0.0f;
volatile float    vision_confidence   = 0.0f;
volatile uint8_t  vision_frame_ready  = 0;
volatile uint32_t last_vision_tick    = 0;
volatile uint8_t  jetson_alive        = 0;
volatile uint32_t last_heartbeat_tick = 0;

// ── Protocol parser (file-scope) ─────────────────────────────────────────
static ProtoRxState_t proto_state    = WAIT_H1;
static uint8_t  proto_msg_id         = 0;
static uint8_t  proto_length         = 0;
static uint8_t  proto_payload[MAX_PAYLOAD];
static uint8_t  proto_idx            = 0;
static uint8_t  proto_checksum       = 0;

// ── Experiment 3 — Navigation ────────────────────────────────────────────
Mission_t  mission   = {0};
NavState_t nav_state = NAV_IDLE;
uint32_t   hover_start_time = 0;
uint8_t    rth_phase = 0;

// ── Experiment 3 — Geofence ──────────────────────────────────────────────
Geofence_t geofence = {
    .center_lat   = 0.0f,
    .center_lon   = 0.0f,
    .radius       = 50.0f,
    .max_altitude = 30.0f,
    .min_altitude = 0.0f,
    .enabled      = 0
};

// ── Experiment 3 — Navigation PIDs ───────────────────────────────────────
NavPID_t pid_nav_yaw = {0};
NavPID_t pid_alt     = {0};

// ── Experiment 3 — Battery ───────────────────────────────────────────────
float    battery_voltage = 12.6f;
uint32_t last_batt_tick  = 0;
/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
// Exp1
void  Sensors_Init(void);
void  Sensors_Read(void);
void  Calculate_Attitude(void);
void  Parse_iBUS(void);
float PID_Generic(float set, float cur, float *integral, float *prev);
void  Parse_GPS(void);

// Exp2 protocol
void Protocol_ParseByte(uint8_t byte);
void Protocol_HandleMessage(uint8_t msg_id, uint8_t *payload, uint8_t len);
void Protocol_SendFrame(uint8_t msg_id, uint8_t *payload, uint8_t len);
void Protocol_SendAttitude(float roll, float pitch, float yaw, float alt);
void Protocol_SendGPS(float lat, float lon, float speed);
void Protocol_SendBattery(float voltage, float current);
void Protocol_SendNavStatus(uint8_t state, uint8_t wp_idx);
void Protocol_SendGeoStatus(uint8_t status);

// Exp3
float       GPS_Distance(Waypoint_t *a, Waypoint_t *b);
float       GPS_Bearing(Waypoint_t *from, Waypoint_t *to);
void        NavPID_Init(NavPID_t *pid, float kp, float ki, float kd,
                        float out_min, float out_max);
float       NavPID_Compute(NavPID_t *pid, float setpoint,
                           float measured, float dt_s);
void        Navigation_Init(void);
void        Navigation_Update(float dt_nav);
void        RTH_Execute(float dt_nav);
GeoStatus_t Geofence_Check(void);
void        Geofence_Enforce(void);
float       Battery_ReadVoltage(void);
void        Battery_CheckAndAct(void);
void        Mission_SetHome(void);
void        Motors_SetAll(uint32_t value);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// ── Debug printf ─────────────────────────────────────────────────────────
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

// ════════════════════════════════════════════════════════════════════════
// EXPERIMENT 1 — Sensors, Attitude, PID
// ════════════════════════════════════════════════════════════════════════

void Sensors_Init(void) {
    uint8_t data;
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,  0x6B, 1, &data, 1, 100);
    data = 0x01; HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, 0x0B, 1, &data, 1, 100);
    data = 0x1D; HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, 0x09, 1, &data, 1, 100);
}

void Sensors_Read(void) {
    uint8_t mpu_buf[14], mag_buf[6];
    static uint8_t i2c_err = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, mpu_buf, 14, 10) == HAL_OK) {
        Accel_X_RAW = (int16_t)(mpu_buf[0]  << 8 | mpu_buf[1]);
        Accel_Y_RAW = (int16_t)(mpu_buf[2]  << 8 | mpu_buf[3]);
        Accel_Z_RAW = (int16_t)(mpu_buf[4]  << 8 | mpu_buf[5]);
        Gyro_X_RAW  = (int16_t)(mpu_buf[8]  << 8 | mpu_buf[9]);
        Gyro_Y_RAW  = (int16_t)(mpu_buf[10] << 8 | mpu_buf[11]);
        Gyro_Z_RAW  = (int16_t)(mpu_buf[12] << 8 | mpu_buf[13]);
        i2c_err = 0;
    } else {
        i2c_err++;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR, 0x00, 1, mag_buf, 6, 10) == HAL_OK) {
        Mag_X_RAW = (int16_t)(mag_buf[1] << 8 | mag_buf[0]);
        Mag_Y_RAW = (int16_t)(mag_buf[3] << 8 | mag_buf[2]);
        Mag_Z_RAW = (int16_t)(mag_buf[5] << 8 | mag_buf[4]);
    }
    if (i2c_err > 5) {
        HAL_I2C_DeInit(&hi2c1); HAL_Delay(1);
        MX_I2C1_Init(); Sensors_Init(); i2c_err = 0;
    }
}

void Calculate_Attitude(void) {
    float acc_p = (atan2f((float)Accel_Y_RAW,
                          (float)Accel_Z_RAW) * 57.3f) - Pitch_Offset;
    float acc_r = (atan2f(-(float)Accel_X_RAW,
                  sqrtf((float)Accel_Y_RAW * Accel_Y_RAW +
                        (float)Accel_Z_RAW * Accel_Z_RAW)) * 57.3f) - Roll_Offset;
    Roll  = 0.98f * (Roll  + ((float)Gyro_X_RAW / 65.5f) * dt) + 0.02f * acc_r;
    Pitch = 0.98f * (Pitch + ((float)Gyro_Y_RAW / 65.5f) * dt) + 0.02f * acc_p;
    float cal_X = (float)Mag_X_RAW - Mag_X_Offset;
    float cal_Y = (float)Mag_Y_RAW - Mag_Y_Offset;
    float r_rad = Roll * 0.01745f, p_rad = Pitch * 0.01745f;
    float Xh = cal_X * cosf(p_rad) + (float)Mag_Z_RAW * sinf(p_rad);
    float Yh = cal_X * sinf(r_rad) * sinf(p_rad)
             + cal_Y * cosf(r_rad)
             - (float)Mag_Z_RAW * sinf(r_rad) * cosf(p_rad);
    float mag_yaw = (atan2f(Yh, Xh) * 57.3f) - Yaw_Offset;
    Yaw = 0.98f * (Yaw + ((float)Gyro_Z_RAW / 65.5f) * dt) + 0.02f * mag_yaw;
}

void Parse_iBUS(void) {
    for (int i = 0; i < 32; i++) {
        if (ibus_data[i] == 0x20 && ibus_data[i+1] == 0x40) {
            roll_in  = (ibus_data[i+3]  << 8) | ibus_data[i+2];
            pitch_in = (ibus_data[i+5]  << 8) | ibus_data[i+4];
            throttle = (ibus_data[i+7]  << 8) | ibus_data[i+6];
            yaw_in   = (ibus_data[i+9]  << 8) | ibus_data[i+8];
            aux1     = (ibus_data[i+11] << 8) | ibus_data[i+10];
            aux2     = (ibus_data[i+13] << 8) | ibus_data[i+12];
            last_rc_tick = HAL_GetTick();
            break;
        }
    }
}

float PID_Generic(float set, float cur, float *integral, float *prev) {
    float error = set - cur;
    *integral  += error * dt;
    if (*integral >  400) *integral =  400;
    if (*integral < -400) *integral = -400;
    float derivative = (error - *prev) / dt;
    *prev = error;
    return (Kp * error) + (Ki * *integral) + (Kd * derivative);
}

void Parse_GPS(void) {
    if (!gps_sentence_ready) return;
    if (strstr(gps_buffer, "$GPGGA") || strstr(gps_buffer, "$GNGGA")) {
        char  ns, ew;
        float time_val, lat, lon, hdop, alt;
        int   fix, sats;
        if (sscanf(gps_buffer, "%*[^,],%f,%f,%c,%f,%c,%d,%d,%f,%f",
                   &time_val, &lat, &ns, &lon, &ew,
                   &fix, &sats, &hdop, &alt) >= 9) {
            GPS_Fix = fix; GPS_Sats = sats;
            if (fix > 0) {
                int   ld = (int)(lat / 100);
                GPS_Lat  = ld + (lat - ld * 100) / 60.0f;
                if (ns == 'S') GPS_Lat *= -1.0f;
                int   od = (int)(lon / 100);
                GPS_Lon  = od + (lon - od * 100) / 60.0f;
                if (ew == 'W') GPS_Lon *= -1.0f;
                GPS_Alt           = alt;
                last_gps_fix_tick = HAL_GetTick();
            }
        }
    }
    gps_sentence_ready = 0;
    memset(gps_buffer, 0, sizeof(gps_buffer));
}

void Motors_SetAll(uint32_t value) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, value);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, value);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, value);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, value);
}

// ════════════════════════════════════════════════════════════════════════
// EXPERIMENT 2 — UART Protocol
// ════════════════════════════════════════════════════════════════════════

void Protocol_ParseByte(uint8_t byte) {
    switch (proto_state) {
        case WAIT_H1:
            if (byte == PROTO_H1) proto_state = WAIT_H2;
            break;
        case WAIT_H2:
            proto_state = (byte == PROTO_H2) ? WAIT_ID : WAIT_H1;
            break;
        case WAIT_ID:
            proto_msg_id = byte; proto_checksum = byte; proto_state = WAIT_LEN;
            break;
        case WAIT_LEN:
            proto_length = byte; proto_checksum ^= byte; proto_idx = 0;
            proto_state  = (byte > 0) ? WAIT_PAYLOAD : WAIT_CHECKSUM;
            break;
        case WAIT_PAYLOAD:
            proto_payload[proto_idx++] = byte; proto_checksum ^= byte;
            if (proto_idx >= proto_length) proto_state = WAIT_CHECKSUM;
            break;
        case WAIT_CHECKSUM:
            if (byte == (proto_checksum & 0xFF))
                Protocol_HandleMessage(proto_msg_id, proto_payload, proto_length);
            proto_state = WAIT_H1;
            break;
    }
}

void Protocol_HandleMessage(uint8_t msg_id, uint8_t *payload, uint8_t len) {
    switch (msg_id) {

        case MSG_HEARTBEAT:
            if (len >= 1) {
                jetson_alive        = payload[0];
                last_heartbeat_tick = HAL_GetTick();
            }
            break;

        case MSG_TRACK_CMD:
            if (len >= 6) {
                int16_t  tx   = (int16_t) ((payload[0] << 8) | payload[1]);
                int16_t  ty   = (int16_t) ((payload[2] << 8) | payload[3]);
                uint16_t conf = (uint16_t)((payload[4] << 8) | payload[5]);
                vision_offset_x    = (float)tx;
                vision_offset_y    = (float)ty;
                vision_confidence  = (float)conf / 100.0f;
                vision_frame_ready = 1;
                last_vision_tick   = HAL_GetTick();
            }
            break;

        case MSG_WP_UPLOAD:
            // Payload: lat(4f) lon(4f) alt(1B) speed(4f) action(1B) hover(2H)
            if (len >= 16 && mission.count < 20) {
                Waypoint_t wp = {0};
                memcpy(&wp.lat,   &payload[0], 4);
                memcpy(&wp.lon,   &payload[4], 4);
                wp.alt        = (float)payload[8];
                memcpy(&wp.speed, &payload[9], 4);
                wp.action     = (WP_Action_t)payload[13];
                wp.hover_time = (uint16_t)((payload[14] << 8) | payload[15]);
                wp.accept_radius = 2.0f;
                mission.waypoints[mission.count++] = wp;
                printf("WP%d received: %.6f %.6f alt=%.0f\r\n",
                       mission.count, wp.lat, wp.lon, wp.alt);
            }
            break;

        case MSG_MISSION_CMD:
            if (len >= 1) {
                if (payload[0] == 0x01 && mission.count > 0 && GPS_Fix > 0) {
                    Mission_SetHome();
                    mission.current_idx = 0;
                    mission.is_active   = 1;
                    nav_state           = NAV_TAKEOFF;
                    rth_phase           = 0;
                    printf("Mission START — %d waypoints\r\n", mission.count);
                } else if (payload[0] == 0x00) {
                    mission.is_active = 0;
                    nav_state         = NAV_RTH;
                    rth_phase         = 0;
                    printf("Mission ABORT — RTH\r\n");
                }
            }
            break;

        default:
            break;
    }
}

void Protocol_SendFrame(uint8_t msg_id, uint8_t *payload, uint8_t len) {
    uint8_t buf[5 + MAX_PAYLOAD];
    uint8_t checksum = msg_id ^ len;
    buf[0] = PROTO_H1; buf[1] = PROTO_H2;
    buf[2] = msg_id;   buf[3] = len;
    for (uint8_t i = 0; i < len; i++) {
        buf[4 + i]  = payload[i];
        checksum   ^= payload[i];
    }
    buf[4 + len] = checksum & 0xFF;
    HAL_UART_Transmit(&huart3, buf, 5 + len, 10);
}

void Protocol_SendAttitude(float roll, float pitch, float yaw, float alt) {
    uint8_t p[8];
    int16_t r=(int16_t)(roll*100),  pi=(int16_t)(pitch*100),
            y=(int16_t)(yaw*100),   a=(int16_t)(alt*100);
    p[0]=(r>>8)&0xFF;  p[1]=r&0xFF;
    p[2]=(pi>>8)&0xFF; p[3]=pi&0xFF;
    p[4]=(y>>8)&0xFF;  p[5]=y&0xFF;
    p[6]=(a>>8)&0xFF;  p[7]=a&0xFF;
    Protocol_SendFrame(MSG_ATTITUDE, p, 8);
}

void Protocol_SendGPS(float lat, float lon, float speed) {
    uint8_t p[10];
    int32_t la=(int32_t)(lat*1e6f), lo=(int32_t)(lon*1e6f);
    int16_t spd=(int16_t)(speed*100);
    p[0]=(la>>24)&0xFF; p[1]=(la>>16)&0xFF;
    p[2]=(la>>8)&0xFF;  p[3]=la&0xFF;
    p[4]=(lo>>24)&0xFF; p[5]=(lo>>16)&0xFF;
    p[6]=(lo>>8)&0xFF;  p[7]=lo&0xFF;
    p[8]=(spd>>8)&0xFF; p[9]=spd&0xFF;
    Protocol_SendFrame(MSG_GPS, p, 10);
}

void Protocol_SendBattery(float voltage, float current) {
    uint8_t p[4];
    int16_t v=(int16_t)(voltage*100), c=(int16_t)(current*100);
    p[0]=(v>>8)&0xFF; p[1]=v&0xFF;
    p[2]=(c>>8)&0xFF; p[3]=c&0xFF;
    Protocol_SendFrame(MSG_BATTERY, p, 4);
}

void Protocol_SendNavStatus(uint8_t state, uint8_t wp_idx) {
    uint8_t p[2] = {state, wp_idx};
    Protocol_SendFrame(MSG_NAV_STATUS, p, 2);
}

void Protocol_SendGeoStatus(uint8_t status) {
    uint8_t p[1] = {status};
    Protocol_SendFrame(MSG_GEO_STATUS, p, 1);
}

// ════════════════════════════════════════════════════════════════════════
// EXPERIMENT 3 — GPS Math, Navigation, RTH, Geofence, Battery
// ════════════════════════════════════════════════════════════════════════

float GPS_Distance(Waypoint_t *a, Waypoint_t *b) {
    float dlat = (b->lat - a->lat) * DEG2RAD;
    float dlon = (b->lon - a->lon) * DEG2RAD;
    float lat1 = a->lat * DEG2RAD;
    float lat2 = b->lat * DEG2RAD;
    float x = sinf(dlat/2)*sinf(dlat/2)
            + cosf(lat1)*cosf(lat2)*sinf(dlon/2)*sinf(dlon/2);
    return R_EARTH * 2.0f * atan2f(sqrtf(x), sqrtf(1.0f - x));
}

float GPS_Bearing(Waypoint_t *from, Waypoint_t *to) {
    float dlon = (to->lon - from->lon) * DEG2RAD;
    float lat1 = from->lat * DEG2RAD;
    float lat2 = to->lat   * DEG2RAD;
    float y    = sinf(dlon) * cosf(lat2);
    float x    = cosf(lat1)*sinf(lat2) - sinf(lat1)*cosf(lat2)*cosf(dlon);
    float b    = atan2f(y, x) / DEG2RAD;
    if (b < 0) b += 360.0f;
    return b;
}

void NavPID_Init(NavPID_t *pid, float kp, float ki, float kd,
                 float out_min, float out_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = 0; pid->prev_error = 0;
    pid->out_min  = out_min; pid->out_max = out_max;
}

float NavPID_Compute(NavPID_t *pid, float setpoint, float measured, float dt_s) {
    float error   = setpoint - measured;
    pid->integral += error * dt_s;
    float out = pid->kp * error
              + pid->ki * pid->integral
              + pid->kd * ((error - pid->prev_error) / dt_s);
    pid->prev_error = error;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return out;
}

void Navigation_Init(void) {
    NavPID_Init(&pid_nav_yaw, 0.8f,  0.01f, 0.3f, -30.0f,  30.0f);
    NavPID_Init(&pid_alt,     1.5f,  0.05f, 0.5f, 1000.0f, 1800.0f);
}

void Mission_SetHome(void) {
    mission.home.lat     = GPS_Lat;
    mission.home.lon     = GPS_Lon;
    mission.home.alt     = GPS_Alt;
    geofence.center_lat  = GPS_Lat;
    geofence.center_lon  = GPS_Lon;
    geofence.enabled     = 1;
    printf("Home set: %.6f %.6f alt=%.1f\r\n", GPS_Lat, GPS_Lon, GPS_Alt);
}

void Navigation_Update(float dt_nav) {
    if (!mission.is_active) return;

    Waypoint_t pos = {GPS_Lat, GPS_Lon, GPS_Alt, 0, 0, 0, 0};
    Waypoint_t *wp;
    float dist, bearing, heading_err, yaw_cmd, pitch_cmd, alt_cmd;

    switch (nav_state) {

        case NAV_IDLE:
            break;

        case NAV_TAKEOFF:
            wp      = &mission.waypoints[0];
            alt_cmd = NavPID_Compute(&pid_alt, wp->alt, GPS_Alt, dt_nav);
            Motors_SetAll((uint32_t)alt_cmd);
            printf("TAKEOFF %.1f/%.1fm\r\n", GPS_Alt, wp->alt);
            if (fabsf(GPS_Alt - wp->alt) < 1.0f)
                nav_state = NAV_NAVIGATE;
            break;

        case NAV_NAVIGATE:
            wp          = &mission.waypoints[mission.current_idx];
            dist        = GPS_Distance(&pos, wp);
            bearing     = GPS_Bearing(&pos, wp);
            heading_err = bearing - Yaw;
            if (heading_err >  180.0f) heading_err -= 360.0f;
            if (heading_err < -180.0f) heading_err += 360.0f;

            yaw_cmd     = NavPID_Compute(&pid_nav_yaw, 0.0f, -heading_err, dt_nav);
            pitch_cmd   = fminf(dist * 0.5f, wp->speed);
            alt_cmd     = NavPID_Compute(&pid_alt, wp->alt, GPS_Alt, dt_nav);

            Yaw_Setpoint   = yaw_cmd;
            Pitch_Setpoint = -pitch_cmd;

            printf("NAV WP%d dist=%.1fm bear=%.0f\r\n",
                   mission.current_idx + 1, dist, bearing);

            if (dist < wp->accept_radius) {
                printf("Arrived WP%d action=%d\r\n",
                       mission.current_idx + 1, wp->action);
                if      (wp->action == WP_ACTION_HOVER) {
                    nav_state        = NAV_HOVER;
                    hover_start_time = HAL_GetTick();
                }
                else if (wp->action == WP_ACTION_LAND) nav_state = NAV_LAND;
                else if (wp->action == WP_ACTION_RTH)  {
                    nav_state = NAV_RTH; rth_phase = 0;
                }
                else nav_state = NAV_ADVANCE;
            }
            break;

        case NAV_HOVER:
            wp             = &mission.waypoints[mission.current_idx];
            Pitch_Setpoint = 0;
            Roll_Setpoint  = 0;
            alt_cmd = NavPID_Compute(&pid_alt, wp->alt, GPS_Alt, dt_nav);
            printf("HOVER WP%d  %.0fs left\r\n",
                   mission.current_idx + 1,
                   wp->hover_time - (HAL_GetTick() - hover_start_time)/1000UL);
            if (HAL_GetTick() - hover_start_time > (uint32_t)wp->hover_time * 1000U)
                nav_state = NAV_ADVANCE;
            break;

        case NAV_ADVANCE:
            mission.current_idx++;
            if (mission.current_idx >= mission.count) {
                printf("Mission complete — RTH\r\n");
                nav_state = NAV_RTH; rth_phase = 0;
            } else {
                printf("Next: WP%d\r\n", mission.current_idx + 1);
                nav_state = NAV_NAVIGATE;
            }
            break;

        case NAV_RTH:
            RTH_Execute(dt_nav);
            break;

        case NAV_LAND:
            Pitch_Setpoint = 0;
            Roll_Setpoint  = 0;
            Motors_SetAll(1150);
            printf("LANDING %.1fm\r\n", GPS_Alt);
            if (GPS_Alt < LAND_THRESHOLD) {
                Motors_SetAll(1000);
                nav_state         = NAV_IDLE;
                mission.is_active = 0;
                printf("Landed.\r\n");
            }
            break;
    }
}

void RTH_Execute(float dt_nav) {
    Waypoint_t pos = {GPS_Lat, GPS_Lon, GPS_Alt, 0, 0, 0, 0};
    float dist_home, bearing, heading_err, alt_cmd;

    switch (rth_phase) {
        case 0:  // Climb to safe altitude
            alt_cmd = NavPID_Compute(&pid_alt, RTH_ALTITUDE, GPS_Alt, dt_nav);
            Motors_SetAll((uint32_t)alt_cmd);
            printf("RTH climb %.1f/%.0fm\r\n", GPS_Alt, RTH_ALTITUDE);
            if (GPS_Alt >= RTH_ALTITUDE - 1.0f) rth_phase = 1;
            break;

        case 1:  // Fly toward home
            dist_home   = GPS_Distance(&pos, &mission.home);
            bearing     = GPS_Bearing(&pos, &mission.home);
            heading_err = bearing - Yaw;
            if (heading_err >  180.0f) heading_err -= 360.0f;
            if (heading_err < -180.0f) heading_err += 360.0f;
            Yaw_Setpoint   =  heading_err * 0.5f;
            Pitch_Setpoint = -fminf(dist_home * 0.3f, RTH_SPEED);
            alt_cmd = NavPID_Compute(&pid_alt, RTH_ALTITUDE, GPS_Alt, dt_nav);
            printf("RTH flying home %.1fm\r\n", dist_home);
            if (dist_home < 2.0f) rth_phase = 2;
            break;

        case 2:  // Descend and land
            Pitch_Setpoint = 0;
            Roll_Setpoint  = 0;
            Motors_SetAll(1150);
            printf("RTH descend %.1fm\r\n", GPS_Alt);
            if (GPS_Alt < LAND_THRESHOLD) {
                Motors_SetAll(1000);
                rth_phase         = 0;
                nav_state         = NAV_IDLE;
                mission.is_active = 0;
                printf("RTH complete. Landed.\r\n");
            }
            break;
    }
}

GeoStatus_t Geofence_Check(void) {
    if (!geofence.enabled) return GEO_OK;
    Waypoint_t pos    = {GPS_Lat, GPS_Lon, 0, 0, 0, 0, 0};
    Waypoint_t center = {geofence.center_lat, geofence.center_lon, 0, 0, 0, 0, 0};
    float dist = GPS_Distance(&pos, &center);
    if (GPS_Alt > geofence.max_altitude || GPS_Alt < geofence.min_altitude)
        return GEO_BREACH;
    if (dist > geofence.radius)         return GEO_BREACH;
    if (dist > geofence.radius * 0.8f)  return GEO_WARNING;
    return GEO_OK;
}

void Geofence_Enforce(void) {
    GeoStatus_t status = Geofence_Check();
    if (status == GEO_WARNING) {
        printf("GEOFENCE WARNING\r\n");
    } else if (status == GEO_BREACH) {
        printf("GEOFENCE BREACH — RTH\r\n");
        nav_state = NAV_RTH;
        rth_phase = 0;
    }
    Protocol_SendGeoStatus((uint8_t)status);
}

float Battery_ReadVoltage(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    return ((float)raw / ADC_MAX) * VREF * VDIV_RATIO;
}

void Battery_CheckAndAct(void) {
    battery_voltage = Battery_ReadVoltage();
    printf("Battery: %.2fV\r\n", battery_voltage);
    if (battery_voltage < BATT_CRITICAL) {
        printf("CRITICAL BATTERY — LAND\r\n");
        nav_state = NAV_LAND;
    } else if (battery_voltage < BATT_WARNING) {
        printf("LOW BATTERY — RTH\r\n");
        if (nav_state != NAV_RTH && nav_state != NAV_LAND) {
            nav_state = NAV_RTH; rth_phase = 0;
        }
    }
}

/* USER CODE END 0 */

// ════════════════════════════════════════════════════════════════════════
// main()
// ════════════════════════════════════════════════════════════════════════
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();

    /* USER CODE BEGIN 2 */
    Sensors_Init();
    Navigation_Init();

    HAL_UART_Receive_DMA(&huart4, ibus_data, 64);
    HAL_UART_Receive_IT(&huart1, &gps_rx_data, 1);
    HAL_UART_Receive_IT(&huart3, &vision_rx_data, 1);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    Motors_SetAll(1000);

    HAL_Delay(3000);
    printf("System ready. Waiting for GPS fix...\r\n");
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN WHILE */

        // 1. Read sensors and RC
        Parse_iBUS();
        Sensors_Read();
        Calculate_Attitude();
        Parse_GPS();

        // 2. KILL SWITCH — AUX2 < 1200 (highest priority, always checked first)
        if (aux2 < 1200) {
            Int_Roll = Int_Pitch = Int_Yaw = 0;
            Prev_Roll = Prev_Pitch = Prev_Yaw = 0;
            mission.is_active = 0;
            nav_state         = NAV_IDLE;
            Motors_SetAll(1000);
            HAL_Delay(10);
            continue;
        }

        // 3. RC signal lost > 2s → RTH
        if ((HAL_GetTick() - last_rc_tick) > 2000 && mission.is_active) {
            printf("RC LOST — RTH\r\n");
            nav_state = NAV_RTH; rth_phase = 0;
        }

        // 4. GPS fix lost > 5s → land
        if (GPS_Fix == 0
            && (HAL_GetTick() - last_gps_fix_tick) > 5000
            && mission.is_active) {
            printf("GPS LOST — LAND\r\n");
            nav_state = NAV_LAND;
        }

        // 5. RTH switch AUX1 > 1700
        if (aux1 > 1700
            && nav_state != NAV_RTH
            && nav_state != NAV_LAND
            && nav_state != NAV_IDLE) {
            printf("RTH switch\r\n");
            nav_state = NAV_RTH; rth_phase = 0;
        }

        // 6. Battery check every 5 seconds
        if ((HAL_GetTick() - last_batt_tick) > 5000) {
            Battery_CheckAndAct();
            last_batt_tick = HAL_GetTick();
        }

        // 7. Geofence (only when flying with GPS fix)
        if (mission.is_active && GPS_Fix > 0)
            Geofence_Enforce();

        // 8. Flight mode
        if (mission.is_active && nav_state != NAV_IDLE) {

            // ── AUTONOMOUS MODE ──────────────────────────────────────
            Navigation_Update(dt);

            // Attitude PID still runs in hover to maintain stability
            if (nav_state == NAV_HOVER || nav_state == NAV_NAVIGATE) {
                PID_Roll  = PID_Generic(Roll_Setpoint,  Roll,  &Int_Roll,  &Prev_Roll);
                PID_Pitch = PID_Generic(Pitch_Setpoint, Pitch, &Int_Pitch, &Prev_Pitch);
                PID_Yaw   = PID_Generic(Yaw_Setpoint,   Yaw,   &Int_Yaw,   &Prev_Yaw);
                float m1 = throttle + PID_Roll - PID_Pitch + PID_Yaw;
                float m2 = throttle - PID_Roll - PID_Pitch - PID_Yaw;
                float m3 = throttle - PID_Roll + PID_Pitch + PID_Yaw;
                float m4 = throttle + PID_Roll + PID_Pitch - PID_Yaw;
                #define CLAMP(x) ((x)<1000?1000:((x)>2000?2000:(x)))
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)CLAMP(m1));
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)CLAMP(m2));
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)CLAMP(m3));
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)CLAMP(m4));
            }

        } else {

            // ── MANUAL + VISION MODE ─────────────────────────────────
            float stick_roll  = (roll_in  > 1000) ? (roll_in  - 1500) / 16.6f : 0;
            float stick_pitch = (pitch_in > 1000) ? (pitch_in - 1500) / 16.6f : 0;
            Yaw_Setpoint      = (yaw_in   > 1000) ? (yaw_in   - 1500) / 16.6f : 0;

            // Vision fusion (Exp2)
            float v_adj_roll = 0.0f, v_adj_pitch = 0.0f;
            if ((HAL_GetTick() - last_vision_tick) > 1000) {
                vision_offset_x = 0; vision_offset_y = 0; vision_frame_ready = 0;
            }
            if (vision_frame_ready && vision_confidence > 0.3f) {
                v_adj_roll  = vision_offset_x * 0.04f;
                v_adj_pitch = vision_offset_y * 0.04f;
                if (v_adj_roll  >  15.0f) v_adj_roll  =  15.0f;
                if (v_adj_roll  < -15.0f) v_adj_roll  = -15.0f;
                if (v_adj_pitch >  15.0f) v_adj_pitch =  15.0f;
                if (v_adj_pitch < -15.0f) v_adj_pitch = -15.0f;
                vision_frame_ready = 0;
            }
            Roll_Setpoint  = stick_roll  + v_adj_roll;
            Pitch_Setpoint = stick_pitch - v_adj_pitch;
            if (Roll_Setpoint  >  25.0f) Roll_Setpoint  =  25.0f;
            if (Roll_Setpoint  < -25.0f) Roll_Setpoint  = -25.0f;
            if (Pitch_Setpoint >  25.0f) Pitch_Setpoint =  25.0f;
            if (Pitch_Setpoint < -25.0f) Pitch_Setpoint = -25.0f;

            if (throttle < 1050) {
                Int_Roll = Int_Pitch = Int_Yaw = 0;
                Prev_Roll = Prev_Pitch = Prev_Yaw = 0;
                Motors_SetAll(1000);
            } else {
                PID_Roll  = PID_Generic(Roll_Setpoint,  Roll,  &Int_Roll,  &Prev_Roll);
                PID_Pitch = PID_Generic(Pitch_Setpoint, Pitch, &Int_Pitch, &Prev_Pitch);
                PID_Yaw   = PID_Generic(Yaw_Setpoint,   Yaw,   &Int_Yaw,   &Prev_Yaw);
                float m1 = throttle + PID_Roll - PID_Pitch + PID_Yaw;
                float m2 = throttle - PID_Roll - PID_Pitch - PID_Yaw;
                float m3 = throttle - PID_Roll + PID_Pitch + PID_Yaw;
                float m4 = throttle + PID_Roll + PID_Pitch - PID_Yaw;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)CLAMP(m1));
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)CLAMP(m2));
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)CLAMP(m3));
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)CLAMP(m4));
            }
        }

        // 9. Telemetry to Jetson every 100ms
        static uint8_t telem_ctr = 0;
        if (++telem_ctr >= 10) {
            Protocol_SendAttitude(Roll, Pitch, Yaw, GPS_Alt);
            Protocol_SendGPS(GPS_Lat, GPS_Lon, 0.0f);
            Protocol_SendBattery(battery_voltage, 0.0f);
            Protocol_SendNavStatus((uint8_t)nav_state, mission.current_idx);
            telem_ctr = 0;
        }

        HAL_Delay(10);
        /* USER CODE END WHILE */
    }
}

// ════════════════════════════════════════════════════════════════════════
// UART INTERRUPT CALLBACK
// ════════════════════════════════════════════════════════════════════════
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (gps_rx_data != '\n' && gps_index < 99)
            gps_buffer[gps_index++] = gps_rx_data;
        else {
            gps_buffer[gps_index] = '\0';
            gps_index = 0;
            gps_sentence_ready = 1;
        }
        HAL_UART_Receive_IT(&huart1, &gps_rx_data, 1);
    }
    if (huart->Instance == USART3) {
        Protocol_ParseByte(vision_rx_data);
        HAL_UART_Receive_IT(&huart3, &vision_rx_data, 1);
    }
}

// ════════════════════════════════════════════════════════════════════════
// PERIPHERAL INIT (CubeMX generated)
// ════════════════════════════════════════════════════════════════════════
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8; RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2; RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)   Error_Handler();
    if (HAL_PWREx_EnableOverDrive()            != HAL_OK)   Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        Error_Handler();
}
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    sConfig.Channel      = ADC_CHANNEL_0;  // PA0 — battery voltage divider
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
static void MX_I2C1_Init(void) {
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}
static void MX_TIM2_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 90 - 1;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 20000 - 1;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_MspPostInit(&htim2);
}
static void MX_UART4_Init(void) {
    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = 115200;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = UART_STOPBITS_1;
    huart4.Init.Parity       = UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();
}
static void MX_USART1_UART_Init(void) {
    huart1.Instance        = USART1;
    huart1.Init.BaudRate   = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits   = UART_STOPBITS_1;
    huart1.Init.Parity     = UART_PARITY_NONE;
    huart1.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);
}
static void MX_USART2_UART_Init(void) {
    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&huart2);
}
static void MX_USART3_UART_Init(void) {
    huart3.Instance        = USART3;
    huart3.Init.BaudRate   = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits   = UART_STOPBITS_1;
    huart3.Init.Parity     = UART_PARITY_NONE;
    huart3.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&huart3);
}
static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
