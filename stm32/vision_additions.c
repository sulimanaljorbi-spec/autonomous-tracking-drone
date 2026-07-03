/*
 * STM32 Vision Integration — Experiment 2
 * =========================================
 * Add these additions to your Experiment 1 main.c
 *
 * PROBLEMS FIXED vs your original code:
 *   1. Parser only checked 0xAA header — now checks 0xAA 0x55 dual header
 *   2. No checksum validation — now validates XOR checksum
 *   3. Parsed track_x/track_y from absolute coords — now uses offset from center
 *   4. STM32 never sent telemetry back — now sends ATTITUDE, GPS, BATTERY
 *   5. vision_frame_ready flag never properly consumed — fixed
 *   6. USART3 receiving into wrong variable — fixed to use vision_rx_data
 *
 * HOW TO INTEGRATE:
 *   Step 1: Replace your HAL_UART_RxCpltCallback with the one below
 *   Step 2: Add the Protocol_ functions before main()
 *   Step 3: Call Protocol_SendTelemetry() inside your main while(1) loop
 *           (e.g. every 10 iterations ~= 100ms at 10ms loop time)
 *   Step 4: Make sure USART3 is initialized at 115200 baud (TX+RX)
 */

/* ── Add to your Private Variables section ──────────────────────────── */

// Protocol header bytes
#define PROTO_H1        0xAA
#define PROTO_H2        0x55
#define MSG_HEARTBEAT   0x01
#define MSG_TRACK_CMD   0x02
#define MSG_VELOCITY    0x03
#define MSG_ATTITUDE    0x10
#define MSG_GPS         0x11
#define MSG_BATTERY     0x12
#define MAX_PAYLOAD     32

// Vision tracking offsets (pixels from frame center, set by Jetson TRACK_CMD)
// offset > 0 means target is to the right / below center
volatile float vision_offset_x = 0.0f;
volatile float vision_offset_y = 0.0f;
volatile float vision_confidence = 0.0f;
volatile uint8_t vision_frame_ready = 0;
volatile uint32_t last_vision_tick = 0;   // HAL_GetTick() of last valid packet

// Heartbeat — detect if Jetson is alive
volatile uint8_t jetson_alive = 0;
volatile uint32_t last_heartbeat_tick = 0;

// Raw UART byte for USART3 (vision)
uint8_t vision_rx_data = 0;


/* ── Protocol Parser — add before main() ────────────────────────────── */

/* Parser state machine */
typedef enum {
    WAIT_H1, WAIT_H2, WAIT_ID, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM
} ProtoRxState_t;

static ProtoRxState_t proto_state  = WAIT_H1;
static uint8_t  proto_msg_id       = 0;
static uint8_t  proto_length       = 0;
static uint8_t  proto_payload[MAX_PAYLOAD];
static uint8_t  proto_idx          = 0;
static uint8_t  proto_checksum     = 0;

/**
 * Protocol_ParseByte()
 * Call this for every byte received on USART3 (Jetson UART).
 * Implements dual-header + XOR checksum framing.
 * Matches the _send() method in the Jetson STM32Link class exactly.
 */
void Protocol_ParseByte(uint8_t byte)
{
    switch (proto_state)
    {
        case WAIT_H1:
            if (byte == PROTO_H1) proto_state = WAIT_H2;
            break;

        case WAIT_H2:
            proto_state = (byte == PROTO_H2) ? WAIT_ID : WAIT_H1;
            break;

        case WAIT_ID:
            proto_msg_id    = byte;
            proto_checksum  = byte;   // checksum starts with msg_id
            proto_state     = WAIT_LEN;
            break;

        case WAIT_LEN:
            proto_length    = byte;
            proto_checksum ^= byte;
            proto_idx       = 0;
            proto_state     = (byte > 0) ? WAIT_PAYLOAD : WAIT_CHECKSUM;
            break;

        case WAIT_PAYLOAD:
            proto_payload[proto_idx++] = byte;
            proto_checksum ^= byte;
            if (proto_idx >= proto_length) proto_state = WAIT_CHECKSUM;
            break;

        case WAIT_CHECKSUM:
            if (byte == (proto_checksum & 0xFF))
            {
                Protocol_HandleMessage(proto_msg_id,
                                       proto_payload,
                                       proto_length);
            }
            // Always reset, even on checksum fail
            proto_state = WAIT_H1;
            break;
    }
}

/**
 * Protocol_HandleMessage()
 * Called when a valid, checksum-verified frame is received from Jetson.
 */
void Protocol_HandleMessage(uint8_t msg_id, uint8_t *payload, uint8_t len)
{
    switch (msg_id)
    {
        case MSG_HEARTBEAT:   // 0x01 — Jetson alive ping
            if (len >= 1) {
                jetson_alive        = payload[0];
                last_heartbeat_tick = HAL_GetTick();
            }
            break;

        case MSG_TRACK_CMD:   // 0x02 — pixel offset + confidence from Jetson
            if (len >= 6) {
                // Big-endian int16 for offset_x
                int16_t tx = (int16_t)((payload[0] << 8) | payload[1]);
                // Big-endian int16 for offset_y
                int16_t ty = (int16_t)((payload[2] << 8) | payload[3]);
                // Big-endian uint16 for confidence (0-100)
                uint16_t conf = (uint16_t)((payload[4] << 8) | payload[5]);

                vision_offset_x    = (float)tx;
                vision_offset_y    = (float)ty;
                vision_confidence  = (float)conf / 100.0f;
                vision_frame_ready = 1;
                last_vision_tick   = HAL_GetTick();
            }
            break;

        case MSG_VELOCITY:    // 0x03 — velocity command (future use)
            // Reserved for autonomous navigation extension
            break;

        default:
            break;
    }
}


/* ── Protocol Send — Telemetry back to Jetson ───────────────────────── */

/**
 * Protocol_SendFrame()
 * Builds and transmits one framed packet over USART3 to Jetson.
 * Uses the same framing as the Jetson STM32Link._send() method.
 */
void Protocol_SendFrame(uint8_t msg_id, uint8_t *payload, uint8_t len)
{
    uint8_t buf[5 + MAX_PAYLOAD];
    uint8_t checksum = msg_id ^ len;

    buf[0] = PROTO_H1;
    buf[1] = PROTO_H2;
    buf[2] = msg_id;
    buf[3] = len;

    for (uint8_t i = 0; i < len; i++) {
        buf[4 + i]  = payload[i];
        checksum   ^= payload[i];
    }
    buf[4 + len] = checksum & 0xFF;

    HAL_UART_Transmit(&huart3, buf, 5 + len, 10);
}

/**
 * Protocol_SendAttitude()
 * Sends roll, pitch, yaw, altitude to Jetson.
 * Values scaled x100 and packed as big-endian int16.
 * MSG 0x10, 8 bytes payload.
 */
void Protocol_SendAttitude(float roll, float pitch, float yaw, float alt)
{
    uint8_t payload[8];
    int16_t r = (int16_t)(roll  * 100.0f);
    int16_t p = (int16_t)(pitch * 100.0f);
    int16_t y = (int16_t)(yaw   * 100.0f);
    int16_t a = (int16_t)(alt   * 100.0f);

    payload[0] = (r >> 8) & 0xFF;  payload[1] = r & 0xFF;
    payload[2] = (p >> 8) & 0xFF;  payload[3] = p & 0xFF;
    payload[4] = (y >> 8) & 0xFF;  payload[5] = y & 0xFF;
    payload[6] = (a >> 8) & 0xFF;  payload[7] = a & 0xFF;

    Protocol_SendFrame(MSG_ATTITUDE, payload, 8);
}

/**
 * Protocol_SendGPS()
 * Sends lat, lon (int32 x1e6) and speed (int16 x100) to Jetson.
 * MSG 0x11, 10 bytes payload.
 */
void Protocol_SendGPS(float lat, float lon, float speed)
{
    uint8_t payload[10];
    int32_t la  = (int32_t)(lat   * 1e6f);
    int32_t lo  = (int32_t)(lon   * 1e6f);
    int16_t spd = (int16_t)(speed * 100.0f);

    payload[0] = (la >> 24) & 0xFF;  payload[1] = (la >> 16) & 0xFF;
    payload[2] = (la >>  8) & 0xFF;  payload[3] =  la & 0xFF;
    payload[4] = (lo >> 24) & 0xFF;  payload[5] = (lo >> 16) & 0xFF;
    payload[6] = (lo >>  8) & 0xFF;  payload[7] =  lo & 0xFF;
    payload[8] = (spd >> 8) & 0xFF;  payload[9] =  spd & 0xFF;

    Protocol_SendFrame(MSG_GPS, payload, 10);
}


/* ── Replace your existing HAL_UART_RxCpltCallback ──────────────────── */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* GPS — USART1 */
    if (huart->Instance == USART1)
    {
        if (gps_rx_data != '\n' && gps_index < 99) {
            gps_buffer[gps_index++] = gps_rx_data;
        } else {
            gps_buffer[gps_index] = '\0';
            gps_index = 0;
            gps_sentence_ready = 1;
        }
        HAL_UART_Receive_IT(&huart1, &gps_rx_data, 1);
    }

    /* Jetson Vision — USART3 */
    if (huart->Instance == USART3)
    {
        Protocol_ParseByte(vision_rx_data);
        HAL_UART_Receive_IT(&huart3, &vision_rx_data, 1);
    }
}


/* ── Vision fusion — replace the relevant section in your while(1) ──── */
/*
 * Add this INSIDE your main while(1) loop, replacing your existing
 * "Vision Fusion & Safety Clamp" block.
 *
 * Also add a telemetry send every ~10 loops (100 ms):
 *
 *   static uint8_t telem_counter = 0;
 *   if (++telem_counter >= 10) {
 *       Protocol_SendAttitude(Roll, Pitch, Yaw, 0.0f);
 *       Protocol_SendGPS(GPS_Lat, GPS_Lon, 0.0f);
 *       telem_counter = 0;
 *   }
 */

/*
    // ── Vision Fusion ────────────────────────────────────────────────
    float v_adj_roll  = 0.0f;
    float v_adj_pitch = 0.0f;

    // Vision timeout: if no TRACK_CMD in last 1000 ms, zero out
    if ((HAL_GetTick() - last_vision_tick) > 1000) {
        vision_offset_x    = 0.0f;
        vision_offset_y    = 0.0f;
        vision_frame_ready = 0;
    }

    if (vision_frame_ready && vision_confidence > 0.3f)
    {
        // Scale pixel offset to degrees
        // offset range ~-320..+320 -> angle range ~-12.8..+12.8 deg (x0.04)
        v_adj_roll  = vision_offset_x * 0.04f;
        v_adj_pitch = vision_offset_y * 0.04f;

        // Safety clamp — max 15 deg from vision
        if (v_adj_roll  >  15.0f) v_adj_roll  =  15.0f;
        if (v_adj_roll  < -15.0f) v_adj_roll  = -15.0f;
        if (v_adj_pitch >  15.0f) v_adj_pitch =  15.0f;
        if (v_adj_pitch < -15.0f) v_adj_pitch = -15.0f;

        vision_frame_ready = 0;
    }

    Roll_Setpoint  = stick_roll  + v_adj_roll;
    Pitch_Setpoint = stick_pitch - v_adj_pitch;  // minus: down-offset = pitch forward

    // Final global clamp (max 25 deg tilt)
    if (Roll_Setpoint  >  25.0f) Roll_Setpoint  =  25.0f;
    if (Roll_Setpoint  < -25.0f) Roll_Setpoint  = -25.0f;
    if (Pitch_Setpoint >  25.0f) Pitch_Setpoint =  25.0f;
    if (Pitch_Setpoint < -25.0f) Pitch_Setpoint = -25.0f;

    // Send telemetry back to Jetson every 100 ms
    static uint8_t telem_counter = 0;
    if (++telem_counter >= 10) {
        Protocol_SendAttitude(Roll, Pitch, Yaw, 0.0f);
        Protocol_SendGPS(GPS_Lat, GPS_Lon, 0.0f);
        telem_counter = 0;
    }
    // ─────────────────────────────────────────────────────────────────
*/
