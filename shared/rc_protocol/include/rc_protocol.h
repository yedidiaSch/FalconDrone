#ifndef RC_PROTOCOL_H
#define RC_PROTOCOL_H

/*
 * FalconDrone radio link — packet format shared by the ESP32 controller and
 * the STM32 flight controller. Both firmwares include this one file, so the
 * format cannot drift between them.
 *
 *   Controller ──RcPacket (50 Hz)──▶ Drone
 *   Controller ◀──RcAck (ACK payload)── Drone
 *
 * The drone answers inside the nRF24 auto-ACK, so status comes back without
 * a second radio exchange or switching the controller to RX.
 *
 * Stick values are normalized (deadband and center calibration already
 * applied on the controller). The drone decides what full stick means
 * (expo, angle limit) — see design doc §7.
 */

#include <stdint.h>

/* ---- Radio settings (both ends must match) ------------------------------- */
#define RC_CHANNEL          108u    /* 2.508 GHz — above Wi-Fi channels 1-13   */
#define RC_DATA_RATE_KBPS   1000u   /* 1 Mbps; some clone chips lack 250 kbps  */
#define RC_ADDRESS          { 0x46, 0x41, 0x4C, 0x43, 0x4E }  /* "FALCN"      */
#define RC_PERIOD_MS        20u     /* controller sends at 50 Hz               */
#define RC_LINK_TIMEOUT_MS  250u    /* drone: no packet this long → disarm     */
#define RC_RETRIES          5u      /* auto-retransmits per packet             */
#define RC_RETRY_DELAY_US   500u    /* >= 500 us needed for ACK payloads @1Mbps */

/* ---- Controller → drone ------------------------------------------------- */
#define RC_THROTTLE_MAX     1000
#define RC_AXIS_MAX         1000

#define RC_FLAG_ARM_REQ     (1u << 0)   /* button held: request arm / disarm */

/* The drone refuses to arm unless throttle is at or below this. */
#define RC_THROTTLE_ARM_MAX 50

typedef struct __attribute__((packed)) {
    uint8_t  seq;        /* increments every packet — detects lost packets  */
    uint8_t  flags;      /* RC_FLAG_*                                       */
    uint16_t throttle;   /* 0 .. RC_THROTTLE_MAX                            */
    int16_t  roll;       /* -RC_AXIS_MAX .. +RC_AXIS_MAX, + = right         */
    int16_t  pitch;      /* -RC_AXIS_MAX .. +RC_AXIS_MAX, + = nose up       */
    int16_t  yaw;        /* 0 until a second stick exists                   */
} RcPacket;

/* ---- Drone → controller (ACK payload) ------------------------------------ */
typedef struct __attribute__((packed)) {
    uint8_t  seq;        /* seq of the last RcPacket the drone received     */
    uint8_t  arm_state;  /* 0 disarmed, 1 armed, 2 arming (ArmState_t)       */
    uint8_t  mode;       /* 0 rate, 1 angle                                 */
    uint8_t  link_pct;   /* packets received over the last second, %        */
    uint16_t throttle_us;/* throttle the drone is actually commanding       */
} RcAck;

/* nRF24 payloads are at most 32 bytes. */
_Static_assert(sizeof(RcPacket) == 10, "RcPacket layout changed");
_Static_assert(sizeof(RcAck) == 6, "RcAck layout changed");

#endif /* RC_PROTOCOL_H */
