#ifndef RC_PROTOCOL_H
#define RC_PROTOCOL_H

/*
 * FalconDrone radio link — packet format shared by the ESP32 controller and
 * the STM32 flight controller. Both firmwares include this one file, so the
 * format cannot drift between them.
 *
 *   Controller ──RcPacket (50 Hz)──▶ Drone
 *   Controller ◀──RcAck (one per packet)── Drone
 *
 * Two interchangeable transports, picked at build time on each side:
 *
 *   ESP-NOW (default)  controller ESP32 ⇄ bridge ESP32 on the drone ⇄ UART
 *                      ⇄ STM32. Packets travel inside rc_frame.h frames.
 *   nRF24              controller nRF24 (PTX) ⇄ drone nRF24 (PRX on SPI2).
 *                      The drone answers inside the auto-ACK, so status
 *                      comes back without a second radio exchange.
 *
 * Stick values are normalized (deadband and center calibration already
 * applied on the controller). The drone decides what full stick means
 * (expo, angle limit) — see design doc §7.
 */

#include <stdint.h>

/* ---- Timing (both transports) -------------------------------------------- */
#define RC_PERIOD_MS        20u     /* controller sends at 50 Hz               */
#define RC_LINK_TIMEOUT_MS  250u    /* drone: no packet this long → disarm     */

/* ---- ESP-NOW + bridge settings (all three boards must match) ------------- */
#define RC_WIFI_CHANNEL     1u      /* 2.412 GHz                                */
#define RC_BRIDGE_BAUD      115200u /* bridge ⇄ STM32 UART                      */

/* ---- nRF24 settings (both ends must match) ------------------------------- */
#define RC_CHANNEL          108u    /* 2.508 GHz — above Wi-Fi channels 1-13   */
#define RC_DATA_RATE_KBPS   1000u   /* 1 Mbps; some clone chips lack 250 kbps  */
#define RC_ADDRESS          { 0x46, 0x41, 0x4C, 0x43, 0x4E }  /* "FALCN"      */
#define RC_RETRIES          5u      /* auto-retransmits per packet             */
#define RC_RETRY_DELAY_US   500u    /* >= 500 us needed for ACK payloads @1Mbps */

/* ---- Controller → drone ------------------------------------------------- */
#define RC_THROTTLE_MAX     1000
#define RC_AXIS_MAX         1000

/* Switch levels, sent in every packet. The drone arms on an ARM OFF→ON flip
 * with the throttle at or below RC_THROTTLE_ARM_MAX, and disarms the moment
 * ARM reads OFF. MODE is applied only while disarmed. */
#define RC_FLAG_ARM         (1u << 0)   /* arm switch ON                       */
#define RC_FLAG_ANGLE       (1u << 1)   /* mode switch: ANGLE (else RATE)      */
#define RC_FLAG_BUTTON      (1u << 2)   /* stick button held — no action yet   */

#define RC_THROTTLE_ARM_MAX 50

typedef struct __attribute__((packed)) {
    uint8_t  seq;        /* increments every packet — detects lost packets  */
    uint8_t  flags;      /* RC_FLAG_*                                       */
    uint16_t throttle;   /* 0 .. RC_THROTTLE_MAX                            */
    int16_t  roll;       /* -RC_AXIS_MAX .. +RC_AXIS_MAX, + = right         */
    int16_t  pitch;      /* -RC_AXIS_MAX .. +RC_AXIS_MAX, + = nose up       */
    int16_t  yaw;        /* 0 until a second stick exists                   */
} RcPacket;

/* ---- Drone → controller -------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  seq;        /* seq of the last RcPacket the drone received     */
    uint8_t  arm_state;  /* 0 disarmed, 1 armed, 2 arming (ArmState_t)       */
    uint8_t  mode;       /* 0 rate, 1 angle                                 */
    uint8_t  link_pct;   /* packets received over the last second, %        */
    uint16_t throttle_us;/* throttle the drone is actually commanding       */
} RcAck;

/* nRF24 payloads are at most 32 bytes; rc_frame.h payloads 16. */
_Static_assert(sizeof(RcPacket) == 10, "RcPacket layout changed");
_Static_assert(sizeof(RcAck) == 6, "RcAck layout changed");

#endif /* RC_PROTOCOL_H */
