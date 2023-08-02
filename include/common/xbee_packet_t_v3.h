#include <stdint.h>

#ifndef __XBEE_PACKET_V3__
#define __XBEE_PACKET_V3__

// v3 XBee Packet
struct __attribute__((packed)) xbee_packet_t {
  uint32_t time;         ///< Unique id for the rigid body being described
  float x;               ///< x-position in the Optitrack frame
  float y;               ///< y-position in the Optitrack frame
  float z;               ///< z-position in the Optitrack frame
  float qx;              ///< qx of quaternion
  float qy;              ///< qy of quaternion
  float qz;              ///< qz of quaternion
  float qw;              ///< qw of quaternion
  int8_t trackingValid;  ///< (bool) of whether or not tracking was valid (0 or 1)
  int16_t state;         ///< state
  float x_d;             ///< Desired X Position
  float y_d;             ///< Desired Y Position
  float z_d;             ///< Desired Z Position
};

// 2 for START & STOP bytes
// 3 for START, STOP, and 8-bit CRC
// 4 for START, STOP, and 16-bit checksum (Flecther 16)
#define NUM_FRAMING_BYTES 4

// Actual Packet Being Sent
#define _MOCAP_DATA_LENGTH sizeof(xbee_packet_t)

// Num bytes in optitrack message (INCLUDING START, STOP and optional CRC)
#define _MOCAP_PACKET_LENGTH _MOCAP_DATA_LENGTH + NUM_FRAMING_BYTES

#endif  //__XBEE_PACKET_V3__