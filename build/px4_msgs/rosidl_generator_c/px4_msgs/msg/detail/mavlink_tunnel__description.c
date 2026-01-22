// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/MavlinkTunnel.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/mavlink_tunnel__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__MavlinkTunnel__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x87, 0x5d, 0xc0, 0x0a, 0x96, 0x95, 0x8b, 0xb0,
      0x01, 0xb6, 0x6a, 0x6c, 0x4d, 0x91, 0xe1, 0x03,
      0xa0, 0xa0, 0x7c, 0xf2, 0x1a, 0x43, 0x99, 0xe2,
      0x90, 0xa8, 0x7f, 0x45, 0x43, 0x31, 0xf4, 0x91,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__MavlinkTunnel__TYPE_NAME[] = "px4_msgs/msg/MavlinkTunnel";

// Define type names, field names, and default values
static char px4_msgs__msg__MavlinkTunnel__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__MavlinkTunnel__FIELD_NAME__payload_type[] = "payload_type";
static char px4_msgs__msg__MavlinkTunnel__FIELD_NAME__target_system[] = "target_system";
static char px4_msgs__msg__MavlinkTunnel__FIELD_NAME__target_component[] = "target_component";
static char px4_msgs__msg__MavlinkTunnel__FIELD_NAME__payload_length[] = "payload_length";
static char px4_msgs__msg__MavlinkTunnel__FIELD_NAME__payload[] = "payload";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__MavlinkTunnel__FIELDS[] = {
  {
    {px4_msgs__msg__MavlinkTunnel__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkTunnel__FIELD_NAME__payload_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkTunnel__FIELD_NAME__target_system, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkTunnel__FIELD_NAME__target_component, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkTunnel__FIELD_NAME__payload_length, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkTunnel__FIELD_NAME__payload, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      128,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__MavlinkTunnel__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__MavlinkTunnel__TYPE_NAME, 26, 26},
      {px4_msgs__msg__MavlinkTunnel__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# MAV_TUNNEL_PAYLOAD_TYPE enum\n"
  "\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN = 0                # Encoding of payload unknown\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208    # Registered for STorM32 gimbal controller\n"
  "uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209    # Registered for STorM32 gimbal controller\n"
  "\n"
  "uint64 timestamp\\t     # Time since system start (microseconds)\n"
  "uint16 payload_type      # A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.\n"
  "uint8 target_system      # System ID (can be 0 for broadcast, but this is discouraged)\n"
  "uint8 target_component   # Component ID (can be 0 for broadcast, but this is discouraged)\n"
  "uint8 payload_length     # Length of the data transported in payload\n"
  "uint8[128] payload       # Data itself\n"
  "\n"
  "# Topic aliases for known payload types\n"
  "# TOPICS mavlink_tunnel esc_serial_passthru";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__MavlinkTunnel__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__MavlinkTunnel__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1997, 1997},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__MavlinkTunnel__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__MavlinkTunnel__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
