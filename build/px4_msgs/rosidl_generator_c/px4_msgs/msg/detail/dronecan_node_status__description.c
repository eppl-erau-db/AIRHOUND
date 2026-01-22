// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DronecanNodeStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/dronecan_node_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DronecanNodeStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0x32, 0xb0, 0xe2, 0x21, 0x3b, 0x49, 0x80,
      0xdd, 0x86, 0x6c, 0xa9, 0xa7, 0xaa, 0x2a, 0x46,
      0x1e, 0x52, 0x92, 0x65, 0xc5, 0x91, 0xef, 0x32,
      0x2d, 0x23, 0xa6, 0xfe, 0xb9, 0xbc, 0xe6, 0x90,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DronecanNodeStatus__TYPE_NAME[] = "px4_msgs/msg/DronecanNodeStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__node_id[] = "node_id";
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__uptime_sec[] = "uptime_sec";
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__health[] = "health";
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__mode[] = "mode";
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__sub_mode[] = "sub_mode";
static char px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__vendor_specific_status_code[] = "vendor_specific_status_code";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DronecanNodeStatus__FIELDS[] = {
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__node_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__uptime_sec, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__health, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__sub_mode, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DronecanNodeStatus__FIELD_NAME__vendor_specific_status_code, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__DronecanNodeStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DronecanNodeStatus__TYPE_NAME, 31, 31},
      {px4_msgs__msg__DronecanNodeStatus__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint16 node_id\\t\\t\\t\\t\\t\\t# The node ID which this data comes from\n"
  "\n"
  "# From the uavcan.protocol.NodeStatus message\n"
  "uint32 uptime_sec\\t\\t\\t\\t\\t# Node uptime\n"
  "\n"
  "#\n"
  "# Abstract node health.\n"
  "#\n"
  "uint8 HEALTH_OK         = 0     # The node is functioning properly.\n"
  "uint8 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.\n"
  "uint8 HEALTH_ERROR      = 2     # The node encountered a major failure.\n"
  "uint8 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.\n"
  "uint8 health\n"
  "\n"
  "#\n"
  "# Current mode.\n"
  "#\n"
  "# Mode OFFLINE can be actually reported by the node to explicitly inform other network\n"
  "# participants that the sending node is about to shutdown. In this case other nodes will not\n"
  "# have to wait OFFLINE_TIMEOUT_MS before they detect that the node is no longer available.\n"
  "#\n"
  "# Reserved values can be used in future revisions of the specification.\n"
  "#\n"
  "uint8 MODE_OPERATIONAL      = 0         # Normal operating mode.\n"
  "uint8 MODE_INITIALIZATION   = 1         # Initialization is in progress; this mode is entered immediately after startup.\n"
  "uint8 MODE_MAINTENANCE      = 2         # E.g. calibration, the bootloader is running, etc.\n"
  "uint8 MODE_SOFTWARE_UPDATE  = 3         # New software/firmware is being loaded.\n"
  "uint8 MODE_OFFLINE          = 7         # The node is no longer available.\n"
  "uint8 mode\n"
  "\n"
  "#\n"
  "# Not used currently, keep zero when publishing, ignore when receiving.\n"
  "#\n"
  "uint8 sub_mode\n"
  "\n"
  "#\n"
  "# Optional, vendor-specific node status code, e.g. a fault code or a status bitmask.\n"
  "#\n"
  "uint16 vendor_specific_status_code";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DronecanNodeStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DronecanNodeStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1606, 1606},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DronecanNodeStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DronecanNodeStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
