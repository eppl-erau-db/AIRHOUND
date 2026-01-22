// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CellularStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/cellular_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CellularStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfa, 0xac, 0x66, 0xc9, 0x0d, 0xc6, 0x8d, 0x2c,
      0xc3, 0x5f, 0x92, 0x59, 0x01, 0xa8, 0x01, 0xd7,
      0x5d, 0xd7, 0x84, 0xa6, 0x4a, 0x3c, 0xbd, 0xdd,
      0xe0, 0x5f, 0x6a, 0x37, 0x1f, 0x36, 0x91, 0x3b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CellularStatus__TYPE_NAME[] = "px4_msgs/msg/CellularStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__CellularStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__status[] = "status";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__failure_reason[] = "failure_reason";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__type[] = "type";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__quality[] = "quality";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__mcc[] = "mcc";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__mnc[] = "mnc";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__lac[] = "lac";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CellularStatus__FIELDS[] = {
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__failure_reason, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__quality, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__mcc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__mnc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__lac, 3, 3},
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
px4_msgs__msg__CellularStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CellularStatus__TYPE_NAME, 27, 27},
      {px4_msgs__msg__CellularStatus__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Cellular status\n"
  "#\n"
  "# This is currently used only for logging cell status from MAVLink.\n"
  "\n"
  "uint64 timestamp  # [us] Time since system start\n"
  "\n"
  "uint16 status                            # [@enum STATUS_FLAG] Status bitmap\n"
  "uint16 STATUS_FLAG_UNKNOWN = 1           # State unknown or not reportable\n"
  "uint16 STATUS_FLAG_FAILED = 2            # Modem is unusable\n"
  "uint16 STATUS_FLAG_INITIALIZING = 4      # Modem is being initialized\n"
  "uint16 STATUS_FLAG_LOCKED = 8            # Modem is locked\n"
  "uint16 STATUS_FLAG_DISABLED = 16         # Modem is not enabled and is powered down\n"
  "uint16 STATUS_FLAG_DISABLING = 32        # Modem is currently transitioning to the STATUS_FLAG_DISABLED state\n"
  "uint16 STATUS_FLAG_ENABLING = 64         # Modem is currently transitioning to the STATUS_FLAG_ENABLED state\n"
  "uint16 STATUS_FLAG_ENABLED = 128         # Modem is enabled and powered on but not registered with a network provider and not available for data connections\n"
  "uint16 STATUS_FLAG_SEARCHING = 256       # Modem is searching for a network provider to register\n"
  "uint16 STATUS_FLAG_REGISTERED = 512      # Modem is registered with a network provider, and data connections and messaging may be available for use\n"
  "uint16 STATUS_FLAG_DISCONNECTING = 1024  # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated\n"
  "uint16 STATUS_FLAG_CONNECTING = 2048     # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered\n"
  "uint16 STATUS_FLAG_CONNECTED = 4096      # One or more packet data bearers is active and connected\n"
  "\n"
  "uint8 failure_reason                  # [@enum FAILURE_REASON] Failure reason\n"
  "uint8 FAILURE_REASON_NONE = 0         # No error\n"
  "uint8 FAILURE_REASON_UNKNOWN = 1      # Error state is unknown\n"
  "uint8 FAILURE_REASON_SIM_MISSING = 2  # SIM is required for the modem but missing\n"
  "uint8 FAILURE_REASON_SIM_ERROR = 3    # SIM is available, but not usable for connection\n"
  "\n"
  "uint8 type                                   # [@enum CELLULAR_NETWORK_RADIO_TYPE] Cellular network radio type\n"
  "uint8 CELLULAR_NETWORK_RADIO_TYPE_NONE = 0   # None\n"
  "uint8 CELLULAR_NETWORK_RADIO_TYPE_GSM = 1    # GSM\n"
  "uint8 CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2   # CDMA\n"
  "uint8 CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3  # WCDMA\n"
  "uint8 CELLULAR_NETWORK_RADIO_TYPE_LTE = 4    # LTE\n"
  "\n"
  "uint8 quality  # [dBm] Cellular network RSSI/RSRP, absolute value\n"
  "uint16 mcc     # [@invalid UINT16_MAX] Mobile country code\n"
  "uint16 mnc     # [@invalid UINT16_MAX] Mobile network code\n"
  "uint16 lac     # [@invalid 0] Location area code";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CellularStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CellularStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2706, 2706},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CellularStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CellularStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
