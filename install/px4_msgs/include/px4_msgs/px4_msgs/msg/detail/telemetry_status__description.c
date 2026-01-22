// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TelemetryStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/telemetry_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TelemetryStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x49, 0x9c, 0x3a, 0xca, 0x7b, 0x08, 0x4d, 0x83,
      0xb7, 0xd5, 0xbb, 0x8b, 0xa9, 0xc2, 0xb1, 0x75,
      0xf5, 0x42, 0x10, 0x44, 0x65, 0xbd, 0x4d, 0x3e,
      0xb4, 0x0d, 0xd8, 0x21, 0xb4, 0x9f, 0xde, 0x23,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TelemetryStatus__TYPE_NAME[] = "px4_msgs/msg/TelemetryStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__type[] = "type";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__mode[] = "mode";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__flow_control[] = "flow_control";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__forwarding[] = "forwarding";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__mavlink_v2[] = "mavlink_v2";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__ftp[] = "ftp";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__streams[] = "streams";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__data_rate[] = "data_rate";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rate_multiplier[] = "rate_multiplier";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_rate_avg[] = "tx_rate_avg";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_error_rate_avg[] = "tx_error_rate_avg";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_message_count[] = "tx_message_count";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_buffer_overruns[] = "tx_buffer_overruns";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_rate_avg[] = "rx_rate_avg";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_message_count[] = "rx_message_count";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_message_lost_count[] = "rx_message_lost_count";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_buffer_overruns[] = "rx_buffer_overruns";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_parse_errors[] = "rx_parse_errors";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_packet_drop_count[] = "rx_packet_drop_count";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_message_lost_rate[] = "rx_message_lost_rate";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_antenna_tracker[] = "heartbeat_type_antenna_tracker";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_gcs[] = "heartbeat_type_gcs";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_onboard_controller[] = "heartbeat_type_onboard_controller";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_gimbal[] = "heartbeat_type_gimbal";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_adsb[] = "heartbeat_type_adsb";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_camera[] = "heartbeat_type_camera";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_parachute[] = "heartbeat_type_parachute";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_open_drone_id[] = "heartbeat_type_open_drone_id";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_telemetry_radio[] = "heartbeat_component_telemetry_radio";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_log[] = "heartbeat_component_log";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_osd[] = "heartbeat_component_osd";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_vio[] = "heartbeat_component_vio";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_pairing_manager[] = "heartbeat_component_pairing_manager";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_udp_bridge[] = "heartbeat_component_udp_bridge";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_uart_bridge[] = "heartbeat_component_uart_bridge";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__open_drone_id_system_healthy[] = "open_drone_id_system_healthy";
static char px4_msgs__msg__TelemetryStatus__FIELD_NAME__parachute_system_healthy[] = "parachute_system_healthy";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TelemetryStatus__FIELDS[] = {
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__flow_control, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__forwarding, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__mavlink_v2, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__ftp, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__streams, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__data_rate, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rate_multiplier, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_rate_avg, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_error_rate_avg, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_message_count, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__tx_buffer_overruns, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_rate_avg, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_message_count, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_message_lost_count, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_buffer_overruns, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_parse_errors, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_packet_drop_count, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__rx_message_lost_rate, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_antenna_tracker, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_gcs, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_onboard_controller, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_gimbal, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_adsb, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_camera, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_parachute, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_type_open_drone_id, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_telemetry_radio, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_log, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_osd, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_vio, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_pairing_manager, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_udp_bridge, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__heartbeat_component_uart_bridge, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__open_drone_id_system_healthy, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TelemetryStatus__FIELD_NAME__parachute_system_healthy, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TelemetryStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TelemetryStatus__TYPE_NAME, 28, 28},
      {px4_msgs__msg__TelemetryStatus__FIELDS, 38, 38},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 LINK_TYPE_GENERIC = 0\n"
  "uint8 LINK_TYPE_UBIQUITY_BULLET = 1\n"
  "uint8 LINK_TYPE_WIRE = 2\n"
  "uint8 LINK_TYPE_USB = 3\n"
  "uint8 LINK_TYPE_IRIDIUM\\t= 4\n"
  "\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 type\\t\\t\\t\\t#  type of the radio hardware (LINK_TYPE_*)\n"
  "\n"
  "uint8 mode\n"
  "\n"
  "bool flow_control\n"
  "bool forwarding\n"
  "bool mavlink_v2\n"
  "bool ftp\n"
  "\n"
  "uint8 streams\n"
  "\n"
  "float32 data_rate                       # configured maximum data rate (Bytes/s)\n"
  "\n"
  "float32 rate_multiplier\n"
  "\n"
  "float32 tx_rate_avg                     # transmit rate average (Bytes/s)\n"
  "float32 tx_error_rate_avg               # transmit error rate average (Bytes/s)\n"
  "uint32 tx_message_count                 # total message sent count\n"
  "uint32 tx_buffer_overruns               # number of TX buffer overruns\n"
  "\n"
  "float32 rx_rate_avg                     # transmit rate average (Bytes/s)\n"
  "uint32 rx_message_count                 # count of total messages received\n"
  "uint32 rx_message_lost_count\n"
  "uint32 rx_buffer_overruns               # number of RX buffer overruns\n"
  "uint32 rx_parse_errors                  # number of parse errors\n"
  "uint32 rx_packet_drop_count             # number of packet drops\n"
  "float32 rx_message_lost_rate\n"
  "\n"
  "\n"
  "uint64 HEARTBEAT_TIMEOUT_US = 2500000       # Heartbeat timeout (tolerate missing 1 + jitter)\n"
  "\n"
  "# Heartbeats per type\n"
  "bool heartbeat_type_antenna_tracker         # MAV_TYPE_ANTENNA_TRACKER\n"
  "bool heartbeat_type_gcs                     # MAV_TYPE_GCS\n"
  "bool heartbeat_type_onboard_controller      # MAV_TYPE_ONBOARD_CONTROLLER\n"
  "bool heartbeat_type_gimbal                  # MAV_TYPE_GIMBAL\n"
  "bool heartbeat_type_adsb                    # MAV_TYPE_ADSB\n"
  "bool heartbeat_type_camera                  # MAV_TYPE_CAMERA\n"
  "bool heartbeat_type_parachute               # MAV_TYPE_PARACHUTE\n"
  "bool heartbeat_type_open_drone_id           # MAV_TYPE_ODID\n"
  "\n"
  "# Heartbeats per component\n"
  "bool heartbeat_component_telemetry_radio    # MAV_COMP_ID_TELEMETRY_RADIO\n"
  "bool heartbeat_component_log                # MAV_COMP_ID_LOG\n"
  "bool heartbeat_component_osd                # MAV_COMP_ID_OSD\n"
  "bool heartbeat_component_vio                # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY\n"
  "bool heartbeat_component_pairing_manager    # MAV_COMP_ID_PAIRING_MANAGER\n"
  "bool heartbeat_component_udp_bridge         # MAV_COMP_ID_UDP_BRIDGE\n"
  "bool heartbeat_component_uart_bridge        # MAV_COMP_ID_UART_BRIDGE\n"
  "\n"
  "# Misc component health\n"
  "bool open_drone_id_system_healthy\n"
  "bool parachute_system_healthy";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TelemetryStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TelemetryStatus__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2400, 2400},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TelemetryStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TelemetryStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
