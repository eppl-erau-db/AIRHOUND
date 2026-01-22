// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FailsafeFlags.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/failsafe_flags__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FailsafeFlags__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0xca, 0xb5, 0x2d, 0x17, 0x26, 0x10, 0xc3,
      0x68, 0xdf, 0xfc, 0xd0, 0xeb, 0x2c, 0x37, 0x2d,
      0xd5, 0xe6, 0x4c, 0xa1, 0xb4, 0x9b, 0x02, 0xaf,
      0xcd, 0xef, 0x19, 0xe2, 0xc4, 0x94, 0xe6, 0x9d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FailsafeFlags__TYPE_NAME[] = "px4_msgs/msg/FailsafeFlags";

// Define type names, field names, and default values
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_angular_velocity[] = "mode_req_angular_velocity";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_attitude[] = "mode_req_attitude";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_local_alt[] = "mode_req_local_alt";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_local_position[] = "mode_req_local_position";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_local_position_relaxed[] = "mode_req_local_position_relaxed";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_global_position[] = "mode_req_global_position";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_global_position_relaxed[] = "mode_req_global_position_relaxed";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_mission[] = "mode_req_mission";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_offboard_signal[] = "mode_req_offboard_signal";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_home_position[] = "mode_req_home_position";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_wind_and_flight_time_compliance[] = "mode_req_wind_and_flight_time_compliance";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_prevent_arming[] = "mode_req_prevent_arming";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_manual_control[] = "mode_req_manual_control";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_other[] = "mode_req_other";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__angular_velocity_invalid[] = "angular_velocity_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__attitude_invalid[] = "attitude_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_altitude_invalid[] = "local_altitude_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_position_invalid[] = "local_position_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_position_invalid_relaxed[] = "local_position_invalid_relaxed";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_velocity_invalid[] = "local_velocity_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__global_position_invalid[] = "global_position_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__global_position_invalid_relaxed[] = "global_position_invalid_relaxed";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__auto_mission_missing[] = "auto_mission_missing";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__offboard_control_signal_lost[] = "offboard_control_signal_lost";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__home_position_invalid[] = "home_position_invalid";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__manual_control_signal_lost[] = "manual_control_signal_lost";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__gcs_connection_lost[] = "gcs_connection_lost";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__battery_warning[] = "battery_warning";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__battery_low_remaining_time[] = "battery_low_remaining_time";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__battery_unhealthy[] = "battery_unhealthy";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__geofence_breached[] = "geofence_breached";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__mission_failure[] = "mission_failure";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__vtol_fixed_wing_system_failure[] = "vtol_fixed_wing_system_failure";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__wind_limit_exceeded[] = "wind_limit_exceeded";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__flight_time_limit_exceeded[] = "flight_time_limit_exceeded";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__position_accuracy_low[] = "position_accuracy_low";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__navigator_failure[] = "navigator_failure";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_critical_failure[] = "fd_critical_failure";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_esc_arming_failure[] = "fd_esc_arming_failure";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_imbalanced_prop[] = "fd_imbalanced_prop";
static char px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_motor_failure[] = "fd_motor_failure";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FailsafeFlags__FIELDS[] = {
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_angular_velocity, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_attitude, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_local_alt, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_local_position, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_local_position_relaxed, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_global_position, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_global_position_relaxed, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_mission, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_offboard_signal, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_home_position, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_wind_and_flight_time_compliance, 40, 40},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_prevent_arming, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_manual_control, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mode_req_other, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__angular_velocity_invalid, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__attitude_invalid, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_altitude_invalid, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_position_invalid, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_position_invalid_relaxed, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__local_velocity_invalid, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__global_position_invalid, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__global_position_invalid_relaxed, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__auto_mission_missing, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__offboard_control_signal_lost, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__home_position_invalid, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__manual_control_signal_lost, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__gcs_connection_lost, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__battery_warning, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__battery_low_remaining_time, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__battery_unhealthy, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__geofence_breached, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__mission_failure, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__vtol_fixed_wing_system_failure, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__wind_limit_exceeded, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__flight_time_limit_exceeded, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__position_accuracy_low, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__navigator_failure, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_critical_failure, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_esc_arming_failure, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_imbalanced_prop, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FailsafeFlags__FIELD_NAME__fd_motor_failure, 16, 16},
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
px4_msgs__msg__FailsafeFlags__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FailsafeFlags__TYPE_NAME, 26, 26},
      {px4_msgs__msg__FailsafeFlags__FIELDS, 42, 42},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Input flags for the failsafe state machine set by the arming & health checks.\n"
  "#\n"
  "# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)\n"
  "# The flag comments are used as label for the failsafe state machine simulation\n"
  "\n"
  "uint64 timestamp\\t\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "# Per-mode requirements\n"
  "uint32 mode_req_angular_velocity\n"
  "uint32 mode_req_attitude\n"
  "uint32 mode_req_local_alt\n"
  "uint32 mode_req_local_position\n"
  "uint32 mode_req_local_position_relaxed\n"
  "uint32 mode_req_global_position\n"
  "uint32 mode_req_global_position_relaxed\n"
  "uint32 mode_req_mission\n"
  "uint32 mode_req_offboard_signal\n"
  "uint32 mode_req_home_position\n"
  "uint32 mode_req_wind_and_flight_time_compliance # if set, mode cannot be entered if wind or flight time limit exceeded\n"
  "uint32 mode_req_prevent_arming    # if set, cannot arm while in this mode\n"
  "uint32 mode_req_manual_control\n"
  "uint32 mode_req_other             # other requirements, not covered above (for external modes)\n"
  "\n"
  "\n"
  "# Mode requirements\n"
  "bool angular_velocity_invalid         # Angular velocity invalid\n"
  "bool attitude_invalid                 # Attitude invalid\n"
  "bool local_altitude_invalid           # Local altitude invalid\n"
  "bool local_position_invalid           # Local position estimate invalid\n"
  "bool local_position_invalid_relaxed   # Local position with reduced accuracy requirements invalid (e.g. flying with optical flow)\n"
  "bool local_velocity_invalid           # Local velocity estimate invalid\n"
  "bool global_position_invalid          # Global position estimate invalid\n"
  "bool global_position_invalid_relaxed  # Global position estimate invalid with relaxed accuracy requirements\n"
  "bool auto_mission_missing             # No mission available\n"
  "bool offboard_control_signal_lost     # Offboard signal lost\n"
  "bool home_position_invalid            # No home position available\n"
  "\n"
  "# Control links\n"
  "bool manual_control_signal_lost       # Manual control (RC) signal lost\n"
  "bool gcs_connection_lost              # GCS connection lost\n"
  "\n"
  "# Battery\n"
  "uint8 battery_warning                 # Battery warning level (see BatteryStatus.msg)\n"
  "bool battery_low_remaining_time       # Low battery based on remaining flight time\n"
  "bool battery_unhealthy                # Battery unhealthy\n"
  "\n"
  "# Other\n"
  "bool geofence_breached        \\t      # Geofence breached (one or multiple)\n"
  "bool mission_failure                  # Mission failure\n"
  "bool vtol_fixed_wing_system_failure   # vehicle in fixed-wing system failure failsafe mode (after quad-chute)\n"
  "bool wind_limit_exceeded              # Wind limit exceeded\n"
  "bool flight_time_limit_exceeded       # Maximum flight time exceeded\n"
  "bool position_accuracy_low            # Position estimate has dropped below threshold, but is currently still declared valid\n"
  "bool navigator_failure        \\t      # Navigator failed to execute a mode\n"
  "\n"
  "# Failure detector\n"
  "bool fd_critical_failure              # Critical failure (attitude/altitude limit exceeded, or external ATS)\n"
  "bool fd_esc_arming_failure            # ESC failed to arm\n"
  "bool fd_imbalanced_prop               # Imbalanced propeller detected\n"
  "bool fd_motor_failure                 # Motor failure";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FailsafeFlags__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FailsafeFlags__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 3094, 3094},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FailsafeFlags__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FailsafeFlags__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
