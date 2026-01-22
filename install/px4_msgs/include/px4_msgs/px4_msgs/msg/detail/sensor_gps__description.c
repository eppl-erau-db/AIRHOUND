// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorGps.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_gps__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorGps__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0x0f, 0x92, 0xe6, 0x80, 0xbb, 0xca, 0xd0,
      0xc9, 0xd0, 0xe6, 0x51, 0x86, 0x8c, 0x04, 0x43,
      0xb9, 0xd3, 0x8f, 0xb8, 0xd5, 0x0c, 0x21, 0xd4,
      0xa8, 0xa5, 0x32, 0xd3, 0xe9, 0x0c, 0xf2, 0x1a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorGps__TYPE_NAME[] = "px4_msgs/msg/SensorGps";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorGps__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorGps__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorGps__FIELD_NAME__latitude_deg[] = "latitude_deg";
static char px4_msgs__msg__SensorGps__FIELD_NAME__longitude_deg[] = "longitude_deg";
static char px4_msgs__msg__SensorGps__FIELD_NAME__altitude_msl_m[] = "altitude_msl_m";
static char px4_msgs__msg__SensorGps__FIELD_NAME__altitude_ellipsoid_m[] = "altitude_ellipsoid_m";
static char px4_msgs__msg__SensorGps__FIELD_NAME__s_variance_m_s[] = "s_variance_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__c_variance_rad[] = "c_variance_rad";
static char px4_msgs__msg__SensorGps__FIELD_NAME__fix_type[] = "fix_type";
static char px4_msgs__msg__SensorGps__FIELD_NAME__eph[] = "eph";
static char px4_msgs__msg__SensorGps__FIELD_NAME__epv[] = "epv";
static char px4_msgs__msg__SensorGps__FIELD_NAME__hdop[] = "hdop";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vdop[] = "vdop";
static char px4_msgs__msg__SensorGps__FIELD_NAME__noise_per_ms[] = "noise_per_ms";
static char px4_msgs__msg__SensorGps__FIELD_NAME__automatic_gain_control[] = "automatic_gain_control";
static char px4_msgs__msg__SensorGps__FIELD_NAME__jamming_state[] = "jamming_state";
static char px4_msgs__msg__SensorGps__FIELD_NAME__jamming_indicator[] = "jamming_indicator";
static char px4_msgs__msg__SensorGps__FIELD_NAME__spoofing_state[] = "spoofing_state";
static char px4_msgs__msg__SensorGps__FIELD_NAME__authentication_state[] = "authentication_state";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_m_s[] = "vel_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_n_m_s[] = "vel_n_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_e_m_s[] = "vel_e_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_d_m_s[] = "vel_d_m_s";
static char px4_msgs__msg__SensorGps__FIELD_NAME__cog_rad[] = "cog_rad";
static char px4_msgs__msg__SensorGps__FIELD_NAME__vel_ned_valid[] = "vel_ned_valid";
static char px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_time_relative[] = "timestamp_time_relative";
static char px4_msgs__msg__SensorGps__FIELD_NAME__time_utc_usec[] = "time_utc_usec";
static char px4_msgs__msg__SensorGps__FIELD_NAME__satellites_used[] = "satellites_used";
static char px4_msgs__msg__SensorGps__FIELD_NAME__system_error[] = "system_error";
static char px4_msgs__msg__SensorGps__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__SensorGps__FIELD_NAME__heading_offset[] = "heading_offset";
static char px4_msgs__msg__SensorGps__FIELD_NAME__heading_accuracy[] = "heading_accuracy";
static char px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_injection_rate[] = "rtcm_injection_rate";
static char px4_msgs__msg__SensorGps__FIELD_NAME__selected_rtcm_instance[] = "selected_rtcm_instance";
static char px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_crc_failed[] = "rtcm_crc_failed";
static char px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_msg_used[] = "rtcm_msg_used";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorGps__FIELDS[] = {
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__latitude_deg, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__longitude_deg, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__altitude_msl_m, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__altitude_ellipsoid_m, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__s_variance_m_s, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__c_variance_rad, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__fix_type, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__eph, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__epv, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__hdop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vdop, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__noise_per_ms, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__automatic_gain_control, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__jamming_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__jamming_indicator, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__spoofing_state, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__authentication_state, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_m_s, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_n_m_s, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_e_m_s, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_d_m_s, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__cog_rad, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__vel_ned_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__timestamp_time_relative, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__time_utc_usec, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__satellites_used, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__system_error, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__heading_offset, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__heading_accuracy, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_injection_rate, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__selected_rtcm_instance, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_crc_failed, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGps__FIELD_NAME__rtcm_msg_used, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorGps__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorGps__TYPE_NAME, 22, 22},
      {px4_msgs__msg__SensorGps__FIELDS, 37, 37},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GPS position in WGS84 coordinates.\n"
  "# the field 'timestamp' is for the position & velocity (microseconds)\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint64 timestamp_sample\n"
  "\n"
  "uint32 device_id                # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "float64 latitude_deg\\t\\t# Latitude in degrees, allows centimeter level RTK precision\n"
  "float64 longitude_deg\\t\\t# Longitude in degrees, allows centimeter level RTK precision\n"
  "float64 altitude_msl_m\\t\\t# Altitude above MSL, meters\n"
  "float64 altitude_ellipsoid_m\\t# Altitude above Ellipsoid, meters\n"
  "\n"
  "float32 s_variance_m_s\\t\\t# GPS speed accuracy estimate, (metres/sec)\n"
  "float32 c_variance_rad\\t\\t# GPS course accuracy estimate, (radians)\n"
  "uint8 FIX_TYPE_NONE                   = 1       # Value 0 is also valid to represent no fix.\n"
  "uint8 FIX_TYPE_2D                     = 2\n"
  "uint8 FIX_TYPE_3D                     = 3\n"
  "uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4\n"
  "uint8 FIX_TYPE_RTK_FLOAT              = 5\n"
  "uint8 FIX_TYPE_RTK_FIXED              = 6\n"
  "uint8 FIX_TYPE_EXTRAPOLATED           = 8\n"
  "uint8 fix_type                  # Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.\n"
  "\n"
  "float32 eph\\t\\t\\t# GPS horizontal position accuracy (metres)\n"
  "float32 epv\\t\\t\\t# GPS vertical position accuracy (metres)\n"
  "\n"
  "float32 hdop\\t\\t\\t# Horizontal dilution of precision\n"
  "float32 vdop\\t\\t\\t# Vertical dilution of precision\n"
  "\n"
  "int32 noise_per_ms\\t\\t# GPS noise per millisecond\n"
  "uint16 automatic_gain_control   # Automatic gain control monitor\n"
  "\n"
  "uint8 JAMMING_STATE_UNKNOWN   = 0 #default\n"
  "uint8 JAMMING_STATE_OK        = 1\n"
  "uint8 JAMMING_STATE_MITIGATED = 2\n"
  "uint8 JAMMING_STATE_DETECTED  = 3\n"
  "uint8 jamming_state\\t      # indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Mitigated, 3: Detected\n"
  "int32 jamming_indicator\\t      # indicates jamming is occurring\n"
  "\n"
  "uint8 SPOOFING_STATE_UNKNOWN   = 0 #default\n"
  "uint8 SPOOFING_STATE_OK        = 1\n"
  "uint8 SPOOFING_STATE_MITIGATED = 2\n"
  "uint8 SPOOFING_STATE_DETECTED  = 3\n"
  "uint8 spoofing_state\\t       # indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Mitigated, 3: Detected\n"
  "\n"
  "# Combined authentication state (e.g. Galileo OSNMA)\n"
  "uint8 AUTHENTICATION_STATE_UNKNOWN      = 0 #default\n"
  "uint8 AUTHENTICATION_STATE_INITIALIZING = 1\n"
  "uint8 AUTHENTICATION_STATE_ERROR        = 2\n"
  "uint8 AUTHENTICATION_STATE_OK           = 3\n"
  "uint8 AUTHENTICATION_STATE_DISABLED     = 4\n"
  "uint8 authentication_state              # GPS signal authentication state\n"
  "\n"
  "float32 vel_m_s\\t\\t\\t# GPS ground speed, (metres/sec)\n"
  "float32 vel_n_m_s\\t\\t# GPS North velocity, (metres/sec)\n"
  "float32 vel_e_m_s\\t\\t# GPS East velocity, (metres/sec)\n"
  "float32 vel_d_m_s\\t\\t# GPS Down velocity, (metres/sec)\n"
  "float32 cog_rad\\t\\t\\t# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)\n"
  "bool vel_ned_valid\\t\\t# True if NED velocity is valid\n"
  "\n"
  "int32 timestamp_time_relative\\t# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)\n"
  "uint64 time_utc_usec\\t\\t# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0\n"
  "\n"
  "uint8 satellites_used\\t\\t# Number of satellites used\n"
  "\n"
  "uint32 SYSTEM_ERROR_OK                   = 0 #default\n"
  "uint32 SYSTEM_ERROR_INCOMING_CORRECTIONS = 1\n"
  "uint32 SYSTEM_ERROR_CONFIGURATION        = 2\n"
  "uint32 SYSTEM_ERROR_SOFTWARE             = 4\n"
  "uint32 SYSTEM_ERROR_ANTENNA              = 8\n"
  "uint32 SYSTEM_ERROR_EVENT_CONGESTION     = 16\n"
  "uint32 SYSTEM_ERROR_CPU_OVERLOAD         = 32\n"
  "uint32 SYSTEM_ERROR_OUTPUT_CONGESTION    = 64\n"
  "uint32 system_error                      # General errors with the connected GPS receiver\n"
  "\n"
  "float32 heading\\t\\t\\t# heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])\n"
  "float32 heading_offset\\t\\t# heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])\n"
  "float32 heading_accuracy\\t# heading accuracy (rad, [0, 2PI])\n"
  "\n"
  "float32 rtcm_injection_rate\\t# RTCM message injection rate Hz\n"
  "uint8 selected_rtcm_instance\\t# uorb instance that is being used for RTCM corrections\n"
  "\n"
  "bool rtcm_crc_failed\\t\\t# RTCM message CRC failure detected\n"
  "\n"
  "uint8 RTCM_MSG_USED_UNKNOWN = 0\n"
  "uint8 RTCM_MSG_USED_NOT_USED = 1\n"
  "uint8 RTCM_MSG_USED_USED = 2\n"
  "uint8 rtcm_msg_used\\t\\t# Indicates if the RTCM message was used successfully by the receiver\n"
  "\n"
  "# TOPICS sensor_gps vehicle_gps_position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorGps__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorGps__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 4526, 4526},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorGps__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorGps__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
