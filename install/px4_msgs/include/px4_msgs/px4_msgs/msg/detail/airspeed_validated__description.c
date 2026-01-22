// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/AirspeedValidated.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/airspeed_validated__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__AirspeedValidated__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7e, 0x9d, 0x97, 0x48, 0xec, 0xf4, 0x77, 0x5f,
      0xc7, 0x71, 0x6b, 0xbb, 0xa7, 0xdd, 0xd0, 0x47,
      0xee, 0x01, 0x31, 0xf0, 0x78, 0x60, 0x14, 0x66,
      0x60, 0x00, 0x9e, 0xed, 0x2b, 0xf0, 0x47, 0x6d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__AirspeedValidated__TYPE_NAME[] = "px4_msgs/msg/AirspeedValidated";

// Define type names, field names, and default values
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__indicated_airspeed_m_s[] = "indicated_airspeed_m_s";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__calibrated_airspeed_m_s[] = "calibrated_airspeed_m_s";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__true_airspeed_m_s[] = "true_airspeed_m_s";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__airspeed_source[] = "airspeed_source";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__calibrated_ground_minus_wind_m_s[] = "calibrated_ground_minus_wind_m_s";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__calibraded_airspeed_synth_m_s[] = "calibraded_airspeed_synth_m_s";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__airspeed_derivative_filtered[] = "airspeed_derivative_filtered";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__throttle_filtered[] = "throttle_filtered";
static char px4_msgs__msg__AirspeedValidated__FIELD_NAME__pitch_filtered[] = "pitch_filtered";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__AirspeedValidated__FIELDS[] = {
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__indicated_airspeed_m_s, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__calibrated_airspeed_m_s, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__true_airspeed_m_s, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__airspeed_source, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__calibrated_ground_minus_wind_m_s, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__calibraded_airspeed_synth_m_s, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__airspeed_derivative_filtered, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__throttle_filtered, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedValidated__FIELD_NAME__pitch_filtered, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__AirspeedValidated__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__AirspeedValidated__TYPE_NAME, 30, 30},
      {px4_msgs__msg__AirspeedValidated__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Validated airspeed\n"
  "#\n"
  "# Provides information about airspeed (indicated, true, calibrated) and the source of the data.\n"
  "# Used by controllers, estimators and for airspeed reporting to operator.\n"
  "\n"
  "\n"
  "uint32 MESSAGE_VERSION = 1\n"
  "\n"
  "uint64 timestamp  # [us] Time since system start\n"
  "\n"
  "float32 indicated_airspeed_m_s   # [m/s] [@invalid NaN] Indicated airspeed (IAS)\n"
  "float32 calibrated_airspeed_m_s  # [m/s] [@invalid NaN] Calibrated airspeed (CAS)\n"
  "float32 true_airspeed_m_s        # [m/s] [@invalid NaN] True airspeed (TAS)\n"
  "\n"
  "int8 airspeed_source               # [@enum SOURCE] Source of currently published airspeed values\n"
  "int8 SOURCE_DISABLED = -1          # Disabled\n"
  "int8 SOURCE_GROUND_MINUS_WIND = 0  # Ground speed minus wind\n"
  "int8 SOURCE_SENSOR_1 = 1           # Sensor 1\n"
  "int8 SOURCE_SENSOR_2 = 2           # Sensor 2\n"
  "int8 SOURCE_SENSOR_3 = 3           # Sensor 3\n"
  "int8 SOURCE_SYNTHETIC = 4          # Synthetic airspeed \n"
  "\n"
  "float32 calibrated_ground_minus_wind_m_s  # [m/s] [@invalid NaN] CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption\n"
  "float32 calibraded_airspeed_synth_m_s     # [m/s] [@invalid NaN] Synthetic airspeed\n"
  "float32 airspeed_derivative_filtered      # [m/s^2] Filtered indicated airspeed derivative \n"
  "float32 throttle_filtered                 # [-] Filtered fixed-wing throttle \n"
  "float32 pitch_filtered                    # [rad] Filtered pitch ";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__AirspeedValidated__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__AirspeedValidated__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1409, 1409},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__AirspeedValidated__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__AirspeedValidated__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
