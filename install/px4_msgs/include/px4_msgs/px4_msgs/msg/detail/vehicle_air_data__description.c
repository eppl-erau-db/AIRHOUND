// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleAirData.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_air_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleAirData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x09, 0x96, 0x33, 0xed, 0x65, 0x33, 0xa1, 0x53,
      0x04, 0xfa, 0x22, 0x99, 0x13, 0xb1, 0x2d, 0x1e,
      0xe5, 0xe9, 0x8e, 0xde, 0x1f, 0x8d, 0xb7, 0x26,
      0xbd, 0xd4, 0x19, 0xc4, 0xcc, 0x80, 0xef, 0xc2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleAirData__TYPE_NAME[] = "px4_msgs/msg/VehicleAirData";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__baro_device_id[] = "baro_device_id";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__baro_alt_meter[] = "baro_alt_meter";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__baro_pressure_pa[] = "baro_pressure_pa";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__ambient_temperature[] = "ambient_temperature";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__temperature_source[] = "temperature_source";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__rho[] = "rho";
static char px4_msgs__msg__VehicleAirData__FIELD_NAME__calibration_count[] = "calibration_count";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleAirData__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__baro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__baro_alt_meter, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__baro_pressure_pa, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__ambient_temperature, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__temperature_source, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__rho, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAirData__FIELD_NAME__calibration_count, 17, 17},
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
px4_msgs__msg__VehicleAirData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleAirData__TYPE_NAME, 27, 27},
      {px4_msgs__msg__VehicleAirData__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Vehicle air data\n"
  "#\n"
  "# Data from the currently selected barometer (plus ambient temperature from the source specified in temperature_source).\n"
  "# Includes calculated data such as barometric altitude and air density.\n"
  "\n"
  "uint64 timestamp             # [us] Time since system start\n"
  "uint64 timestamp_sample      # [us] Timestamp of the raw data\n"
  "uint32 baro_device_id        # Unique device ID for the selected barometer\n"
  "float32 baro_alt_meter       # [m] [@frame MSL] Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH\n"
  "float32 baro_pressure_pa     # [Pa] Absolute pressure\n"
  "float32 ambient_temperature  # [degC] Ambient temperature\n"
  "uint8 temperature_source     # Source of temperature data: 0: Default Temperature (15\\xc2\\xb0C), 1: External Baro, 2: Airspeed\n"
  "float32 rho                  # [kg/m^3] Air density\n"
  "uint8 calibration_count      # Calibration changed counter. Monotonically increases whenever calibration changes.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleAirData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleAirData__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 994, 994},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleAirData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleAirData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
