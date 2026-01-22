// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/InternalCombustionEngineStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/internal_combustion_engine_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__InternalCombustionEngineStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x09, 0x18, 0x7f, 0x7f, 0x1c, 0x5f, 0x9a, 0x08,
      0xd3, 0x5f, 0x47, 0xdf, 0xe2, 0xb3, 0x44, 0x4b,
      0x3f, 0x96, 0x7a, 0xff, 0xa0, 0x06, 0x32, 0x8e,
      0xf8, 0x46, 0x80, 0x86, 0xff, 0x44, 0xd6, 0x57,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__InternalCombustionEngineStatus__TYPE_NAME[] = "px4_msgs/msg/InternalCombustionEngineStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__state[] = "state";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__flags[] = "flags";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__engine_load_percent[] = "engine_load_percent";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__engine_speed_rpm[] = "engine_speed_rpm";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__spark_dwell_time_ms[] = "spark_dwell_time_ms";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__atmospheric_pressure_kpa[] = "atmospheric_pressure_kpa";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__intake_manifold_pressure_kpa[] = "intake_manifold_pressure_kpa";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__intake_manifold_temperature[] = "intake_manifold_temperature";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__coolant_temperature[] = "coolant_temperature";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__oil_pressure[] = "oil_pressure";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__oil_temperature[] = "oil_temperature";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__fuel_pressure[] = "fuel_pressure";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__fuel_consumption_rate_cm3pm[] = "fuel_consumption_rate_cm3pm";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__estimated_consumed_fuel_volume_cm3[] = "estimated_consumed_fuel_volume_cm3";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__throttle_position_percent[] = "throttle_position_percent";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__ecu_index[] = "ecu_index";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__spark_plug_usage[] = "spark_plug_usage";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__ignition_timing_deg[] = "ignition_timing_deg";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__injection_time_ms[] = "injection_time_ms";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__cylinder_head_temperature[] = "cylinder_head_temperature";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__exhaust_gas_temperature[] = "exhaust_gas_temperature";
static char px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__lambda_coefficient[] = "lambda_coefficient";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__InternalCombustionEngineStatus__FIELDS[] = {
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__flags, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__engine_load_percent, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__engine_speed_rpm, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__spark_dwell_time_ms, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__atmospheric_pressure_kpa, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__intake_manifold_pressure_kpa, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__intake_manifold_temperature, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__coolant_temperature, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__oil_pressure, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__oil_temperature, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__fuel_pressure, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__fuel_consumption_rate_cm3pm, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__estimated_consumed_fuel_volume_cm3, 34, 34},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__throttle_position_percent, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__ecu_index, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__spark_plug_usage, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__ignition_timing_deg, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__injection_time_ms, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__cylinder_head_temperature, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__exhaust_gas_temperature, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineStatus__FIELD_NAME__lambda_coefficient, 18, 18},
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
px4_msgs__msg__InternalCombustionEngineStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__InternalCombustionEngineStatus__TYPE_NAME, 43, 43},
      {px4_msgs__msg__InternalCombustionEngineStatus__FIELDS, 23, 23},
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
  "uint8 STATE_STOPPED = 0\\t\\t\\t\\t\\t# The engine is not running. This is the default state.\n"
  "uint8 STATE_STARTING = 1\\t\\t\\t\\t# The engine is starting. This is a transient state.\n"
  "uint8 STATE_RUNNING = 2\\t\\t\\t\\t\\t# The engine is running normally.\n"
  "uint8 STATE_FAULT = 3\\t\\t\\t\\t\\t# The engine can no longer function.\n"
  "uint8 state\n"
  "\n"
  "uint32 FLAG_GENERAL_ERROR = 1\\t\\t\\t\\t# General error.\n"
  "\n"
  "uint32 FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED = 2\\t# Error of the crankshaft sensor. This flag is optional.\n"
  "uint32 FLAG_CRANKSHAFT_SENSOR_ERROR = 4\n"
  "\n"
  "uint32 FLAG_TEMPERATURE_SUPPORTED = 8\\t\\t\\t# Temperature levels. These flags are optional\n"
  "uint32 FLAG_TEMPERATURE_BELOW_NOMINAL = 16      \\t# Under-temperature warning\n"
  "uint32 FLAG_TEMPERATURE_ABOVE_NOMINAL = 32      \\t# Over-temperature warning\n"
  "uint32 FLAG_TEMPERATURE_OVERHEATING = 64      \\t\\t# Critical overheating\n"
  "uint32 FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL = 128     \\t# Exhaust gas over-temperature warning\n"
  "\n"
  "uint32 FLAG_FUEL_PRESSURE_SUPPORTED = 256\\t\\t# Fuel pressure. These flags are optional\n"
  "uint32 FLAG_FUEL_PRESSURE_BELOW_NOMINAL  = 512     \\t# Under-pressure warning\n"
  "uint32 FLAG_FUEL_PRESSURE_ABOVE_NOMINAL = 1024   \\t# Over-pressure warning\n"
  "\n"
  "uint32 FLAG_DETONATION_SUPPORTED = 2048\\t\\t\\t# Detonation warning. This flag is optional.\n"
  "uint32 FLAG_DETONATION_OBSERVED = 4096    \\t\\t# Detonation condition observed warning\n"
  "\n"
  "uint32 FLAG_MISFIRE_SUPPORTED = 8192\\t\\t\\t# Misfire warning. This flag is optional.\n"
  "uint32 FLAG_MISFIRE_OBSERVED = 16384   \\t\\t\\t# Misfire condition observed warning\n"
  "\n"
  "uint32 FLAG_OIL_PRESSURE_SUPPORTED = 32768\\t\\t# Oil pressure. These flags are optional\n"
  "uint32 FLAG_OIL_PRESSURE_BELOW_NOMINAL = 65536   \\t# Under-pressure warning\n"
  "uint32 FLAG_OIL_PRESSURE_ABOVE_NOMINAL = 131072  \\t# Over-pressure warning\n"
  "\n"
  "uint32 FLAG_DEBRIS_SUPPORTED = 262144\\t\\t\\t# Debris warning. This flag is optional\n"
  "uint32 FLAG_DEBRIS_DETECTED = 524288  \\t\\t\\t# Detection of debris warning\n"
  "uint32 flags\n"
  "\n"
  "uint8 engine_load_percent\\t\\t\\t\\t# Engine load estimate, percent, [0, 127]\n"
  "uint32 engine_speed_rpm\\t\\t\\t\\t\\t# Engine speed, revolutions per minute\n"
  "float32 spark_dwell_time_ms \\t\\t\\t\\t# Spark dwell time, millisecond\n"
  "float32 atmospheric_pressure_kpa\\t\\t\\t# Atmospheric (barometric) pressure, kilopascal\n"
  "float32 intake_manifold_pressure_kpa\\t\\t\\t# Engine intake manifold pressure, kilopascal\n"
  "float32 intake_manifold_temperature\\t\\t\\t# Engine intake manifold temperature, kelvin\n"
  "float32 coolant_temperature\\t\\t\\t\\t# Engine coolant temperature, kelvin\n"
  "float32 oil_pressure\\t\\t\\t\\t\\t# Oil pressure, kilopascal\n"
  "float32 oil_temperature\\t\\t\\t\\t\\t# Oil temperature, kelvin\n"
  "float32 fuel_pressure\\t\\t\\t\\t\\t# Fuel pressure, kilopascal\n"
  "float32 fuel_consumption_rate_cm3pm\\t\\t\\t# Instant fuel consumption estimate, (centimeter^3)/minute\n"
  "float32 estimated_consumed_fuel_volume_cm3\\t\\t# Estimate of the consumed fuel since the start of the engine, centimeter^3\n"
  "uint8 throttle_position_percent\\t\\t\\t\\t# Throttle position, percent\n"
  "uint8 ecu_index\\t\\t\\t\\t\\t\\t# The index of the publishing ECU\n"
  "\n"
  "\n"
  "uint8 SPARK_PLUG_SINGLE         = 0\n"
  "uint8 SPARK_PLUG_FIRST_ACTIVE   = 1\n"
  "uint8 SPARK_PLUG_SECOND_ACTIVE  = 2\n"
  "uint8 SPARK_PLUG_BOTH_ACTIVE    = 3\n"
  "uint8 spark_plug_usage\\t\\t\\t\\t\\t# Spark plug activity report.\n"
  "\n"
  "float32 ignition_timing_deg\\t\\t\\t\\t# Cylinder ignition timing, angular degrees of the crankshaft\n"
  "float32 injection_time_ms\\t\\t\\t\\t# Fuel injection time, millisecond\n"
  "float32 cylinder_head_temperature\\t\\t\\t# Cylinder head temperature (CHT), kelvin\n"
  "float32 exhaust_gas_temperature\\t\\t\\t\\t# Exhaust gas temperature (EGT), kelvin\n"
  "float32 lambda_coefficient\\t\\t\\t\\t# Estimated lambda coefficient, dimensionless ratio";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__InternalCombustionEngineStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__InternalCombustionEngineStatus__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 3552, 3552},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__InternalCombustionEngineStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__InternalCombustionEngineStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
