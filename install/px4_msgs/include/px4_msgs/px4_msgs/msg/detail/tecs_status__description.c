// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TecsStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/tecs_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TecsStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0xba, 0xac, 0xd1, 0xf7, 0xb4, 0x90, 0xea,
      0xfc, 0x33, 0x12, 0x8c, 0x84, 0x62, 0x74, 0xd8,
      0x07, 0x02, 0x78, 0x44, 0xc4, 0x0b, 0x57, 0x3b,
      0xb1, 0x0b, 0xa0, 0xcf, 0x1d, 0x43, 0xce, 0x3b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TecsStatus__TYPE_NAME[] = "px4_msgs/msg/TecsStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__TecsStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__altitude_sp[] = "altitude_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__altitude_reference[] = "altitude_reference";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__altitude_time_constant[] = "altitude_time_constant";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate_reference[] = "height_rate_reference";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate_direct[] = "height_rate_direct";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate_setpoint[] = "height_rate_setpoint";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate[] = "height_rate";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__equivalent_airspeed_sp[] = "equivalent_airspeed_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_sp[] = "true_airspeed_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_filtered[] = "true_airspeed_filtered";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_derivative_sp[] = "true_airspeed_derivative_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_derivative[] = "true_airspeed_derivative";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_derivative_raw[] = "true_airspeed_derivative_raw";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_rate_sp[] = "total_energy_rate_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_rate[] = "total_energy_rate";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_balance_rate_sp[] = "total_energy_balance_rate_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_balance_rate[] = "total_energy_balance_rate";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__throttle_integ[] = "throttle_integ";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__pitch_integ[] = "pitch_integ";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__throttle_sp[] = "throttle_sp";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__pitch_sp_rad[] = "pitch_sp_rad";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__throttle_trim[] = "throttle_trim";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__underspeed_ratio[] = "underspeed_ratio";
static char px4_msgs__msg__TecsStatus__FIELD_NAME__fast_descend_ratio[] = "fast_descend_ratio";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TecsStatus__FIELDS[] = {
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__altitude_sp, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__altitude_reference, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__altitude_time_constant, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate_reference, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate_direct, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate_setpoint, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__height_rate, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__equivalent_airspeed_sp, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_sp, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_filtered, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_derivative_sp, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_derivative, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__true_airspeed_derivative_raw, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_rate_sp, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_rate, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_balance_rate_sp, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__total_energy_balance_rate, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__throttle_integ, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__pitch_integ, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__throttle_sp, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__pitch_sp_rad, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__throttle_trim, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__underspeed_ratio, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TecsStatus__FIELD_NAME__fast_descend_ratio, 18, 18},
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
px4_msgs__msg__TecsStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TecsStatus__TYPE_NAME, 23, 23},
      {px4_msgs__msg__TecsStatus__FIELDS, 25, 25},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "float32 altitude_sp\\t\\t\\t# Altitude setpoint AMSL [m]\n"
  "float32 altitude_reference\\t\\t# Altitude setpoint reference AMSL [m]\n"
  "float32 altitude_time_constant\\t\\t# Time constant of the altitude tracker [s]\n"
  "float32 height_rate_reference\\t\\t# Height rate setpoint reference [m/s]\n"
  "float32 height_rate_direct\\t\\t# Direct height rate setpoint from velocity reference generator [m/s]\n"
  "float32 height_rate_setpoint\\t\\t# Height rate setpoint [m/s]\n"
  "float32 height_rate\\t\\t\\t# Height rate [m/s]\n"
  "float32 equivalent_airspeed_sp\\t\\t# Equivalent airspeed setpoint [m/s]\n"
  "float32 true_airspeed_sp\\t\\t# True airspeed setpoint [m/s]\n"
  "float32 true_airspeed_filtered\\t\\t# True airspeed filtered [m/s]\n"
  "float32 true_airspeed_derivative_sp\\t# True airspeed derivative setpoint [m/s^2]\n"
  "float32 true_airspeed_derivative\\t# True airspeed derivative [m/s^2]\n"
  "float32 true_airspeed_derivative_raw\\t# True airspeed derivative raw [m/s^2]\n"
  "\n"
  "float32 total_energy_rate_sp\\t\\t# Total energy rate setpoint [m^2/s^3]\n"
  "float32 total_energy_rate\\t\\t# Total energy rate estimate [m^2/s^3]\n"
  "\n"
  "float32 total_energy_balance_rate_sp\\t# Energy balance rate setpoint [m^2/s^3]\n"
  "float32 total_energy_balance_rate\\t# Energy balance rate estimate [m^2/s^3]\n"
  "\n"
  "float32 throttle_integ\\t\\t\\t# Throttle integrator value [-]\n"
  "float32 pitch_integ\\t\\t\\t# Pitch integrator value [rad]\n"
  "\n"
  "float32 throttle_sp\\t\\t\\t# Current throttle setpoint [-]\n"
  "float32 pitch_sp_rad\\t\\t\\t# Current pitch setpoint [rad]\n"
  "float32 throttle_trim\\t\\t\\t# estimated throttle value [0,1] required to fly level at equivalent_airspeed_sp in the current atmospheric conditions\n"
  "\n"
  "float32 underspeed_ratio\\t\\t# 0: no underspeed, 1: maximal underspeed. Controller takes measures to avoid stall proportional to ratio if >0.\n"
  "float32 fast_descend_ratio \\t\\t#  value indicating if fast descend mode is enabled with ramp up and ramp down [0-1]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TecsStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TecsStatus__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1843, 1843},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TecsStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TecsStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
