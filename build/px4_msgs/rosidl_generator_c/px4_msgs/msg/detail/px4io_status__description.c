// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Px4ioStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/px4io_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Px4ioStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0x54, 0x97, 0x24, 0xc0, 0xbd, 0xf5, 0x9a,
      0x34, 0x37, 0xae, 0xb2, 0x2e, 0x70, 0x4e, 0x4c,
      0x33, 0x3c, 0x83, 0x1d, 0x44, 0xa0, 0xeb, 0x35,
      0x8b, 0x0a, 0x4a, 0x6f, 0xc2, 0xd9, 0x4f, 0x0b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Px4ioStatus__TYPE_NAME[] = "px4_msgs/msg/Px4ioStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__free_memory_bytes[] = "free_memory_bytes";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__voltage_v[] = "voltage_v";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__rssi_v[] = "rssi_v";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_arm_sync[] = "status_arm_sync";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_failsafe[] = "status_failsafe";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_fmu_initialized[] = "status_fmu_initialized";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_fmu_ok[] = "status_fmu_ok";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_init_ok[] = "status_init_ok";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_outputs_armed[] = "status_outputs_armed";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_raw_pwm[] = "status_raw_pwm";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_ok[] = "status_rc_ok";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_dsm[] = "status_rc_dsm";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_ppm[] = "status_rc_ppm";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_sbus[] = "status_rc_sbus";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_st24[] = "status_rc_st24";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_sumd[] = "status_rc_sumd";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_safety_button_event[] = "status_safety_button_event";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__alarm_pwm_error[] = "alarm_pwm_error";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__alarm_rc_lost[] = "alarm_rc_lost";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_failsafe_custom[] = "arming_failsafe_custom";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_fmu_armed[] = "arming_fmu_armed";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_fmu_prearmed[] = "arming_fmu_prearmed";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_termination[] = "arming_termination";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_io_arm_ok[] = "arming_io_arm_ok";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_lockdown[] = "arming_lockdown";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_termination_failsafe[] = "arming_termination_failsafe";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm[] = "pwm";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm_disarmed[] = "pwm_disarmed";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm_failsafe[] = "pwm_failsafe";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm_rate_hz[] = "pwm_rate_hz";
static char px4_msgs__msg__Px4ioStatus__FIELD_NAME__raw_inputs[] = "raw_inputs";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Px4ioStatus__FIELDS[] = {
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__free_memory_bytes, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__voltage_v, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__rssi_v, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_arm_sync, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_failsafe, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_fmu_initialized, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_fmu_ok, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_init_ok, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_outputs_armed, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_raw_pwm, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_ok, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_dsm, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_ppm, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_sbus, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_st24, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_rc_sumd, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__status_safety_button_event, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__alarm_pwm_error, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__alarm_rc_lost, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_failsafe_custom, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_fmu_armed, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_fmu_prearmed, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_termination, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_io_arm_ok, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_lockdown, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__arming_termination_failsafe, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm_disarmed, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm_failsafe, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__pwm_rate_hz, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Px4ioStatus__FIELD_NAME__raw_inputs, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      18,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__Px4ioStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Px4ioStatus__TYPE_NAME, 24, 24},
      {px4_msgs__msg__Px4ioStatus__FIELDS, 32, 32},
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
  "uint16 free_memory_bytes\n"
  "\n"
  "float32 voltage_v\\t\\t# Servo rail voltage in volts\n"
  "float32 rssi_v\\t\\t\\t# RSSI pin voltage in volts\n"
  "\n"
  "# PX4IO status flags (PX4IO_P_STATUS_FLAGS)\n"
  "bool status_arm_sync\n"
  "bool status_failsafe\n"
  "bool status_fmu_initialized\n"
  "bool status_fmu_ok\n"
  "bool status_init_ok\n"
  "bool status_outputs_armed\n"
  "bool status_raw_pwm\n"
  "bool status_rc_ok\n"
  "bool status_rc_dsm\n"
  "bool status_rc_ppm\n"
  "bool status_rc_sbus\n"
  "bool status_rc_st24\n"
  "bool status_rc_sumd\n"
  "bool status_safety_button_event # px4io safety button was pressed for longer than 1 second\n"
  "\n"
  "# PX4IO alarms (PX4IO_P_STATUS_ALARMS)\n"
  "bool alarm_pwm_error\n"
  "bool alarm_rc_lost\n"
  "\n"
  "# PX4IO arming (PX4IO_P_SETUP_ARMING)\n"
  "bool arming_failsafe_custom\n"
  "bool arming_fmu_armed\n"
  "bool arming_fmu_prearmed\n"
  "bool arming_termination\n"
  "bool arming_io_arm_ok\n"
  "bool arming_lockdown\n"
  "bool arming_termination_failsafe\n"
  "\n"
  "uint16[8] pwm\n"
  "uint16[8] pwm_disarmed\n"
  "uint16[8] pwm_failsafe\n"
  "\n"
  "uint16[8] pwm_rate_hz\n"
  "\n"
  "uint16[18] raw_inputs";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Px4ioStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Px4ioStatus__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 988, 988},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Px4ioStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Px4ioStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
