// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ControlAllocatorStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/control_allocator_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ControlAllocatorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x61, 0x5d, 0xe5, 0xa4, 0x7c, 0x08, 0x7e, 0x5c,
      0x4f, 0x9a, 0x83, 0x2d, 0xcd, 0xe7, 0xf9, 0x85,
      0x80, 0xc4, 0xb9, 0x2e, 0x3d, 0x64, 0x2d, 0x95,
      0x82, 0x17, 0x36, 0x9d, 0x61, 0x79, 0x51, 0x92,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ControlAllocatorStatus__TYPE_NAME[] = "px4_msgs/msg/ControlAllocatorStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__torque_setpoint_achieved[] = "torque_setpoint_achieved";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__unallocated_torque[] = "unallocated_torque";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__thrust_setpoint_achieved[] = "thrust_setpoint_achieved";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__unallocated_thrust[] = "unallocated_thrust";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__actuator_saturation[] = "actuator_saturation";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__handled_motor_failure_mask[] = "handled_motor_failure_mask";
static char px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__motor_stop_mask[] = "motor_stop_mask";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ControlAllocatorStatus__FIELDS[] = {
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__torque_setpoint_achieved, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__unallocated_torque, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__thrust_setpoint_achieved, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__unallocated_thrust, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__actuator_saturation, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__handled_motor_failure_mask, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ControlAllocatorStatus__FIELD_NAME__motor_stop_mask, 15, 15},
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
px4_msgs__msg__ControlAllocatorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ControlAllocatorStatus__TYPE_NAME, 35, 35},
      {px4_msgs__msg__ControlAllocatorStatus__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                        # time since system start (microseconds)\n"
  "\n"
  "bool torque_setpoint_achieved           # Boolean indicating whether the 3D torque setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.\n"
  "float32[3] unallocated_torque           # Unallocated torque. Equal to 0 if the setpoint was achieved.\n"
  "                                        # Computed as: unallocated_torque = torque_setpoint - allocated_torque\n"
  "\n"
  "bool thrust_setpoint_achieved           # Boolean indicating whether the 3D thrust setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.\n"
  "float32[3] unallocated_thrust           # Unallocated thrust. Equal to 0 if the setpoint was achieved.\n"
  "                                        # Computed as: unallocated_thrust = thrust_setpoint - allocated_thrust\n"
  "\n"
  "int8 ACTUATOR_SATURATION_OK        =  0 # The actuator is not saturated\n"
  "int8 ACTUATOR_SATURATION_UPPER_DYN =  1 # The actuator is saturated (with a value <= the desired value) because it cannot increase its value faster\n"
  "int8 ACTUATOR_SATURATION_UPPER     =  2 # The actuator is saturated (with a value <= the desired value) because it has reached its maximum value\n"
  "int8 ACTUATOR_SATURATION_LOWER_DYN = -1 # The actuator is saturated (with a value >= the desired value) because it cannot decrease its value faster\n"
  "int8 ACTUATOR_SATURATION_LOWER     = -2 # The actuator is saturated (with a value >= the desired value) because it has reached its minimum value\n"
  "\n"
  "int8[16] actuator_saturation            # Indicates actuator saturation status.\n"
  "                                        # Note 1: actuator saturation does not necessarily imply that the thrust setpoint or the torque setpoint were not achieved.\n"
  "                                        # Note 2: an actuator with limited dynamics can be indicated as upper-saturated even if it as not reached its maximum value.\n"
  "\n"
  "uint16 handled_motor_failure_mask        # Bitmask of failed motors that were removed from the allocation / effectiveness matrix. Not necessarily identical to the report from FailureDetector\n"
  "uint16 motor_stop_mask                   # Bitmaks of motors stopped by failure injection";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ControlAllocatorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ControlAllocatorStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2190, 2190},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ControlAllocatorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ControlAllocatorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
