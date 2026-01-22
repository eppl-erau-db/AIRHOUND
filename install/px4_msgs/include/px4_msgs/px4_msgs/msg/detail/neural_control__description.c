// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/NeuralControl.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/neural_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__NeuralControl__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0xdb, 0xa8, 0x9d, 0xb1, 0x08, 0xf2, 0x7d,
      0x3a, 0x6d, 0x45, 0x6f, 0xc3, 0xe9, 0x67, 0x42,
      0xdf, 0x03, 0x58, 0xbc, 0xac, 0xe4, 0x1f, 0x47,
      0x0f, 0xa6, 0xeb, 0x00, 0x45, 0x4d, 0xb8, 0x57,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__NeuralControl__TYPE_NAME[] = "px4_msgs/msg/NeuralControl";

// Define type names, field names, and default values
static char px4_msgs__msg__NeuralControl__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__NeuralControl__FIELD_NAME__observation[] = "observation";
static char px4_msgs__msg__NeuralControl__FIELD_NAME__network_output[] = "network_output";
static char px4_msgs__msg__NeuralControl__FIELD_NAME__controller_time[] = "controller_time";
static char px4_msgs__msg__NeuralControl__FIELD_NAME__inference_time[] = "inference_time";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__NeuralControl__FIELDS[] = {
  {
    {px4_msgs__msg__NeuralControl__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NeuralControl__FIELD_NAME__observation, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      15,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NeuralControl__FIELD_NAME__network_output, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NeuralControl__FIELD_NAME__controller_time, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NeuralControl__FIELD_NAME__inference_time, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__NeuralControl__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__NeuralControl__TYPE_NAME, 26, 26},
      {px4_msgs__msg__NeuralControl__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Neural control\n"
  "#\n"
  "# Debugging topic for the Neural controller, logs the inputs and output vectors of the neural network, and the time it takes to run\n"
  "# Publisher: mc_nn_control\n"
  "# Subscriber: logger\n"
  "\n"
  "uint64 timestamp # [us] Time since system start\n"
  "\n"
  "float32[15] observation # Observation vector (pos error (3), att (6d), lin vel (3), ang vel (3))\n"
  "float32[4] network_output # Output from neural network\n"
  "\n"
  "int32 controller_time # [us] Time spent from input to output\n"
  "int32 inference_time # [us] Time spent for NN inference";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__NeuralControl__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__NeuralControl__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 519, 519},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__NeuralControl__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__NeuralControl__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
