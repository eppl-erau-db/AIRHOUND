// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SystemPower.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/system_power__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SystemPower__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0e, 0x21, 0x97, 0x6e, 0x6f, 0x9e, 0x06, 0xe1,
      0x3e, 0xc0, 0x94, 0x81, 0x98, 0xe0, 0x84, 0xe0,
      0x5b, 0xa8, 0x77, 0x67, 0xff, 0xaa, 0xf9, 0xb2,
      0xc3, 0xfd, 0xbc, 0xf2, 0x9b, 0x22, 0x23, 0x52,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SystemPower__TYPE_NAME[] = "px4_msgs/msg/SystemPower";

// Define type names, field names, and default values
static char px4_msgs__msg__SystemPower__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SystemPower__FIELD_NAME__voltage5v_v[] = "voltage5v_v";
static char px4_msgs__msg__SystemPower__FIELD_NAME__voltage_payload_v[] = "voltage_payload_v";
static char px4_msgs__msg__SystemPower__FIELD_NAME__sensors3v3[] = "sensors3v3";
static char px4_msgs__msg__SystemPower__FIELD_NAME__sensors3v3_valid[] = "sensors3v3_valid";
static char px4_msgs__msg__SystemPower__FIELD_NAME__usb_connected[] = "usb_connected";
static char px4_msgs__msg__SystemPower__FIELD_NAME__brick_valid[] = "brick_valid";
static char px4_msgs__msg__SystemPower__FIELD_NAME__usb_valid[] = "usb_valid";
static char px4_msgs__msg__SystemPower__FIELD_NAME__servo_valid[] = "servo_valid";
static char px4_msgs__msg__SystemPower__FIELD_NAME__periph_5v_oc[] = "periph_5v_oc";
static char px4_msgs__msg__SystemPower__FIELD_NAME__hipower_5v_oc[] = "hipower_5v_oc";
static char px4_msgs__msg__SystemPower__FIELD_NAME__comp_5v_valid[] = "comp_5v_valid";
static char px4_msgs__msg__SystemPower__FIELD_NAME__can1_gps1_5v_valid[] = "can1_gps1_5v_valid";
static char px4_msgs__msg__SystemPower__FIELD_NAME__payload_v_valid[] = "payload_v_valid";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SystemPower__FIELDS[] = {
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__voltage5v_v, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__voltage_payload_v, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__sensors3v3, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__sensors3v3_valid, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__usb_connected, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__brick_valid, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__usb_valid, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__servo_valid, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__periph_5v_oc, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__hipower_5v_oc, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__comp_5v_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__can1_gps1_5v_valid, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SystemPower__FIELD_NAME__payload_v_valid, 15, 15},
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
px4_msgs__msg__SystemPower__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SystemPower__TYPE_NAME, 24, 24},
      {px4_msgs__msg__SystemPower__FIELDS, 14, 14},
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
  "float32 voltage5v_v\\t\\t# peripheral 5V rail voltage\n"
  "float32 voltage_payload_v\\t# payload rail voltage\n"
  "float32[4] sensors3v3\\t\\t# Sensors 3V3 rail voltage\n"
  "uint8 sensors3v3_valid\\t\\t# Sensors 3V3 rail voltage was read (bitfield).\n"
  "uint8 usb_connected\\t\\t# USB is connected when 1\n"
  "uint8 brick_valid\\t\\t# brick bits power is good when bit 1\n"
  "uint8 usb_valid \\t\\t# USB is valid when 1\n"
  "uint8 servo_valid\\t\\t# servo power is good when 1\n"
  "uint8 periph_5v_oc\\t\\t# peripheral overcurrent when 1\n"
  "uint8 hipower_5v_oc\\t\\t# high power peripheral overcurrent when 1\n"
  "uint8 comp_5v_valid\\t\\t# 5V to companion valid\n"
  "uint8 can1_gps1_5v_valid\\t# 5V for CAN1/GPS1 valid\n"
  "uint8 payload_v_valid\\t\\t# payload rail voltage is valid\n"
  "\n"
  "uint8 BRICK1_VALID_SHIFTS=0\n"
  "uint8 BRICK1_VALID_MASK=1\n"
  "uint8 BRICK2_VALID_SHIFTS=1\n"
  "uint8 BRICK2_VALID_MASK=2\n"
  "uint8 BRICK3_VALID_SHIFTS=2\n"
  "uint8 BRICK3_VALID_MASK=4\n"
  "uint8 BRICK4_VALID_SHIFTS=3\n"
  "uint8 BRICK4_VALID_MASK=8";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SystemPower__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SystemPower__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 955, 955},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SystemPower__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SystemPower__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
