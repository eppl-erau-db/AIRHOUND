// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ObstacleDistance.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/obstacle_distance__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ObstacleDistance__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8e, 0x67, 0xaa, 0x05, 0x35, 0x87, 0xcf, 0x93,
      0xa4, 0xcb, 0x62, 0x42, 0x8d, 0xe6, 0xd5, 0x8f,
      0x58, 0x42, 0xda, 0x83, 0xeb, 0xcf, 0x2c, 0xd2,
      0xca, 0x67, 0x84, 0x9d, 0x11, 0xa5, 0x82, 0xc8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ObstacleDistance__TYPE_NAME[] = "px4_msgs/msg/ObstacleDistance";

// Define type names, field names, and default values
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__frame[] = "frame";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__sensor_type[] = "sensor_type";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__distances[] = "distances";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__increment[] = "increment";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__min_distance[] = "min_distance";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__max_distance[] = "max_distance";
static char px4_msgs__msg__ObstacleDistance__FIELD_NAME__angle_offset[] = "angle_offset";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ObstacleDistance__FIELDS[] = {
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__frame, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__sensor_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__distances, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      72,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__increment, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__min_distance, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__max_distance, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ObstacleDistance__FIELD_NAME__angle_offset, 12, 12},
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
px4_msgs__msg__ObstacleDistance__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ObstacleDistance__TYPE_NAME, 29, 29},
      {px4_msgs__msg__ObstacleDistance__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Obstacle distances in front of the sensor.\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 frame\\t\\t#Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is North aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.\n"
  "uint8 MAV_FRAME_GLOBAL = 0\n"
  "uint8 MAV_FRAME_LOCAL_NED = 1\n"
  "uint8 MAV_FRAME_BODY_FRD = 12\n"
  "\n"
  "uint8 sensor_type # Type from MAV_DISTANCE_SENSOR enum.\n"
  "uint8 MAV_DISTANCE_SENSOR_LASER = 0\n"
  "uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1\n"
  "uint8 MAV_DISTANCE_SENSOR_INFRARED = 2\n"
  "uint8 MAV_DISTANCE_SENSOR_RADAR = 3\n"
  "\n"
  "uint16[72] distances # Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.\n"
  "\n"
  "float32 increment # Angular width in degrees of each array element.\n"
  "\n"
  "uint16 min_distance # Minimum distance the sensor can measure in centimeters.\n"
  "uint16 max_distance # Maximum distance the sensor can measure in centimeters.\n"
  "\n"
  "float32 angle_offset # Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive is clockwise direction, negative is counter-clockwise.\n"
  "\n"
  "# TOPICS obstacle_distance obstacle_distance_fused";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ObstacleDistance__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ObstacleDistance__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1426, 1426},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ObstacleDistance__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ObstacleDistance__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
