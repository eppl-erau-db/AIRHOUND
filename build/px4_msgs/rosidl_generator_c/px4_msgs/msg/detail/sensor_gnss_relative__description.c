// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorGnssRelative.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_gnss_relative__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorGnssRelative__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb6, 0x69, 0x6c, 0x9e, 0xc4, 0xe2, 0xc1, 0x4d,
      0xbb, 0x56, 0x18, 0x98, 0x0f, 0x22, 0x9f, 0x76,
      0x1a, 0x35, 0x46, 0xb1, 0x54, 0x4b, 0x0c, 0x35,
      0x59, 0x50, 0x4a, 0x35, 0xda, 0xf4, 0xd2, 0x89,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorGnssRelative__TYPE_NAME[] = "px4_msgs/msg/SensorGnssRelative";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__time_utc_usec[] = "time_utc_usec";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__reference_station_id[] = "reference_station_id";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__position_accuracy[] = "position_accuracy";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__heading_accuracy[] = "heading_accuracy";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__position_length[] = "position_length";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__accuracy_length[] = "accuracy_length";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__gnss_fix_ok[] = "gnss_fix_ok";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__differential_solution[] = "differential_solution";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__relative_position_valid[] = "relative_position_valid";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__carrier_solution_floating[] = "carrier_solution_floating";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__carrier_solution_fixed[] = "carrier_solution_fixed";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__moving_base_mode[] = "moving_base_mode";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__reference_position_miss[] = "reference_position_miss";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__reference_observations_miss[] = "reference_observations_miss";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__heading_valid[] = "heading_valid";
static char px4_msgs__msg__SensorGnssRelative__FIELD_NAME__relative_position_normalized[] = "relative_position_normalized";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorGnssRelative__FIELDS[] = {
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__time_utc_usec, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__reference_station_id, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__position_accuracy, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__heading_accuracy, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__position_length, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__accuracy_length, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__gnss_fix_ok, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__differential_solution, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__relative_position_valid, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__carrier_solution_floating, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__carrier_solution_fixed, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__moving_base_mode, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__reference_position_miss, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__reference_observations_miss, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__heading_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssRelative__FIELD_NAME__relative_position_normalized, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorGnssRelative__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorGnssRelative__TYPE_NAME, 31, 31},
      {px4_msgs__msg__SensorGnssRelative__FIELDS, 21, 21},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GNSS relative positioning information in NED frame. The NED frame is defined as the local topological system at the reference station.\n"
  "\n"
  "uint64 timestamp                  # time since system start (microseconds)\n"
  "uint64 timestamp_sample           # time since system start (microseconds)\n"
  "\n"
  "uint32 device_id                  # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "uint64 time_utc_usec              # Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0\n"
  "\n"
  "uint16 reference_station_id       # Reference Station ID\n"
  "\n"
  "float32[3] position               # GPS NED relative position vector (m)\n"
  "float32[3] position_accuracy      # Accuracy of relative position (m)\n"
  "\n"
  "float32 heading                   # Heading of the relative position vector (radians)\n"
  "float32 heading_accuracy          # Accuracy of heading of the relative position vector (radians)\n"
  "\n"
  "float32 position_length           # Length of the position vector (m)\n"
  "float32 accuracy_length           # Accuracy of the position length (m)\n"
  "\n"
  "bool gnss_fix_ok                  # GNSS valid fix (i.e within DOP & accuracy masks)\n"
  "bool differential_solution        # differential corrections were applied\n"
  "bool relative_position_valid\n"
  "bool carrier_solution_floating    # carrier phase range solution with floating ambiguities\n"
  "bool carrier_solution_fixed       # carrier phase range solution with fixed ambiguities\n"
  "bool moving_base_mode             # if the receiver is operating in moving base mode\n"
  "bool reference_position_miss      # extrapolated reference position was used to compute moving base solution this epoch\n"
  "bool reference_observations_miss  # extrapolated reference observations were used to compute moving base solution this epoch\n"
  "bool heading_valid\n"
  "bool relative_position_normalized # the components of the relative position vector (including the high-precision parts) are normalized";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorGnssRelative__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorGnssRelative__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1978, 1978},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorGnssRelative__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorGnssRelative__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
