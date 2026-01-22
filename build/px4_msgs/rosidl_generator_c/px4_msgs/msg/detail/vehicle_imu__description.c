// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleImu.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_imu__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleImu__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0x91, 0x6c, 0xbe, 0xa4, 0x4a, 0x1f, 0x27,
      0x73, 0x88, 0xc7, 0xd8, 0xc3, 0xd1, 0xd5, 0xc7,
      0xac, 0x01, 0xbb, 0x02, 0x0e, 0xa0, 0x9e, 0x16,
      0xa9, 0x40, 0xd5, 0x88, 0x02, 0x83, 0x4c, 0x25,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleImu__TYPE_NAME[] = "px4_msgs/msg/VehicleImu";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleImu__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__accel_device_id[] = "accel_device_id";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__gyro_device_id[] = "gyro_device_id";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__delta_angle[] = "delta_angle";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__delta_velocity[] = "delta_velocity";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__delta_angle_dt[] = "delta_angle_dt";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__delta_velocity_dt[] = "delta_velocity_dt";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__delta_angle_clipping[] = "delta_angle_clipping";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__delta_velocity_clipping[] = "delta_velocity_clipping";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__accel_calibration_count[] = "accel_calibration_count";
static char px4_msgs__msg__VehicleImu__FIELD_NAME__gyro_calibration_count[] = "gyro_calibration_count";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleImu__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__accel_device_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__gyro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__delta_angle, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__delta_velocity, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__delta_angle_dt, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__delta_velocity_dt, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__delta_angle_clipping, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__delta_velocity_clipping, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__accel_calibration_count, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImu__FIELD_NAME__gyro_calibration_count, 22, 22},
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
px4_msgs__msg__VehicleImu__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleImu__TYPE_NAME, 23, 23},
      {px4_msgs__msg__VehicleImu__FIELDS, 12, 12},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# IMU readings in SI-unit form.\n"
  "\n"
  "uint64 timestamp          # time since system start (microseconds)\n"
  "uint64 timestamp_sample\n"
  "\n"
  "uint32 accel_device_id          # Accelerometer unique device ID for the sensor that does not change between power cycles\n"
  "uint32 gyro_device_id           # Gyroscope unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "float32[3] delta_angle          # delta angle about the FRD body frame XYZ-axis in rad over the integration time frame (delta_angle_dt)\n"
  "float32[3] delta_velocity       # delta velocity in the FRD body frame XYZ-axis in m/s over the integration time frame (delta_velocity_dt)\n"
  "\n"
  "uint32 delta_angle_dt           # integration period in microseconds\n"
  "uint32 delta_velocity_dt        # integration period in microseconds\n"
  "\n"
  "uint8 CLIPPING_X = 1\n"
  "uint8 CLIPPING_Y = 2\n"
  "uint8 CLIPPING_Z = 4\n"
  "\n"
  "uint8 delta_angle_clipping     # bitfield indicating if there was any gyro clipping (per axis) during the integration time frame\n"
  "uint8 delta_velocity_clipping   # bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame\n"
  "\n"
  "uint8 accel_calibration_count  \\t# Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.\n"
  "uint8 gyro_calibration_count   \\t# Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleImu__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleImu__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1371, 1371},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleImu__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleImu__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
