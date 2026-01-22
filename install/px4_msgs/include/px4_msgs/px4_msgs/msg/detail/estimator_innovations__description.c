// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorInnovations.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_innovations__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorInnovations__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x32, 0x75, 0x6c, 0x14, 0xbe, 0x46, 0xda, 0x33,
      0x3c, 0xb5, 0x59, 0x3f, 0x1c, 0x3a, 0x67, 0x5f,
      0x48, 0x36, 0x01, 0x3f, 0x5f, 0x7a, 0x94, 0x2f,
      0x0a, 0x07, 0x34, 0xe2, 0x55, 0x9a, 0x52, 0x9e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorInnovations__TYPE_NAME[] = "px4_msgs/msg/EstimatorInnovations";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_hvel[] = "gps_hvel";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_vvel[] = "gps_vvel";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_hpos[] = "gps_hpos";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_vpos[] = "gps_vpos";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_hvel[] = "ev_hvel";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_vvel[] = "ev_vvel";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_hpos[] = "ev_hpos";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_vpos[] = "ev_vpos";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__rng_vpos[] = "rng_vpos";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__baro_vpos[] = "baro_vpos";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__aux_hvel[] = "aux_hvel";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__flow[] = "flow";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__mag_field[] = "mag_field";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gravity[] = "gravity";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__drag[] = "drag";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__airspeed[] = "airspeed";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__beta[] = "beta";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__hagl[] = "hagl";
static char px4_msgs__msg__EstimatorInnovations__FIELD_NAME__hagl_rate[] = "hagl_rate";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorInnovations__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_hvel, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_vvel, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_hpos, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gps_vpos, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_hvel, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_vvel, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_hpos, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__ev_vpos, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__rng_vpos, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__baro_vpos, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__aux_hvel, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__flow, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__mag_field, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__gravity, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__drag, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__airspeed, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__beta, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__hagl, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorInnovations__FIELD_NAME__hagl_rate, 9, 9},
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
px4_msgs__msg__EstimatorInnovations__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorInnovations__TYPE_NAME, 33, 33},
      {px4_msgs__msg__EstimatorInnovations__FIELDS, 22, 22},
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
  "uint64 timestamp_sample         # the timestamp of the raw data (microseconds)\n"
  "\n"
  "# GPS\n"
  "float32[2] gps_hvel\\t# horizontal GPS velocity innovation (m/sec) and innovation variance ((m/sec)**2)\n"
  "float32    gps_vvel\\t# vertical GPS velocity innovation (m/sec) and innovation variance ((m/sec)**2)\n"
  "float32[2] gps_hpos\\t# horizontal GPS position innovation (m) and innovation variance (m**2)\n"
  "float32    gps_vpos\\t# vertical GPS position innovation (m) and innovation variance (m**2)\n"
  "\n"
  "# External Vision\n"
  "float32[2] ev_hvel\\t# horizontal external vision velocity innovation (m/sec) and innovation variance ((m/sec)**2)\n"
  "float32    ev_vvel\\t# vertical external vision velocity innovation (m/sec) and innovation variance ((m/sec)**2)\n"
  "float32[2] ev_hpos\\t# horizontal external vision position innovation (m) and innovation variance (m**2)\n"
  "float32    ev_vpos\\t# vertical external vision position innovation (m) and innovation variance (m**2)\n"
  "\n"
  "# Height sensors\n"
  "float32 rng_vpos\\t# range sensor height innovation (m) and innovation variance (m**2)\n"
  "float32 baro_vpos\\t# barometer height innovation (m) and innovation variance (m**2)\n"
  "\n"
  "# Auxiliary velocity\n"
  "float32[2] aux_hvel\\t# horizontal auxiliary velocity innovation from landing target measurement (m/sec) and innovation variance ((m/sec)**2)\n"
  "\n"
  "# Optical flow\n"
  "float32[2] flow\\t\\t# flow innvoation (rad/sec) and innovation variance ((rad/sec)**2)\n"
  "\n"
  "# Various\n"
  "float32 heading\\t\\t# heading innovation (rad) and innovation variance (rad**2)\n"
  "float32[3] mag_field\\t# earth magnetic field innovation (Gauss) and innovation variance (Gauss**2)\n"
  "float32[3] gravity\\t# gravity innovation from accelerometerr vector (m/s**2)\n"
  "float32[2] drag\\t\\t# drag specific force innovation (m/sec**2) and innovation variance ((m/sec)**2)\n"
  "float32 airspeed\\t# airspeed innovation (m/sec) and innovation variance ((m/sec)**2)\n"
  "float32 beta\\t\\t# synthetic sideslip innovation (rad) and innovation variance (rad**2)\n"
  "float32 hagl\\t\\t# height of ground innovation (m) and innovation variance (m**2)\n"
  "float32 hagl_rate\\t# height of ground rate innovation (m/s) and innovation variance ((m/s)**2)\n"
  "\n"
  "# The innovation test ratios are scalar values. In case the field is a vector,\n"
  "# the test ratio will be put in the first component of the vector.\n"
  "\n"
  "# TOPICS estimator_innovations estimator_innovation_variances estimator_innovation_test_ratios";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorInnovations__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorInnovations__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2369, 2369},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorInnovations__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorInnovations__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
