// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TransponderReport.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/transponder_report__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TransponderReport__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd1, 0x8d, 0xf2, 0x9c, 0xfc, 0x33, 0x2d, 0x45,
      0xf9, 0xce, 0xe3, 0x68, 0xe8, 0xf4, 0xec, 0xee,
      0x54, 0x78, 0xba, 0x03, 0xe8, 0x6e, 0xfd, 0x3a,
      0xd7, 0xbf, 0xf8, 0x46, 0x08, 0x7c, 0x1e, 0xb4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TransponderReport__TYPE_NAME[] = "px4_msgs/msg/TransponderReport";

// Define type names, field names, and default values
static char px4_msgs__msg__TransponderReport__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__icao_address[] = "icao_address";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__lat[] = "lat";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__lon[] = "lon";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__altitude_type[] = "altitude_type";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__altitude[] = "altitude";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__hor_velocity[] = "hor_velocity";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__ver_velocity[] = "ver_velocity";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__callsign[] = "callsign";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__emitter_type[] = "emitter_type";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__tslc[] = "tslc";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__flags[] = "flags";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__squawk[] = "squawk";
static char px4_msgs__msg__TransponderReport__FIELD_NAME__uas_id[] = "uas_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TransponderReport__FIELDS[] = {
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__icao_address, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__lat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__lon, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__altitude_type, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__hor_velocity, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__ver_velocity, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__callsign, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__emitter_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__tslc, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__flags, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__squawk, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TransponderReport__FIELD_NAME__uas_id, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      18,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TransponderReport__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TransponderReport__TYPE_NAME, 30, 30},
      {px4_msgs__msg__TransponderReport__FIELDS, 15, 15},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "uint32 icao_address \\t# ICAO address\n"
  "float64 lat \\t\\t# Latitude, expressed as degrees\n"
  "float64 lon \\t\\t# Longitude, expressed as degrees\n"
  "uint8 altitude_type\\t# Type from ADSB_ALTITUDE_TYPE enum\n"
  "float32 altitude \\t# Altitude(ASL) in meters\n"
  "float32 heading \\t# Course over ground in radians, 0 to 2pi, 0 is north\n"
  "float32 hor_velocity\\t# The horizontal velocity in m/s\n"
  "float32 ver_velocity \\t# The vertical velocity in m/s, positive is up\n"
  "char[9] callsign\\t# The callsign, 8+null\n"
  "uint8 emitter_type \\t# Type from ADSB_EMITTER_TYPE enum\n"
  "uint8 tslc \\t\\t# Time since last communication in seconds\n"
  "uint16 flags \\t\\t# Flags to indicate various statuses including valid data fields\n"
  "uint16 squawk \\t\\t# Squawk code\n"
  "uint8[18] uas_id\\t# Unique UAS ID\n"
  "\n"
  "# ADSB flags\n"
  "uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1\n"
  "uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2\n"
  "uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4\n"
  "uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8\n"
  "uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16\n"
  "uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32\n"
  "uint16 PX4_ADSB_FLAGS_RETRANSLATE = 256\n"
  "\n"
  "#ADSB Emitter Data:\n"
  "#from mavlink/v2.0/common/common.h\n"
  "uint16 ADSB_EMITTER_TYPE_NO_INFO=0\n"
  "uint16 ADSB_EMITTER_TYPE_LIGHT=1\n"
  "uint16 ADSB_EMITTER_TYPE_SMALL=2\n"
  "uint16 ADSB_EMITTER_TYPE_LARGE=3\n"
  "uint16 ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE=4\n"
  "uint16 ADSB_EMITTER_TYPE_HEAVY=5\n"
  "uint16 ADSB_EMITTER_TYPE_HIGHLY_MANUV=6\n"
  "uint16 ADSB_EMITTER_TYPE_ROTOCRAFT=7\n"
  "uint16 ADSB_EMITTER_TYPE_UNASSIGNED=8\n"
  "uint16 ADSB_EMITTER_TYPE_GLIDER=9\n"
  "uint16 ADSB_EMITTER_TYPE_LIGHTER_AIR=10\n"
  "uint16 ADSB_EMITTER_TYPE_PARACHUTE=11\n"
  "uint16 ADSB_EMITTER_TYPE_ULTRA_LIGHT=12\n"
  "uint16 ADSB_EMITTER_TYPE_UNASSIGNED2=13\n"
  "uint16 ADSB_EMITTER_TYPE_UAV=14\n"
  "uint16 ADSB_EMITTER_TYPE_SPACE=15\n"
  "uint16 ADSB_EMITTER_TYPE_UNASSGINED3=16\n"
  "uint16 ADSB_EMITTER_TYPE_EMERGENCY_SURFACE=17\n"
  "uint16 ADSB_EMITTER_TYPE_SERVICE_SURFACE=18\n"
  "uint16 ADSB_EMITTER_TYPE_POINT_OBSTACLE=19\n"
  "uint16 ADSB_EMITTER_TYPE_ENUM_END=20\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 16";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TransponderReport__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TransponderReport__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1954, 1954},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TransponderReport__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TransponderReport__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
