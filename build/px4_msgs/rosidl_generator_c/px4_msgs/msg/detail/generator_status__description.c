// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GeneratorStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/generator_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GeneratorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0xad, 0xba, 0x91, 0x57, 0x65, 0xcc, 0x7d,
      0xf8, 0xc4, 0x7b, 0x89, 0xf7, 0xdd, 0x53, 0xcd,
      0x4e, 0x08, 0x00, 0xea, 0x5d, 0xe6, 0x00, 0xe3,
      0xf2, 0x3e, 0x93, 0x98, 0x30, 0x84, 0xa6, 0xcd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GeneratorStatus__TYPE_NAME[] = "px4_msgs/msg/GeneratorStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__status[] = "status";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__battery_current[] = "battery_current";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__load_current[] = "load_current";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__power_generated[] = "power_generated";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__bus_voltage[] = "bus_voltage";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__bat_current_setpoint[] = "bat_current_setpoint";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__runtime[] = "runtime";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__time_until_maintenance[] = "time_until_maintenance";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__generator_speed[] = "generator_speed";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__rectifier_temperature[] = "rectifier_temperature";
static char px4_msgs__msg__GeneratorStatus__FIELD_NAME__generator_temperature[] = "generator_temperature";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GeneratorStatus__FIELDS[] = {
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__battery_current, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__load_current, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__power_generated, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__bus_voltage, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__bat_current_setpoint, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__runtime, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__time_until_maintenance, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__generator_speed, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__rectifier_temperature, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GeneratorStatus__FIELD_NAME__generator_temperature, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__GeneratorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GeneratorStatus__TYPE_NAME, 28, 28},
      {px4_msgs__msg__GeneratorStatus__FIELDS, 12, 12},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "\n"
  "uint64 STATUS_FLAG_OFF                              = 1       # Generator is off.\n"
  "uint64 STATUS_FLAG_READY                            = 2       # Generator is ready to start generating power.\n"
  "uint64 STATUS_FLAG_GENERATING                       = 4       # Generator is generating power.\n"
  "uint64 STATUS_FLAG_CHARGING                         = 8       # Generator is charging the batteries (generating enough power to charge and provide the load).\n"
  "uint64 STATUS_FLAG_REDUCED_POWER                    = 16      # Generator is operating at a reduced maximum power.\n"
  "uint64 STATUS_FLAG_MAXPOWER                         = 32      # Generator is providing the maximum output.\n"
  "uint64 STATUS_FLAG_OVERTEMP_WARNING                 = 64      # Generator is near the maximum operating temperature, cooling is insufficient.\n"
  "uint64 STATUS_FLAG_OVERTEMP_FAULT                   = 128     # Generator hit the maximum operating temperature and shutdown.\n"
  "uint64 STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING     = 256     # Power electronics are near the maximum operating temperature, cooling is insufficient.\n"
  "uint64 STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT       = 512     # Power electronics hit the maximum operating temperature and shutdown.\n"
  "uint64 STATUS_FLAG_ELECTRONICS_FAULT                = 1024    # Power electronics experienced a fault and shutdown.\n"
  "uint64 STATUS_FLAG_POWERSOURCE_FAULT                = 2048    # The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening.\n"
  "uint64 STATUS_FLAG_COMMUNICATION_WARNING            = 4096    # Generator controller having communication problems.\n"
  "uint64 STATUS_FLAG_COOLING_WARNING                  = 8192    # Power electronic or generator cooling system error.\n"
  "uint64 STATUS_FLAG_POWER_RAIL_FAULT                 = 16384   # Generator controller power rail experienced a fault.\n"
  "uint64 STATUS_FLAG_OVERCURRENT_FAULT                = 32768   # Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.\n"
  "uint64 STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT = 65536   # Generator controller detected a high current going into the batteries and shutdown to prevent battery damage. |\n"
  "uint64 STATUS_FLAG_OVERVOLTAGE_FAULT                = 131072  # Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating.\n"
  "uint64 STATUS_FLAG_BATTERY_UNDERVOLT_FAULT          = 262144  # Batteries are under voltage (generator will not start).\n"
  "uint64 STATUS_FLAG_START_INHIBITED                  = 524288  # Generator start is inhibited by e.g. a safety switch.\n"
  "uint64 STATUS_FLAG_MAINTENANCE_REQUIRED             = 1048576 # Generator requires maintenance.\n"
  "uint64 STATUS_FLAG_WARMING_UP                       = 2097152 # Generator is not ready to generate yet.\n"
  "uint64 STATUS_FLAG_IDLE                             = 4194304 # Generator is idle.\n"
  "\n"
  "uint64 status                      # Status flags\n"
  "\n"
  "\n"
  "float32 battery_current            # [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.\n"
  "float32 load_current               # [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided\n"
  "float32 power_generated            # [W] The power being generated. NaN: field not provided\n"
  "float32 bus_voltage                # [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.\n"
  "float32 bat_current_setpoint       # [A] The target battery current. Positive for out. Negative for in. NaN: field not provided\n"
  "\n"
  "uint32 runtime                     # [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.\n"
  "\n"
  "int32 time_until_maintenance       # [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.\n"
  "\n"
  "uint16 generator_speed             # [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.\n"
  "\n"
  "int16 rectifier_temperature        # [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.\n"
  "int16 generator_temperature        # [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GeneratorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GeneratorStatus__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 4506, 4506},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GeneratorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GeneratorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
