// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FuelTankStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/fuel_tank_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FuelTankStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xae, 0xea, 0xae, 0x3c, 0xd2, 0x15, 0xdd, 0x95,
      0xa4, 0x5b, 0x0b, 0xb4, 0x42, 0x7b, 0x6f, 0x74,
      0xd9, 0xcb, 0x3c, 0x6f, 0x83, 0x79, 0x27, 0xf8,
      0x98, 0x32, 0x1e, 0x4b, 0x33, 0x00, 0xfc, 0x02,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FuelTankStatus__TYPE_NAME[] = "px4_msgs/msg/FuelTankStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__maximum_fuel_capacity[] = "maximum_fuel_capacity";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__consumed_fuel[] = "consumed_fuel";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__fuel_consumption_rate[] = "fuel_consumption_rate";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__percent_remaining[] = "percent_remaining";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__remaining_fuel[] = "remaining_fuel";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__fuel_tank_id[] = "fuel_tank_id";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__fuel_type[] = "fuel_type";
static char px4_msgs__msg__FuelTankStatus__FIELD_NAME__temperature[] = "temperature";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FuelTankStatus__FIELDS[] = {
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__maximum_fuel_capacity, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__consumed_fuel, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__fuel_consumption_rate, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__percent_remaining, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__remaining_fuel, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__fuel_tank_id, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__fuel_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FuelTankStatus__FIELD_NAME__temperature, 11, 11},
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
px4_msgs__msg__FuelTankStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FuelTankStatus__TYPE_NAME, 27, 27},
      {px4_msgs__msg__FuelTankStatus__FIELDS, 9, 9},
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
  "float32 maximum_fuel_capacity       \\t# maximum fuel capacity. Must always be provided, either from the driver or a parameter\n"
  "float32 consumed_fuel       \\t\\t# consumed fuel, NaN if not measured. Should not be inferred from the max fuel capacity\n"
  "float32 fuel_consumption_rate     \\t# fuel consumption rate, NaN if not measured\n"
  "\n"
  "uint8 percent_remaining                 # percentage of remaining fuel, UINT8_MAX if not provided\n"
  "float32 remaining_fuel      \\t\\t# remaining fuel, NaN if not measured. Should not be inferred from the max fuel capacity\n"
  "\n"
  "uint8 fuel_tank_id                      # identifier for the fuel tank. Must match ID of other messages for same fuel system. 0 by default when only a single tank exists\n"
  "\n"
  "uint32 fuel_type                        # type of fuel based on MAV_FUEL_TYPE enum. Set to MAV_FUEL_TYPE_UNKNOWN if unknown or it does not fit the provided types\n"
  "uint8 MAV_FUEL_TYPE_UNKNOWN = 0\\t\\t# fuel type not specified. Fuel levels are normalized (i.e., maximum is 1, and other levels are relative to 1).\n"
  "uint8 MAV_FUEL_TYPE_LIQUID = 1\\t\\t# represents generic liquid fuels, such as gasoline or diesel. Fuel levels are measured in millilitres (ml), and flow rates in millilitres per second (ml/s).\n"
  "uint8 MAV_FUEL_TYPE_GAS = 2\\t\\t# represents a gas fuel, such as hydrogen, methane, or propane. Fuel levels are in kilo-Pascal (kPa), and flow rates are in milliliters per second (ml/s).\n"
  "\n"
  "float32 temperature                     # fuel temperature in Kelvin, NaN if not measured";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FuelTankStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FuelTankStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1567, 1567},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FuelTankStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FuelTankStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
