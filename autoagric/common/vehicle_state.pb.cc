// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: autoagric/common/vehicle_state.proto

#include "autoagric/common/vehicle_state.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace autoagric {
namespace common {
constexpr VehicleState::VehicleState(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : pose_(nullptr)
  , x_(0)
  , y_(0)
  , z_(0)
  , timestamp_(0)
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , heading_(0)
  , kappa_(0)
  , linear_velocity_(0)
  , angular_velocity_(0)
  , linear_acceleration_(0)
  , gear_(0)

  , driving_mode_(0)

  , steering_percentage_(0){}
struct VehicleStateDefaultTypeInternal {
  constexpr VehicleStateDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~VehicleStateDefaultTypeInternal() {}
  union {
    VehicleState _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT VehicleStateDefaultTypeInternal _VehicleState_default_instance_;
}  // namespace common
}  // namespace autoagric
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_autoagric_2fcommon_2fvehicle_5fstate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_autoagric_2fcommon_2fvehicle_5fstate_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_autoagric_2fcommon_2fvehicle_5fstate_2eproto = nullptr;

const uint32_t TableStruct_autoagric_2fcommon_2fvehicle_5fstate_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, x_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, y_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, z_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, timestamp_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, roll_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, pitch_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, yaw_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, heading_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, kappa_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, linear_velocity_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, angular_velocity_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, linear_acceleration_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, gear_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, driving_mode_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, pose_),
  PROTOBUF_FIELD_OFFSET(::autoagric::common::VehicleState, steering_percentage_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::autoagric::common::VehicleState)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::autoagric::common::_VehicleState_default_instance_),
};

const char descriptor_table_protodef_autoagric_2fcommon_2fvehicle_5fstate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$autoagric/common/vehicle_state.proto\022\020"
  "autoagric.common\032\036autoagric/canbus/chass"
  "is.proto\032!autoagric/localization/pose.pr"
  "oto\"\230\003\n\014VehicleState\022\t\n\001x\030\001 \001(\001\022\t\n\001y\030\002 \001"
  "(\001\022\t\n\001z\030\003 \001(\001\022\021\n\ttimestamp\030\004 \001(\001\022\014\n\004roll"
  "\030\005 \001(\001\022\r\n\005pitch\030\006 \001(\001\022\013\n\003yaw\030\007 \001(\001\022\017\n\007he"
  "ading\030\010 \001(\001\022\r\n\005kappa\030\t \001(\001\022\027\n\017linear_vel"
  "ocity\030\n \001(\001\022\030\n\020angular_velocity\030\013 \001(\001\022\033\n"
  "\023linear_acceleration\030\014 \001(\001\0224\n\004gear\030\r \001(\016"
  "2&.autoagric.canbus.Chassis.GearPosition"
  "\022;\n\014driving_mode\030\016 \001(\0162%.autoagric.canbu"
  "s.Chassis.DrivingMode\022*\n\004pose\030\017 \001(\0132\034.au"
  "toagric.localization.Pose\022\033\n\023steering_pe"
  "rcentage\030\020 \001(\001b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_deps[2] = {
  &::descriptor_table_autoagric_2fcanbus_2fchassis_2eproto,
  &::descriptor_table_autoagric_2flocalization_2fpose_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto = {
  false, false, 542, descriptor_table_protodef_autoagric_2fcommon_2fvehicle_5fstate_2eproto, "autoagric/common/vehicle_state.proto", 
  &descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_once, descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_autoagric_2fcommon_2fvehicle_5fstate_2eproto::offsets,
  file_level_metadata_autoagric_2fcommon_2fvehicle_5fstate_2eproto, file_level_enum_descriptors_autoagric_2fcommon_2fvehicle_5fstate_2eproto, file_level_service_descriptors_autoagric_2fcommon_2fvehicle_5fstate_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_getter() {
  return &descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_autoagric_2fcommon_2fvehicle_5fstate_2eproto(&descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto);
namespace autoagric {
namespace common {

// ===================================================================

class VehicleState::_Internal {
 public:
  static const ::autoagric::localization::Pose& pose(const VehicleState* msg);
};

const ::autoagric::localization::Pose&
VehicleState::_Internal::pose(const VehicleState* msg) {
  return *msg->pose_;
}
void VehicleState::clear_pose() {
  if (GetArenaForAllocation() == nullptr && pose_ != nullptr) {
    delete pose_;
  }
  pose_ = nullptr;
}
VehicleState::VehicleState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:autoagric.common.VehicleState)
}
VehicleState::VehicleState(const VehicleState& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_pose()) {
    pose_ = new ::autoagric::localization::Pose(*from.pose_);
  } else {
    pose_ = nullptr;
  }
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&steering_percentage_) -
    reinterpret_cast<char*>(&x_)) + sizeof(steering_percentage_));
  // @@protoc_insertion_point(copy_constructor:autoagric.common.VehicleState)
}

inline void VehicleState::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&pose_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&steering_percentage_) -
    reinterpret_cast<char*>(&pose_)) + sizeof(steering_percentage_));
}

VehicleState::~VehicleState() {
  // @@protoc_insertion_point(destructor:autoagric.common.VehicleState)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void VehicleState::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete pose_;
}

void VehicleState::ArenaDtor(void* object) {
  VehicleState* _this = reinterpret_cast< VehicleState* >(object);
  (void)_this;
}
void VehicleState::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void VehicleState::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void VehicleState::Clear() {
// @@protoc_insertion_point(message_clear_start:autoagric.common.VehicleState)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && pose_ != nullptr) {
    delete pose_;
  }
  pose_ = nullptr;
  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&steering_percentage_) -
      reinterpret_cast<char*>(&x_)) + sizeof(steering_percentage_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* VehicleState::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double x = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 9)) {
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double y = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 17)) {
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double z = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 25)) {
          z_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double timestamp = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 33)) {
          timestamp_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double roll = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 41)) {
          roll_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double pitch = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 49)) {
          pitch_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double yaw = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 57)) {
          yaw_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double heading = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 65)) {
          heading_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double kappa = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 73)) {
          kappa_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double linear_velocity = 10;
      case 10:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 81)) {
          linear_velocity_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double angular_velocity = 11;
      case 11:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 89)) {
          angular_velocity_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double linear_acceleration = 12;
      case 12:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 97)) {
          linear_acceleration_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // .autoagric.canbus.Chassis.GearPosition gear = 13;
      case 13:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 104)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_gear(static_cast<::autoagric::canbus::Chassis_GearPosition>(val));
        } else
          goto handle_unusual;
        continue;
      // .autoagric.canbus.Chassis.DrivingMode driving_mode = 14;
      case 14:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 112)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_driving_mode(static_cast<::autoagric::canbus::Chassis_DrivingMode>(val));
        } else
          goto handle_unusual;
        continue;
      // .autoagric.localization.Pose pose = 15;
      case 15:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 122)) {
          ptr = ctx->ParseMessage(_internal_mutable_pose(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // double steering_percentage = 16;
      case 16:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 129)) {
          steering_percentage_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* VehicleState::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autoagric.common.VehicleState)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // double x = 1;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_x = this->_internal_x();
  uint64_t raw_x;
  memcpy(&raw_x, &tmp_x, sizeof(tmp_x));
  if (raw_x != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_x(), target);
  }

  // double y = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_y = this->_internal_y();
  uint64_t raw_y;
  memcpy(&raw_y, &tmp_y, sizeof(tmp_y));
  if (raw_y != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_y(), target);
  }

  // double z = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_z = this->_internal_z();
  uint64_t raw_z;
  memcpy(&raw_z, &tmp_z, sizeof(tmp_z));
  if (raw_z != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_z(), target);
  }

  // double timestamp = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_timestamp = this->_internal_timestamp();
  uint64_t raw_timestamp;
  memcpy(&raw_timestamp, &tmp_timestamp, sizeof(tmp_timestamp));
  if (raw_timestamp != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_timestamp(), target);
  }

  // double roll = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_roll = this->_internal_roll();
  uint64_t raw_roll;
  memcpy(&raw_roll, &tmp_roll, sizeof(tmp_roll));
  if (raw_roll != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_roll(), target);
  }

  // double pitch = 6;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_pitch = this->_internal_pitch();
  uint64_t raw_pitch;
  memcpy(&raw_pitch, &tmp_pitch, sizeof(tmp_pitch));
  if (raw_pitch != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(6, this->_internal_pitch(), target);
  }

  // double yaw = 7;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_yaw = this->_internal_yaw();
  uint64_t raw_yaw;
  memcpy(&raw_yaw, &tmp_yaw, sizeof(tmp_yaw));
  if (raw_yaw != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(7, this->_internal_yaw(), target);
  }

  // double heading = 8;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_heading = this->_internal_heading();
  uint64_t raw_heading;
  memcpy(&raw_heading, &tmp_heading, sizeof(tmp_heading));
  if (raw_heading != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(8, this->_internal_heading(), target);
  }

  // double kappa = 9;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kappa = this->_internal_kappa();
  uint64_t raw_kappa;
  memcpy(&raw_kappa, &tmp_kappa, sizeof(tmp_kappa));
  if (raw_kappa != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(9, this->_internal_kappa(), target);
  }

  // double linear_velocity = 10;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_linear_velocity = this->_internal_linear_velocity();
  uint64_t raw_linear_velocity;
  memcpy(&raw_linear_velocity, &tmp_linear_velocity, sizeof(tmp_linear_velocity));
  if (raw_linear_velocity != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(10, this->_internal_linear_velocity(), target);
  }

  // double angular_velocity = 11;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_angular_velocity = this->_internal_angular_velocity();
  uint64_t raw_angular_velocity;
  memcpy(&raw_angular_velocity, &tmp_angular_velocity, sizeof(tmp_angular_velocity));
  if (raw_angular_velocity != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(11, this->_internal_angular_velocity(), target);
  }

  // double linear_acceleration = 12;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_linear_acceleration = this->_internal_linear_acceleration();
  uint64_t raw_linear_acceleration;
  memcpy(&raw_linear_acceleration, &tmp_linear_acceleration, sizeof(tmp_linear_acceleration));
  if (raw_linear_acceleration != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(12, this->_internal_linear_acceleration(), target);
  }

  // .autoagric.canbus.Chassis.GearPosition gear = 13;
  if (this->_internal_gear() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      13, this->_internal_gear(), target);
  }

  // .autoagric.canbus.Chassis.DrivingMode driving_mode = 14;
  if (this->_internal_driving_mode() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      14, this->_internal_driving_mode(), target);
  }

  // .autoagric.localization.Pose pose = 15;
  if (this->_internal_has_pose()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        15, _Internal::pose(this), target, stream);
  }

  // double steering_percentage = 16;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_steering_percentage = this->_internal_steering_percentage();
  uint64_t raw_steering_percentage;
  memcpy(&raw_steering_percentage, &tmp_steering_percentage, sizeof(tmp_steering_percentage));
  if (raw_steering_percentage != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(16, this->_internal_steering_percentage(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autoagric.common.VehicleState)
  return target;
}

size_t VehicleState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autoagric.common.VehicleState)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .autoagric.localization.Pose pose = 15;
  if (this->_internal_has_pose()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *pose_);
  }

  // double x = 1;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_x = this->_internal_x();
  uint64_t raw_x;
  memcpy(&raw_x, &tmp_x, sizeof(tmp_x));
  if (raw_x != 0) {
    total_size += 1 + 8;
  }

  // double y = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_y = this->_internal_y();
  uint64_t raw_y;
  memcpy(&raw_y, &tmp_y, sizeof(tmp_y));
  if (raw_y != 0) {
    total_size += 1 + 8;
  }

  // double z = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_z = this->_internal_z();
  uint64_t raw_z;
  memcpy(&raw_z, &tmp_z, sizeof(tmp_z));
  if (raw_z != 0) {
    total_size += 1 + 8;
  }

  // double timestamp = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_timestamp = this->_internal_timestamp();
  uint64_t raw_timestamp;
  memcpy(&raw_timestamp, &tmp_timestamp, sizeof(tmp_timestamp));
  if (raw_timestamp != 0) {
    total_size += 1 + 8;
  }

  // double roll = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_roll = this->_internal_roll();
  uint64_t raw_roll;
  memcpy(&raw_roll, &tmp_roll, sizeof(tmp_roll));
  if (raw_roll != 0) {
    total_size += 1 + 8;
  }

  // double pitch = 6;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_pitch = this->_internal_pitch();
  uint64_t raw_pitch;
  memcpy(&raw_pitch, &tmp_pitch, sizeof(tmp_pitch));
  if (raw_pitch != 0) {
    total_size += 1 + 8;
  }

  // double yaw = 7;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_yaw = this->_internal_yaw();
  uint64_t raw_yaw;
  memcpy(&raw_yaw, &tmp_yaw, sizeof(tmp_yaw));
  if (raw_yaw != 0) {
    total_size += 1 + 8;
  }

  // double heading = 8;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_heading = this->_internal_heading();
  uint64_t raw_heading;
  memcpy(&raw_heading, &tmp_heading, sizeof(tmp_heading));
  if (raw_heading != 0) {
    total_size += 1 + 8;
  }

  // double kappa = 9;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kappa = this->_internal_kappa();
  uint64_t raw_kappa;
  memcpy(&raw_kappa, &tmp_kappa, sizeof(tmp_kappa));
  if (raw_kappa != 0) {
    total_size += 1 + 8;
  }

  // double linear_velocity = 10;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_linear_velocity = this->_internal_linear_velocity();
  uint64_t raw_linear_velocity;
  memcpy(&raw_linear_velocity, &tmp_linear_velocity, sizeof(tmp_linear_velocity));
  if (raw_linear_velocity != 0) {
    total_size += 1 + 8;
  }

  // double angular_velocity = 11;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_angular_velocity = this->_internal_angular_velocity();
  uint64_t raw_angular_velocity;
  memcpy(&raw_angular_velocity, &tmp_angular_velocity, sizeof(tmp_angular_velocity));
  if (raw_angular_velocity != 0) {
    total_size += 1 + 8;
  }

  // double linear_acceleration = 12;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_linear_acceleration = this->_internal_linear_acceleration();
  uint64_t raw_linear_acceleration;
  memcpy(&raw_linear_acceleration, &tmp_linear_acceleration, sizeof(tmp_linear_acceleration));
  if (raw_linear_acceleration != 0) {
    total_size += 1 + 8;
  }

  // .autoagric.canbus.Chassis.GearPosition gear = 13;
  if (this->_internal_gear() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_gear());
  }

  // .autoagric.canbus.Chassis.DrivingMode driving_mode = 14;
  if (this->_internal_driving_mode() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_driving_mode());
  }

  // double steering_percentage = 16;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_steering_percentage = this->_internal_steering_percentage();
  uint64_t raw_steering_percentage;
  memcpy(&raw_steering_percentage, &tmp_steering_percentage, sizeof(tmp_steering_percentage));
  if (raw_steering_percentage != 0) {
    total_size += 2 + 8;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData VehicleState::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    VehicleState::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*VehicleState::GetClassData() const { return &_class_data_; }

void VehicleState::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<VehicleState *>(to)->MergeFrom(
      static_cast<const VehicleState &>(from));
}


void VehicleState::MergeFrom(const VehicleState& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:autoagric.common.VehicleState)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_pose()) {
    _internal_mutable_pose()->::autoagric::localization::Pose::MergeFrom(from._internal_pose());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_x = from._internal_x();
  uint64_t raw_x;
  memcpy(&raw_x, &tmp_x, sizeof(tmp_x));
  if (raw_x != 0) {
    _internal_set_x(from._internal_x());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_y = from._internal_y();
  uint64_t raw_y;
  memcpy(&raw_y, &tmp_y, sizeof(tmp_y));
  if (raw_y != 0) {
    _internal_set_y(from._internal_y());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_z = from._internal_z();
  uint64_t raw_z;
  memcpy(&raw_z, &tmp_z, sizeof(tmp_z));
  if (raw_z != 0) {
    _internal_set_z(from._internal_z());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_timestamp = from._internal_timestamp();
  uint64_t raw_timestamp;
  memcpy(&raw_timestamp, &tmp_timestamp, sizeof(tmp_timestamp));
  if (raw_timestamp != 0) {
    _internal_set_timestamp(from._internal_timestamp());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_roll = from._internal_roll();
  uint64_t raw_roll;
  memcpy(&raw_roll, &tmp_roll, sizeof(tmp_roll));
  if (raw_roll != 0) {
    _internal_set_roll(from._internal_roll());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_pitch = from._internal_pitch();
  uint64_t raw_pitch;
  memcpy(&raw_pitch, &tmp_pitch, sizeof(tmp_pitch));
  if (raw_pitch != 0) {
    _internal_set_pitch(from._internal_pitch());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_yaw = from._internal_yaw();
  uint64_t raw_yaw;
  memcpy(&raw_yaw, &tmp_yaw, sizeof(tmp_yaw));
  if (raw_yaw != 0) {
    _internal_set_yaw(from._internal_yaw());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_heading = from._internal_heading();
  uint64_t raw_heading;
  memcpy(&raw_heading, &tmp_heading, sizeof(tmp_heading));
  if (raw_heading != 0) {
    _internal_set_heading(from._internal_heading());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kappa = from._internal_kappa();
  uint64_t raw_kappa;
  memcpy(&raw_kappa, &tmp_kappa, sizeof(tmp_kappa));
  if (raw_kappa != 0) {
    _internal_set_kappa(from._internal_kappa());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_linear_velocity = from._internal_linear_velocity();
  uint64_t raw_linear_velocity;
  memcpy(&raw_linear_velocity, &tmp_linear_velocity, sizeof(tmp_linear_velocity));
  if (raw_linear_velocity != 0) {
    _internal_set_linear_velocity(from._internal_linear_velocity());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_angular_velocity = from._internal_angular_velocity();
  uint64_t raw_angular_velocity;
  memcpy(&raw_angular_velocity, &tmp_angular_velocity, sizeof(tmp_angular_velocity));
  if (raw_angular_velocity != 0) {
    _internal_set_angular_velocity(from._internal_angular_velocity());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_linear_acceleration = from._internal_linear_acceleration();
  uint64_t raw_linear_acceleration;
  memcpy(&raw_linear_acceleration, &tmp_linear_acceleration, sizeof(tmp_linear_acceleration));
  if (raw_linear_acceleration != 0) {
    _internal_set_linear_acceleration(from._internal_linear_acceleration());
  }
  if (from._internal_gear() != 0) {
    _internal_set_gear(from._internal_gear());
  }
  if (from._internal_driving_mode() != 0) {
    _internal_set_driving_mode(from._internal_driving_mode());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_steering_percentage = from._internal_steering_percentage();
  uint64_t raw_steering_percentage;
  memcpy(&raw_steering_percentage, &tmp_steering_percentage, sizeof(tmp_steering_percentage));
  if (raw_steering_percentage != 0) {
    _internal_set_steering_percentage(from._internal_steering_percentage());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void VehicleState::CopyFrom(const VehicleState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autoagric.common.VehicleState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleState::IsInitialized() const {
  return true;
}

void VehicleState::InternalSwap(VehicleState* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(VehicleState, steering_percentage_)
      + sizeof(VehicleState::steering_percentage_)
      - PROTOBUF_FIELD_OFFSET(VehicleState, pose_)>(
          reinterpret_cast<char*>(&pose_),
          reinterpret_cast<char*>(&other->pose_));
}

::PROTOBUF_NAMESPACE_ID::Metadata VehicleState::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_getter, &descriptor_table_autoagric_2fcommon_2fvehicle_5fstate_2eproto_once,
      file_level_metadata_autoagric_2fcommon_2fvehicle_5fstate_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace autoagric
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::autoagric::common::VehicleState* Arena::CreateMaybeMessage< ::autoagric::common::VehicleState >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autoagric::common::VehicleState >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
