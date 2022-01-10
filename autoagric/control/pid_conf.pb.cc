// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: autoagric/control/pid_conf.proto

#include "autoagric/control/pid_conf.pb.h"

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
namespace control {
constexpr PidConf::PidConf(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : integrator_saturation_level_(0)
  , kp_(0)
  , ki_(0)
  , kd_(0)
  , kaw_(0)
  , output_saturation_level_(0)
  , integrator_enable_(false){}
struct PidConfDefaultTypeInternal {
  constexpr PidConfDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~PidConfDefaultTypeInternal() {}
  union {
    PidConf _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PidConfDefaultTypeInternal _PidConf_default_instance_;
}  // namespace control
}  // namespace autoagric
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_autoagric_2fcontrol_2fpid_5fconf_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_autoagric_2fcontrol_2fpid_5fconf_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_autoagric_2fcontrol_2fpid_5fconf_2eproto = nullptr;

const uint32_t TableStruct_autoagric_2fcontrol_2fpid_5fconf_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, integrator_enable_),
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, integrator_saturation_level_),
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, kp_),
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, ki_),
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, kd_),
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, kaw_),
  PROTOBUF_FIELD_OFFSET(::autoagric::control::PidConf, output_saturation_level_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::autoagric::control::PidConf)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::autoagric::control::_PidConf_default_instance_),
};

const char descriptor_table_protodef_autoagric_2fcontrol_2fpid_5fconf_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n autoagric/control/pid_conf.proto\022\021auto"
  "agric.control\"\233\001\n\007PidConf\022\031\n\021integrator_"
  "enable\030\001 \001(\010\022#\n\033integrator_saturation_le"
  "vel\030\002 \001(\001\022\n\n\002kp\030\003 \001(\001\022\n\n\002ki\030\004 \001(\001\022\n\n\002kd\030"
  "\005 \001(\001\022\013\n\003kaw\030\006 \001(\001\022\037\n\027output_saturation_"
  "level\030\007 \001(\001b\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto = {
  false, false, 219, descriptor_table_protodef_autoagric_2fcontrol_2fpid_5fconf_2eproto, "autoagric/control/pid_conf.proto", 
  &descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_autoagric_2fcontrol_2fpid_5fconf_2eproto::offsets,
  file_level_metadata_autoagric_2fcontrol_2fpid_5fconf_2eproto, file_level_enum_descriptors_autoagric_2fcontrol_2fpid_5fconf_2eproto, file_level_service_descriptors_autoagric_2fcontrol_2fpid_5fconf_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto_getter() {
  return &descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_autoagric_2fcontrol_2fpid_5fconf_2eproto(&descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto);
namespace autoagric {
namespace control {

// ===================================================================

class PidConf::_Internal {
 public:
};

PidConf::PidConf(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:autoagric.control.PidConf)
}
PidConf::PidConf(const PidConf& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&integrator_saturation_level_, &from.integrator_saturation_level_,
    static_cast<size_t>(reinterpret_cast<char*>(&integrator_enable_) -
    reinterpret_cast<char*>(&integrator_saturation_level_)) + sizeof(integrator_enable_));
  // @@protoc_insertion_point(copy_constructor:autoagric.control.PidConf)
}

inline void PidConf::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&integrator_saturation_level_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&integrator_enable_) -
    reinterpret_cast<char*>(&integrator_saturation_level_)) + sizeof(integrator_enable_));
}

PidConf::~PidConf() {
  // @@protoc_insertion_point(destructor:autoagric.control.PidConf)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void PidConf::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void PidConf::ArenaDtor(void* object) {
  PidConf* _this = reinterpret_cast< PidConf* >(object);
  (void)_this;
}
void PidConf::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void PidConf::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void PidConf::Clear() {
// @@protoc_insertion_point(message_clear_start:autoagric.control.PidConf)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&integrator_saturation_level_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&integrator_enable_) -
      reinterpret_cast<char*>(&integrator_saturation_level_)) + sizeof(integrator_enable_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PidConf::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // bool integrator_enable = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          integrator_enable_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // double integrator_saturation_level = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 17)) {
          integrator_saturation_level_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double kp = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 25)) {
          kp_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double ki = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 33)) {
          ki_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double kd = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 41)) {
          kd_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double kaw = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 49)) {
          kaw_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double output_saturation_level = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 57)) {
          output_saturation_level_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
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

uint8_t* PidConf::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autoagric.control.PidConf)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // bool integrator_enable = 1;
  if (this->_internal_integrator_enable() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1, this->_internal_integrator_enable(), target);
  }

  // double integrator_saturation_level = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_integrator_saturation_level = this->_internal_integrator_saturation_level();
  uint64_t raw_integrator_saturation_level;
  memcpy(&raw_integrator_saturation_level, &tmp_integrator_saturation_level, sizeof(tmp_integrator_saturation_level));
  if (raw_integrator_saturation_level != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_integrator_saturation_level(), target);
  }

  // double kp = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kp = this->_internal_kp();
  uint64_t raw_kp;
  memcpy(&raw_kp, &tmp_kp, sizeof(tmp_kp));
  if (raw_kp != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_kp(), target);
  }

  // double ki = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_ki = this->_internal_ki();
  uint64_t raw_ki;
  memcpy(&raw_ki, &tmp_ki, sizeof(tmp_ki));
  if (raw_ki != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_ki(), target);
  }

  // double kd = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kd = this->_internal_kd();
  uint64_t raw_kd;
  memcpy(&raw_kd, &tmp_kd, sizeof(tmp_kd));
  if (raw_kd != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_kd(), target);
  }

  // double kaw = 6;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kaw = this->_internal_kaw();
  uint64_t raw_kaw;
  memcpy(&raw_kaw, &tmp_kaw, sizeof(tmp_kaw));
  if (raw_kaw != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(6, this->_internal_kaw(), target);
  }

  // double output_saturation_level = 7;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_output_saturation_level = this->_internal_output_saturation_level();
  uint64_t raw_output_saturation_level;
  memcpy(&raw_output_saturation_level, &tmp_output_saturation_level, sizeof(tmp_output_saturation_level));
  if (raw_output_saturation_level != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(7, this->_internal_output_saturation_level(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autoagric.control.PidConf)
  return target;
}

size_t PidConf::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autoagric.control.PidConf)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double integrator_saturation_level = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_integrator_saturation_level = this->_internal_integrator_saturation_level();
  uint64_t raw_integrator_saturation_level;
  memcpy(&raw_integrator_saturation_level, &tmp_integrator_saturation_level, sizeof(tmp_integrator_saturation_level));
  if (raw_integrator_saturation_level != 0) {
    total_size += 1 + 8;
  }

  // double kp = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kp = this->_internal_kp();
  uint64_t raw_kp;
  memcpy(&raw_kp, &tmp_kp, sizeof(tmp_kp));
  if (raw_kp != 0) {
    total_size += 1 + 8;
  }

  // double ki = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_ki = this->_internal_ki();
  uint64_t raw_ki;
  memcpy(&raw_ki, &tmp_ki, sizeof(tmp_ki));
  if (raw_ki != 0) {
    total_size += 1 + 8;
  }

  // double kd = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kd = this->_internal_kd();
  uint64_t raw_kd;
  memcpy(&raw_kd, &tmp_kd, sizeof(tmp_kd));
  if (raw_kd != 0) {
    total_size += 1 + 8;
  }

  // double kaw = 6;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kaw = this->_internal_kaw();
  uint64_t raw_kaw;
  memcpy(&raw_kaw, &tmp_kaw, sizeof(tmp_kaw));
  if (raw_kaw != 0) {
    total_size += 1 + 8;
  }

  // double output_saturation_level = 7;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_output_saturation_level = this->_internal_output_saturation_level();
  uint64_t raw_output_saturation_level;
  memcpy(&raw_output_saturation_level, &tmp_output_saturation_level, sizeof(tmp_output_saturation_level));
  if (raw_output_saturation_level != 0) {
    total_size += 1 + 8;
  }

  // bool integrator_enable = 1;
  if (this->_internal_integrator_enable() != 0) {
    total_size += 1 + 1;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData PidConf::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    PidConf::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*PidConf::GetClassData() const { return &_class_data_; }

void PidConf::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<PidConf *>(to)->MergeFrom(
      static_cast<const PidConf &>(from));
}


void PidConf::MergeFrom(const PidConf& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:autoagric.control.PidConf)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_integrator_saturation_level = from._internal_integrator_saturation_level();
  uint64_t raw_integrator_saturation_level;
  memcpy(&raw_integrator_saturation_level, &tmp_integrator_saturation_level, sizeof(tmp_integrator_saturation_level));
  if (raw_integrator_saturation_level != 0) {
    _internal_set_integrator_saturation_level(from._internal_integrator_saturation_level());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kp = from._internal_kp();
  uint64_t raw_kp;
  memcpy(&raw_kp, &tmp_kp, sizeof(tmp_kp));
  if (raw_kp != 0) {
    _internal_set_kp(from._internal_kp());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_ki = from._internal_ki();
  uint64_t raw_ki;
  memcpy(&raw_ki, &tmp_ki, sizeof(tmp_ki));
  if (raw_ki != 0) {
    _internal_set_ki(from._internal_ki());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kd = from._internal_kd();
  uint64_t raw_kd;
  memcpy(&raw_kd, &tmp_kd, sizeof(tmp_kd));
  if (raw_kd != 0) {
    _internal_set_kd(from._internal_kd());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_kaw = from._internal_kaw();
  uint64_t raw_kaw;
  memcpy(&raw_kaw, &tmp_kaw, sizeof(tmp_kaw));
  if (raw_kaw != 0) {
    _internal_set_kaw(from._internal_kaw());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_output_saturation_level = from._internal_output_saturation_level();
  uint64_t raw_output_saturation_level;
  memcpy(&raw_output_saturation_level, &tmp_output_saturation_level, sizeof(tmp_output_saturation_level));
  if (raw_output_saturation_level != 0) {
    _internal_set_output_saturation_level(from._internal_output_saturation_level());
  }
  if (from._internal_integrator_enable() != 0) {
    _internal_set_integrator_enable(from._internal_integrator_enable());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void PidConf::CopyFrom(const PidConf& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autoagric.control.PidConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PidConf::IsInitialized() const {
  return true;
}

void PidConf::InternalSwap(PidConf* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(PidConf, integrator_enable_)
      + sizeof(PidConf::integrator_enable_)
      - PROTOBUF_FIELD_OFFSET(PidConf, integrator_saturation_level_)>(
          reinterpret_cast<char*>(&integrator_saturation_level_),
          reinterpret_cast<char*>(&other->integrator_saturation_level_));
}

::PROTOBUF_NAMESPACE_ID::Metadata PidConf::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto_getter, &descriptor_table_autoagric_2fcontrol_2fpid_5fconf_2eproto_once,
      file_level_metadata_autoagric_2fcontrol_2fpid_5fconf_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace control
}  // namespace autoagric
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::autoagric::control::PidConf* Arena::CreateMaybeMessage< ::autoagric::control::PidConf >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autoagric::control::PidConf >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
