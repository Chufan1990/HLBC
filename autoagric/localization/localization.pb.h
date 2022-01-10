// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: autoagric/localization/localization.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_autoagric_2flocalization_2flocalization_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_autoagric_2flocalization_2flocalization_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3019000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3019001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "autoagric/localization/pose.pb.h"
#include "autoagric/common/header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_autoagric_2flocalization_2flocalization_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_autoagric_2flocalization_2flocalization_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_autoagric_2flocalization_2flocalization_2eproto;
namespace autoagric {
namespace localization {
class LocalizationEstimate;
struct LocalizationEstimateDefaultTypeInternal;
extern LocalizationEstimateDefaultTypeInternal _LocalizationEstimate_default_instance_;
}  // namespace localization
}  // namespace autoagric
PROTOBUF_NAMESPACE_OPEN
template<> ::autoagric::localization::LocalizationEstimate* Arena::CreateMaybeMessage<::autoagric::localization::LocalizationEstimate>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace autoagric {
namespace localization {

// ===================================================================

class LocalizationEstimate final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:autoagric.localization.LocalizationEstimate) */ {
 public:
  inline LocalizationEstimate() : LocalizationEstimate(nullptr) {}
  ~LocalizationEstimate() override;
  explicit constexpr LocalizationEstimate(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  LocalizationEstimate(const LocalizationEstimate& from);
  LocalizationEstimate(LocalizationEstimate&& from) noexcept
    : LocalizationEstimate() {
    *this = ::std::move(from);
  }

  inline LocalizationEstimate& operator=(const LocalizationEstimate& from) {
    CopyFrom(from);
    return *this;
  }
  inline LocalizationEstimate& operator=(LocalizationEstimate&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const LocalizationEstimate& default_instance() {
    return *internal_default_instance();
  }
  static inline const LocalizationEstimate* internal_default_instance() {
    return reinterpret_cast<const LocalizationEstimate*>(
               &_LocalizationEstimate_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LocalizationEstimate& a, LocalizationEstimate& b) {
    a.Swap(&b);
  }
  inline void Swap(LocalizationEstimate* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(LocalizationEstimate* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  LocalizationEstimate* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<LocalizationEstimate>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const LocalizationEstimate& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const LocalizationEstimate& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(LocalizationEstimate* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "autoagric.localization.LocalizationEstimate";
  }
  protected:
  explicit LocalizationEstimate(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kPoseFieldNumber = 2,
    kMeasurementTimeFieldNumber = 4,
  };
  // .autoagric.common.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::autoagric::common::Header& header() const;
  PROTOBUF_NODISCARD ::autoagric::common::Header* release_header();
  ::autoagric::common::Header* mutable_header();
  void set_allocated_header(::autoagric::common::Header* header);
  private:
  const ::autoagric::common::Header& _internal_header() const;
  ::autoagric::common::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::autoagric::common::Header* header);
  ::autoagric::common::Header* unsafe_arena_release_header();

  // .autoagric.localization.Pose pose = 2;
  bool has_pose() const;
  private:
  bool _internal_has_pose() const;
  public:
  void clear_pose();
  const ::autoagric::localization::Pose& pose() const;
  PROTOBUF_NODISCARD ::autoagric::localization::Pose* release_pose();
  ::autoagric::localization::Pose* mutable_pose();
  void set_allocated_pose(::autoagric::localization::Pose* pose);
  private:
  const ::autoagric::localization::Pose& _internal_pose() const;
  ::autoagric::localization::Pose* _internal_mutable_pose();
  public:
  void unsafe_arena_set_allocated_pose(
      ::autoagric::localization::Pose* pose);
  ::autoagric::localization::Pose* unsafe_arena_release_pose();

  // optional double measurement_time = 4;
  bool has_measurement_time() const;
  private:
  bool _internal_has_measurement_time() const;
  public:
  void clear_measurement_time();
  double measurement_time() const;
  void set_measurement_time(double value);
  private:
  double _internal_measurement_time() const;
  void _internal_set_measurement_time(double value);
  public:

  // @@protoc_insertion_point(class_scope:autoagric.localization.LocalizationEstimate)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::autoagric::common::Header* header_;
  ::autoagric::localization::Pose* pose_;
  double measurement_time_;
  friend struct ::TableStruct_autoagric_2flocalization_2flocalization_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LocalizationEstimate

// .autoagric.common.Header header = 1;
inline bool LocalizationEstimate::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool LocalizationEstimate::has_header() const {
  return _internal_has_header();
}
inline const ::autoagric::common::Header& LocalizationEstimate::_internal_header() const {
  const ::autoagric::common::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::autoagric::common::Header&>(
      ::autoagric::common::_Header_default_instance_);
}
inline const ::autoagric::common::Header& LocalizationEstimate::header() const {
  // @@protoc_insertion_point(field_get:autoagric.localization.LocalizationEstimate.header)
  return _internal_header();
}
inline void LocalizationEstimate::unsafe_arena_set_allocated_header(
    ::autoagric::common::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autoagric.localization.LocalizationEstimate.header)
}
inline ::autoagric::common::Header* LocalizationEstimate::release_header() {
  
  ::autoagric::common::Header* temp = header_;
  header_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::autoagric::common::Header* LocalizationEstimate::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:autoagric.localization.LocalizationEstimate.header)
  
  ::autoagric::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::autoagric::common::Header* LocalizationEstimate::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::autoagric::common::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::autoagric::common::Header* LocalizationEstimate::mutable_header() {
  ::autoagric::common::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:autoagric.localization.LocalizationEstimate.header)
  return _msg;
}
inline void LocalizationEstimate::set_allocated_header(::autoagric::common::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:autoagric.localization.LocalizationEstimate.header)
}

// .autoagric.localization.Pose pose = 2;
inline bool LocalizationEstimate::_internal_has_pose() const {
  return this != internal_default_instance() && pose_ != nullptr;
}
inline bool LocalizationEstimate::has_pose() const {
  return _internal_has_pose();
}
inline const ::autoagric::localization::Pose& LocalizationEstimate::_internal_pose() const {
  const ::autoagric::localization::Pose* p = pose_;
  return p != nullptr ? *p : reinterpret_cast<const ::autoagric::localization::Pose&>(
      ::autoagric::localization::_Pose_default_instance_);
}
inline const ::autoagric::localization::Pose& LocalizationEstimate::pose() const {
  // @@protoc_insertion_point(field_get:autoagric.localization.LocalizationEstimate.pose)
  return _internal_pose();
}
inline void LocalizationEstimate::unsafe_arena_set_allocated_pose(
    ::autoagric::localization::Pose* pose) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose_);
  }
  pose_ = pose;
  if (pose) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autoagric.localization.LocalizationEstimate.pose)
}
inline ::autoagric::localization::Pose* LocalizationEstimate::release_pose() {
  
  ::autoagric::localization::Pose* temp = pose_;
  pose_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::autoagric::localization::Pose* LocalizationEstimate::unsafe_arena_release_pose() {
  // @@protoc_insertion_point(field_release:autoagric.localization.LocalizationEstimate.pose)
  
  ::autoagric::localization::Pose* temp = pose_;
  pose_ = nullptr;
  return temp;
}
inline ::autoagric::localization::Pose* LocalizationEstimate::_internal_mutable_pose() {
  
  if (pose_ == nullptr) {
    auto* p = CreateMaybeMessage<::autoagric::localization::Pose>(GetArenaForAllocation());
    pose_ = p;
  }
  return pose_;
}
inline ::autoagric::localization::Pose* LocalizationEstimate::mutable_pose() {
  ::autoagric::localization::Pose* _msg = _internal_mutable_pose();
  // @@protoc_insertion_point(field_mutable:autoagric.localization.LocalizationEstimate.pose)
  return _msg;
}
inline void LocalizationEstimate::set_allocated_pose(::autoagric::localization::Pose* pose) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose_);
  }
  if (pose) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose));
    if (message_arena != submessage_arena) {
      pose = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, pose, submessage_arena);
    }
    
  } else {
    
  }
  pose_ = pose;
  // @@protoc_insertion_point(field_set_allocated:autoagric.localization.LocalizationEstimate.pose)
}

// optional double measurement_time = 4;
inline bool LocalizationEstimate::_internal_has_measurement_time() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LocalizationEstimate::has_measurement_time() const {
  return _internal_has_measurement_time();
}
inline void LocalizationEstimate::clear_measurement_time() {
  measurement_time_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double LocalizationEstimate::_internal_measurement_time() const {
  return measurement_time_;
}
inline double LocalizationEstimate::measurement_time() const {
  // @@protoc_insertion_point(field_get:autoagric.localization.LocalizationEstimate.measurement_time)
  return _internal_measurement_time();
}
inline void LocalizationEstimate::_internal_set_measurement_time(double value) {
  _has_bits_[0] |= 0x00000001u;
  measurement_time_ = value;
}
inline void LocalizationEstimate::set_measurement_time(double value) {
  _internal_set_measurement_time(value);
  // @@protoc_insertion_point(field_set:autoagric.localization.LocalizationEstimate.measurement_time)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace localization
}  // namespace autoagric

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_autoagric_2flocalization_2flocalization_2eproto