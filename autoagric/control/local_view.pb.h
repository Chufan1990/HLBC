// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: autoagric/control/local_view.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_autoagric_2fcontrol_2flocal_5fview_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_autoagric_2fcontrol_2flocal_5fview_2eproto

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
#include "autoagric/canbus/chassis.pb.h"
#include "autoagric/common/header.pb.h"
#include "autoagric/localization/localization.pb.h"
#include "autoagric/planning/planning.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_autoagric_2fcontrol_2flocal_5fview_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_autoagric_2fcontrol_2flocal_5fview_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_autoagric_2fcontrol_2flocal_5fview_2eproto;
namespace autoagric {
namespace control {
class LocalView;
struct LocalViewDefaultTypeInternal;
extern LocalViewDefaultTypeInternal _LocalView_default_instance_;
}  // namespace control
}  // namespace autoagric
PROTOBUF_NAMESPACE_OPEN
template<> ::autoagric::control::LocalView* Arena::CreateMaybeMessage<::autoagric::control::LocalView>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace autoagric {
namespace control {

// ===================================================================

class LocalView final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:autoagric.control.LocalView) */ {
 public:
  inline LocalView() : LocalView(nullptr) {}
  ~LocalView() override;
  explicit constexpr LocalView(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  LocalView(const LocalView& from);
  LocalView(LocalView&& from) noexcept
    : LocalView() {
    *this = ::std::move(from);
  }

  inline LocalView& operator=(const LocalView& from) {
    CopyFrom(from);
    return *this;
  }
  inline LocalView& operator=(LocalView&& from) noexcept {
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
  static const LocalView& default_instance() {
    return *internal_default_instance();
  }
  static inline const LocalView* internal_default_instance() {
    return reinterpret_cast<const LocalView*>(
               &_LocalView_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LocalView& a, LocalView& b) {
    a.Swap(&b);
  }
  inline void Swap(LocalView* other) {
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
  void UnsafeArenaSwap(LocalView* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  LocalView* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<LocalView>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const LocalView& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const LocalView& from);
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
  void InternalSwap(LocalView* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "autoagric.control.LocalView";
  }
  protected:
  explicit LocalView(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kChassisFieldNumber = 2,
    kTrajectoryFieldNumber = 3,
    kLocalizationFieldNumber = 4,
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

  // .autoagric.canbus.Chassis chassis = 2;
  bool has_chassis() const;
  private:
  bool _internal_has_chassis() const;
  public:
  void clear_chassis();
  const ::autoagric::canbus::Chassis& chassis() const;
  PROTOBUF_NODISCARD ::autoagric::canbus::Chassis* release_chassis();
  ::autoagric::canbus::Chassis* mutable_chassis();
  void set_allocated_chassis(::autoagric::canbus::Chassis* chassis);
  private:
  const ::autoagric::canbus::Chassis& _internal_chassis() const;
  ::autoagric::canbus::Chassis* _internal_mutable_chassis();
  public:
  void unsafe_arena_set_allocated_chassis(
      ::autoagric::canbus::Chassis* chassis);
  ::autoagric::canbus::Chassis* unsafe_arena_release_chassis();

  // .autoagric.planning.ADCTrajectory trajectory = 3;
  bool has_trajectory() const;
  private:
  bool _internal_has_trajectory() const;
  public:
  void clear_trajectory();
  const ::autoagric::planning::ADCTrajectory& trajectory() const;
  PROTOBUF_NODISCARD ::autoagric::planning::ADCTrajectory* release_trajectory();
  ::autoagric::planning::ADCTrajectory* mutable_trajectory();
  void set_allocated_trajectory(::autoagric::planning::ADCTrajectory* trajectory);
  private:
  const ::autoagric::planning::ADCTrajectory& _internal_trajectory() const;
  ::autoagric::planning::ADCTrajectory* _internal_mutable_trajectory();
  public:
  void unsafe_arena_set_allocated_trajectory(
      ::autoagric::planning::ADCTrajectory* trajectory);
  ::autoagric::planning::ADCTrajectory* unsafe_arena_release_trajectory();

  // .autoagric.localization.LocalizationEstimate localization = 4;
  bool has_localization() const;
  private:
  bool _internal_has_localization() const;
  public:
  void clear_localization();
  const ::autoagric::localization::LocalizationEstimate& localization() const;
  PROTOBUF_NODISCARD ::autoagric::localization::LocalizationEstimate* release_localization();
  ::autoagric::localization::LocalizationEstimate* mutable_localization();
  void set_allocated_localization(::autoagric::localization::LocalizationEstimate* localization);
  private:
  const ::autoagric::localization::LocalizationEstimate& _internal_localization() const;
  ::autoagric::localization::LocalizationEstimate* _internal_mutable_localization();
  public:
  void unsafe_arena_set_allocated_localization(
      ::autoagric::localization::LocalizationEstimate* localization);
  ::autoagric::localization::LocalizationEstimate* unsafe_arena_release_localization();

  // @@protoc_insertion_point(class_scope:autoagric.control.LocalView)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::autoagric::common::Header* header_;
  ::autoagric::canbus::Chassis* chassis_;
  ::autoagric::planning::ADCTrajectory* trajectory_;
  ::autoagric::localization::LocalizationEstimate* localization_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_autoagric_2fcontrol_2flocal_5fview_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LocalView

// .autoagric.common.Header header = 1;
inline bool LocalView::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool LocalView::has_header() const {
  return _internal_has_header();
}
inline const ::autoagric::common::Header& LocalView::_internal_header() const {
  const ::autoagric::common::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::autoagric::common::Header&>(
      ::autoagric::common::_Header_default_instance_);
}
inline const ::autoagric::common::Header& LocalView::header() const {
  // @@protoc_insertion_point(field_get:autoagric.control.LocalView.header)
  return _internal_header();
}
inline void LocalView::unsafe_arena_set_allocated_header(
    ::autoagric::common::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autoagric.control.LocalView.header)
}
inline ::autoagric::common::Header* LocalView::release_header() {
  
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
inline ::autoagric::common::Header* LocalView::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:autoagric.control.LocalView.header)
  
  ::autoagric::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::autoagric::common::Header* LocalView::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::autoagric::common::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::autoagric::common::Header* LocalView::mutable_header() {
  ::autoagric::common::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:autoagric.control.LocalView.header)
  return _msg;
}
inline void LocalView::set_allocated_header(::autoagric::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:autoagric.control.LocalView.header)
}

// .autoagric.canbus.Chassis chassis = 2;
inline bool LocalView::_internal_has_chassis() const {
  return this != internal_default_instance() && chassis_ != nullptr;
}
inline bool LocalView::has_chassis() const {
  return _internal_has_chassis();
}
inline const ::autoagric::canbus::Chassis& LocalView::_internal_chassis() const {
  const ::autoagric::canbus::Chassis* p = chassis_;
  return p != nullptr ? *p : reinterpret_cast<const ::autoagric::canbus::Chassis&>(
      ::autoagric::canbus::_Chassis_default_instance_);
}
inline const ::autoagric::canbus::Chassis& LocalView::chassis() const {
  // @@protoc_insertion_point(field_get:autoagric.control.LocalView.chassis)
  return _internal_chassis();
}
inline void LocalView::unsafe_arena_set_allocated_chassis(
    ::autoagric::canbus::Chassis* chassis) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(chassis_);
  }
  chassis_ = chassis;
  if (chassis) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autoagric.control.LocalView.chassis)
}
inline ::autoagric::canbus::Chassis* LocalView::release_chassis() {
  
  ::autoagric::canbus::Chassis* temp = chassis_;
  chassis_ = nullptr;
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
inline ::autoagric::canbus::Chassis* LocalView::unsafe_arena_release_chassis() {
  // @@protoc_insertion_point(field_release:autoagric.control.LocalView.chassis)
  
  ::autoagric::canbus::Chassis* temp = chassis_;
  chassis_ = nullptr;
  return temp;
}
inline ::autoagric::canbus::Chassis* LocalView::_internal_mutable_chassis() {
  
  if (chassis_ == nullptr) {
    auto* p = CreateMaybeMessage<::autoagric::canbus::Chassis>(GetArenaForAllocation());
    chassis_ = p;
  }
  return chassis_;
}
inline ::autoagric::canbus::Chassis* LocalView::mutable_chassis() {
  ::autoagric::canbus::Chassis* _msg = _internal_mutable_chassis();
  // @@protoc_insertion_point(field_mutable:autoagric.control.LocalView.chassis)
  return _msg;
}
inline void LocalView::set_allocated_chassis(::autoagric::canbus::Chassis* chassis) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(chassis_);
  }
  if (chassis) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(chassis));
    if (message_arena != submessage_arena) {
      chassis = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, chassis, submessage_arena);
    }
    
  } else {
    
  }
  chassis_ = chassis;
  // @@protoc_insertion_point(field_set_allocated:autoagric.control.LocalView.chassis)
}

// .autoagric.planning.ADCTrajectory trajectory = 3;
inline bool LocalView::_internal_has_trajectory() const {
  return this != internal_default_instance() && trajectory_ != nullptr;
}
inline bool LocalView::has_trajectory() const {
  return _internal_has_trajectory();
}
inline const ::autoagric::planning::ADCTrajectory& LocalView::_internal_trajectory() const {
  const ::autoagric::planning::ADCTrajectory* p = trajectory_;
  return p != nullptr ? *p : reinterpret_cast<const ::autoagric::planning::ADCTrajectory&>(
      ::autoagric::planning::_ADCTrajectory_default_instance_);
}
inline const ::autoagric::planning::ADCTrajectory& LocalView::trajectory() const {
  // @@protoc_insertion_point(field_get:autoagric.control.LocalView.trajectory)
  return _internal_trajectory();
}
inline void LocalView::unsafe_arena_set_allocated_trajectory(
    ::autoagric::planning::ADCTrajectory* trajectory) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(trajectory_);
  }
  trajectory_ = trajectory;
  if (trajectory) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autoagric.control.LocalView.trajectory)
}
inline ::autoagric::planning::ADCTrajectory* LocalView::release_trajectory() {
  
  ::autoagric::planning::ADCTrajectory* temp = trajectory_;
  trajectory_ = nullptr;
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
inline ::autoagric::planning::ADCTrajectory* LocalView::unsafe_arena_release_trajectory() {
  // @@protoc_insertion_point(field_release:autoagric.control.LocalView.trajectory)
  
  ::autoagric::planning::ADCTrajectory* temp = trajectory_;
  trajectory_ = nullptr;
  return temp;
}
inline ::autoagric::planning::ADCTrajectory* LocalView::_internal_mutable_trajectory() {
  
  if (trajectory_ == nullptr) {
    auto* p = CreateMaybeMessage<::autoagric::planning::ADCTrajectory>(GetArenaForAllocation());
    trajectory_ = p;
  }
  return trajectory_;
}
inline ::autoagric::planning::ADCTrajectory* LocalView::mutable_trajectory() {
  ::autoagric::planning::ADCTrajectory* _msg = _internal_mutable_trajectory();
  // @@protoc_insertion_point(field_mutable:autoagric.control.LocalView.trajectory)
  return _msg;
}
inline void LocalView::set_allocated_trajectory(::autoagric::planning::ADCTrajectory* trajectory) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(trajectory_);
  }
  if (trajectory) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(trajectory));
    if (message_arena != submessage_arena) {
      trajectory = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, trajectory, submessage_arena);
    }
    
  } else {
    
  }
  trajectory_ = trajectory;
  // @@protoc_insertion_point(field_set_allocated:autoagric.control.LocalView.trajectory)
}

// .autoagric.localization.LocalizationEstimate localization = 4;
inline bool LocalView::_internal_has_localization() const {
  return this != internal_default_instance() && localization_ != nullptr;
}
inline bool LocalView::has_localization() const {
  return _internal_has_localization();
}
inline const ::autoagric::localization::LocalizationEstimate& LocalView::_internal_localization() const {
  const ::autoagric::localization::LocalizationEstimate* p = localization_;
  return p != nullptr ? *p : reinterpret_cast<const ::autoagric::localization::LocalizationEstimate&>(
      ::autoagric::localization::_LocalizationEstimate_default_instance_);
}
inline const ::autoagric::localization::LocalizationEstimate& LocalView::localization() const {
  // @@protoc_insertion_point(field_get:autoagric.control.LocalView.localization)
  return _internal_localization();
}
inline void LocalView::unsafe_arena_set_allocated_localization(
    ::autoagric::localization::LocalizationEstimate* localization) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(localization_);
  }
  localization_ = localization;
  if (localization) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:autoagric.control.LocalView.localization)
}
inline ::autoagric::localization::LocalizationEstimate* LocalView::release_localization() {
  
  ::autoagric::localization::LocalizationEstimate* temp = localization_;
  localization_ = nullptr;
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
inline ::autoagric::localization::LocalizationEstimate* LocalView::unsafe_arena_release_localization() {
  // @@protoc_insertion_point(field_release:autoagric.control.LocalView.localization)
  
  ::autoagric::localization::LocalizationEstimate* temp = localization_;
  localization_ = nullptr;
  return temp;
}
inline ::autoagric::localization::LocalizationEstimate* LocalView::_internal_mutable_localization() {
  
  if (localization_ == nullptr) {
    auto* p = CreateMaybeMessage<::autoagric::localization::LocalizationEstimate>(GetArenaForAllocation());
    localization_ = p;
  }
  return localization_;
}
inline ::autoagric::localization::LocalizationEstimate* LocalView::mutable_localization() {
  ::autoagric::localization::LocalizationEstimate* _msg = _internal_mutable_localization();
  // @@protoc_insertion_point(field_mutable:autoagric.control.LocalView.localization)
  return _msg;
}
inline void LocalView::set_allocated_localization(::autoagric::localization::LocalizationEstimate* localization) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(localization_);
  }
  if (localization) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(localization));
    if (message_arena != submessage_arena) {
      localization = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, localization, submessage_arena);
    }
    
  } else {
    
  }
  localization_ = localization;
  // @@protoc_insertion_point(field_set_allocated:autoagric.control.LocalView.localization)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace control
}  // namespace autoagric

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_autoagric_2fcontrol_2flocal_5fview_2eproto
