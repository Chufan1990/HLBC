// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: autoagric/planning/sl_boundary.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_autoagric_2fplanning_2fsl_5fboundary_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_autoagric_2fplanning_2fsl_5fboundary_2eproto

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
#include "autoagric/common/pnc_point.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_autoagric_2fplanning_2fsl_5fboundary_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_autoagric_2fplanning_2fsl_5fboundary_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_autoagric_2fplanning_2fsl_5fboundary_2eproto;
namespace autoagric {
namespace planning {
class SLBoundary;
struct SLBoundaryDefaultTypeInternal;
extern SLBoundaryDefaultTypeInternal _SLBoundary_default_instance_;
}  // namespace planning
}  // namespace autoagric
PROTOBUF_NAMESPACE_OPEN
template<> ::autoagric::planning::SLBoundary* Arena::CreateMaybeMessage<::autoagric::planning::SLBoundary>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace autoagric {
namespace planning {

// ===================================================================

class SLBoundary final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:autoagric.planning.SLBoundary) */ {
 public:
  inline SLBoundary() : SLBoundary(nullptr) {}
  ~SLBoundary() override;
  explicit constexpr SLBoundary(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SLBoundary(const SLBoundary& from);
  SLBoundary(SLBoundary&& from) noexcept
    : SLBoundary() {
    *this = ::std::move(from);
  }

  inline SLBoundary& operator=(const SLBoundary& from) {
    CopyFrom(from);
    return *this;
  }
  inline SLBoundary& operator=(SLBoundary&& from) noexcept {
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
  static const SLBoundary& default_instance() {
    return *internal_default_instance();
  }
  static inline const SLBoundary* internal_default_instance() {
    return reinterpret_cast<const SLBoundary*>(
               &_SLBoundary_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SLBoundary& a, SLBoundary& b) {
    a.Swap(&b);
  }
  inline void Swap(SLBoundary* other) {
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
  void UnsafeArenaSwap(SLBoundary* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  SLBoundary* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<SLBoundary>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const SLBoundary& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const SLBoundary& from);
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
  void InternalSwap(SLBoundary* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "autoagric.planning.SLBoundary";
  }
  protected:
  explicit SLBoundary(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kBoundaryPointFieldNumber = 5,
    kStartSFieldNumber = 1,
    kEndSFieldNumber = 2,
    kStartLFieldNumber = 3,
    kEndLFieldNumber = 4,
  };
  // repeated .autoagric.common.SLPoint boundary_point = 5;
  int boundary_point_size() const;
  private:
  int _internal_boundary_point_size() const;
  public:
  void clear_boundary_point();
  ::autoagric::common::SLPoint* mutable_boundary_point(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autoagric::common::SLPoint >*
      mutable_boundary_point();
  private:
  const ::autoagric::common::SLPoint& _internal_boundary_point(int index) const;
  ::autoagric::common::SLPoint* _internal_add_boundary_point();
  public:
  const ::autoagric::common::SLPoint& boundary_point(int index) const;
  ::autoagric::common::SLPoint* add_boundary_point();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autoagric::common::SLPoint >&
      boundary_point() const;

  // double start_s = 1;
  void clear_start_s();
  double start_s() const;
  void set_start_s(double value);
  private:
  double _internal_start_s() const;
  void _internal_set_start_s(double value);
  public:

  // double end_s = 2;
  void clear_end_s();
  double end_s() const;
  void set_end_s(double value);
  private:
  double _internal_end_s() const;
  void _internal_set_end_s(double value);
  public:

  // double start_l = 3;
  void clear_start_l();
  double start_l() const;
  void set_start_l(double value);
  private:
  double _internal_start_l() const;
  void _internal_set_start_l(double value);
  public:

  // double end_l = 4;
  void clear_end_l();
  double end_l() const;
  void set_end_l(double value);
  private:
  double _internal_end_l() const;
  void _internal_set_end_l(double value);
  public:

  // @@protoc_insertion_point(class_scope:autoagric.planning.SLBoundary)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autoagric::common::SLPoint > boundary_point_;
  double start_s_;
  double end_s_;
  double start_l_;
  double end_l_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_autoagric_2fplanning_2fsl_5fboundary_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SLBoundary

// double start_s = 1;
inline void SLBoundary::clear_start_s() {
  start_s_ = 0;
}
inline double SLBoundary::_internal_start_s() const {
  return start_s_;
}
inline double SLBoundary::start_s() const {
  // @@protoc_insertion_point(field_get:autoagric.planning.SLBoundary.start_s)
  return _internal_start_s();
}
inline void SLBoundary::_internal_set_start_s(double value) {
  
  start_s_ = value;
}
inline void SLBoundary::set_start_s(double value) {
  _internal_set_start_s(value);
  // @@protoc_insertion_point(field_set:autoagric.planning.SLBoundary.start_s)
}

// double end_s = 2;
inline void SLBoundary::clear_end_s() {
  end_s_ = 0;
}
inline double SLBoundary::_internal_end_s() const {
  return end_s_;
}
inline double SLBoundary::end_s() const {
  // @@protoc_insertion_point(field_get:autoagric.planning.SLBoundary.end_s)
  return _internal_end_s();
}
inline void SLBoundary::_internal_set_end_s(double value) {
  
  end_s_ = value;
}
inline void SLBoundary::set_end_s(double value) {
  _internal_set_end_s(value);
  // @@protoc_insertion_point(field_set:autoagric.planning.SLBoundary.end_s)
}

// double start_l = 3;
inline void SLBoundary::clear_start_l() {
  start_l_ = 0;
}
inline double SLBoundary::_internal_start_l() const {
  return start_l_;
}
inline double SLBoundary::start_l() const {
  // @@protoc_insertion_point(field_get:autoagric.planning.SLBoundary.start_l)
  return _internal_start_l();
}
inline void SLBoundary::_internal_set_start_l(double value) {
  
  start_l_ = value;
}
inline void SLBoundary::set_start_l(double value) {
  _internal_set_start_l(value);
  // @@protoc_insertion_point(field_set:autoagric.planning.SLBoundary.start_l)
}

// double end_l = 4;
inline void SLBoundary::clear_end_l() {
  end_l_ = 0;
}
inline double SLBoundary::_internal_end_l() const {
  return end_l_;
}
inline double SLBoundary::end_l() const {
  // @@protoc_insertion_point(field_get:autoagric.planning.SLBoundary.end_l)
  return _internal_end_l();
}
inline void SLBoundary::_internal_set_end_l(double value) {
  
  end_l_ = value;
}
inline void SLBoundary::set_end_l(double value) {
  _internal_set_end_l(value);
  // @@protoc_insertion_point(field_set:autoagric.planning.SLBoundary.end_l)
}

// repeated .autoagric.common.SLPoint boundary_point = 5;
inline int SLBoundary::_internal_boundary_point_size() const {
  return boundary_point_.size();
}
inline int SLBoundary::boundary_point_size() const {
  return _internal_boundary_point_size();
}
inline ::autoagric::common::SLPoint* SLBoundary::mutable_boundary_point(int index) {
  // @@protoc_insertion_point(field_mutable:autoagric.planning.SLBoundary.boundary_point)
  return boundary_point_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autoagric::common::SLPoint >*
SLBoundary::mutable_boundary_point() {
  // @@protoc_insertion_point(field_mutable_list:autoagric.planning.SLBoundary.boundary_point)
  return &boundary_point_;
}
inline const ::autoagric::common::SLPoint& SLBoundary::_internal_boundary_point(int index) const {
  return boundary_point_.Get(index);
}
inline const ::autoagric::common::SLPoint& SLBoundary::boundary_point(int index) const {
  // @@protoc_insertion_point(field_get:autoagric.planning.SLBoundary.boundary_point)
  return _internal_boundary_point(index);
}
inline ::autoagric::common::SLPoint* SLBoundary::_internal_add_boundary_point() {
  return boundary_point_.Add();
}
inline ::autoagric::common::SLPoint* SLBoundary::add_boundary_point() {
  ::autoagric::common::SLPoint* _add = _internal_add_boundary_point();
  // @@protoc_insertion_point(field_add:autoagric.planning.SLBoundary.boundary_point)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::autoagric::common::SLPoint >&
SLBoundary::boundary_point() const {
  // @@protoc_insertion_point(field_list:autoagric.planning.SLBoundary.boundary_point)
  return boundary_point_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace autoagric

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_autoagric_2fplanning_2fsl_5fboundary_2eproto
