// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: kinect_rgbd.proto

#ifndef PROTOBUF_kinect_5frgbd_2eproto__INCLUDED
#define PROTOBUF_kinect_5frgbd_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace kinectrgbd {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_kinect_5frgbd_2eproto();
void protobuf_AssignDesc_kinect_5frgbd_2eproto();
void protobuf_ShutdownFile_kinect_5frgbd_2eproto();

class Point;
class Response;

// ===================================================================

class Point : public ::google::protobuf::Message {
 public:
  Point();
  virtual ~Point();

  Point(const Point& from);

  inline Point& operator=(const Point& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Point& default_instance();

  void Swap(Point* other);

  // implements Message ----------------------------------------------

  inline Point* New() const { return New(NULL); }

  Point* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Point& from);
  void MergeFrom(const Point& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Point* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional float x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  float x() const;
  void set_x(float value);

  // optional float y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  float y() const;
  void set_y(float value);

  // optional float z = 3;
  void clear_z();
  static const int kZFieldNumber = 3;
  float z() const;
  void set_z(float value);

  // optional int32 color = 4;
  void clear_color();
  static const int kColorFieldNumber = 4;
  ::google::protobuf::int32 color() const;
  void set_color(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:kinectrgbd.Point)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  float x_;
  float y_;
  float z_;
  ::google::protobuf::int32 color_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5frgbd_2eproto();
  friend void protobuf_AssignDesc_kinect_5frgbd_2eproto();
  friend void protobuf_ShutdownFile_kinect_5frgbd_2eproto();

  void InitAsDefaultInstance();
  static Point* default_instance_;
};
// -------------------------------------------------------------------

class Response : public ::google::protobuf::Message {
 public:
  Response();
  virtual ~Response();

  Response(const Response& from);

  inline Response& operator=(const Response& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Response& default_instance();

  void Swap(Response* other);

  // implements Message ----------------------------------------------

  inline Response* New() const { return New(NULL); }

  Response* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Response& from);
  void MergeFrom(const Response& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Response* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  ::google::protobuf::int32 x() const;
  void set_x(::google::protobuf::int32 value);

  // optional int32 y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  ::google::protobuf::int32 y() const;
  void set_y(::google::protobuf::int32 value);

  // optional int32 width = 3;
  void clear_width();
  static const int kWidthFieldNumber = 3;
  ::google::protobuf::int32 width() const;
  void set_width(::google::protobuf::int32 value);

  // optional int32 height = 4;
  void clear_height();
  static const int kHeightFieldNumber = 4;
  ::google::protobuf::int32 height() const;
  void set_height(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:kinectrgbd.Response)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::int32 x_;
  ::google::protobuf::int32 y_;
  ::google::protobuf::int32 width_;
  ::google::protobuf::int32 height_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5frgbd_2eproto();
  friend void protobuf_AssignDesc_kinect_5frgbd_2eproto();
  friend void protobuf_ShutdownFile_kinect_5frgbd_2eproto();

  void InitAsDefaultInstance();
  static Response* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Point

// optional float x = 1;
inline void Point::clear_x() {
  x_ = 0;
}
inline float Point::x() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Point.x)
  return x_;
}
inline void Point::set_x(float value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Point.x)
}

// optional float y = 2;
inline void Point::clear_y() {
  y_ = 0;
}
inline float Point::y() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Point.y)
  return y_;
}
inline void Point::set_y(float value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Point.y)
}

// optional float z = 3;
inline void Point::clear_z() {
  z_ = 0;
}
inline float Point::z() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Point.z)
  return z_;
}
inline void Point::set_z(float value) {
  
  z_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Point.z)
}

// optional int32 color = 4;
inline void Point::clear_color() {
  color_ = 0;
}
inline ::google::protobuf::int32 Point::color() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Point.color)
  return color_;
}
inline void Point::set_color(::google::protobuf::int32 value) {
  
  color_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Point.color)
}

// -------------------------------------------------------------------

// Response

// optional int32 x = 1;
inline void Response::clear_x() {
  x_ = 0;
}
inline ::google::protobuf::int32 Response::x() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Response.x)
  return x_;
}
inline void Response::set_x(::google::protobuf::int32 value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Response.x)
}

// optional int32 y = 2;
inline void Response::clear_y() {
  y_ = 0;
}
inline ::google::protobuf::int32 Response::y() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Response.y)
  return y_;
}
inline void Response::set_y(::google::protobuf::int32 value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Response.y)
}

// optional int32 width = 3;
inline void Response::clear_width() {
  width_ = 0;
}
inline ::google::protobuf::int32 Response::width() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Response.width)
  return width_;
}
inline void Response::set_width(::google::protobuf::int32 value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Response.width)
}

// optional int32 height = 4;
inline void Response::clear_height() {
  height_ = 0;
}
inline ::google::protobuf::int32 Response::height() const {
  // @@protoc_insertion_point(field_get:kinectrgbd.Response.height)
  return height_;
}
inline void Response::set_height(::google::protobuf::int32 value) {
  
  height_ = value;
  // @@protoc_insertion_point(field_set:kinectrgbd.Response.height)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace kinectrgbd

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_kinect_5frgbd_2eproto__INCLUDED
