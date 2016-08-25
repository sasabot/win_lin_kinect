// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: kinect_person.proto

#ifndef PROTOBUF_kinect_5fperson_2eproto__INCLUDED
#define PROTOBUF_kinect_5fperson_2eproto__INCLUDED

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

namespace kinectperson {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_kinect_5fperson_2eproto();
void protobuf_AssignDesc_kinect_5fperson_2eproto();
void protobuf_ShutdownFile_kinect_5fperson_2eproto();

class Face;
class Person;
class PersonStream;
class Response;
class Text;

// ===================================================================

class Face : public ::google::protobuf::Message {
 public:
  Face();
  virtual ~Face();

  Face(const Face& from);

  inline Face& operator=(const Face& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Face& default_instance();

  void Swap(Face* other);

  // implements Message ----------------------------------------------

  inline Face* New() const { return New(NULL); }

  Face* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Face& from);
  void MergeFrom(const Face& from);
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
  void InternalSwap(Face* other);
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

  // optional float width = 3;
  void clear_width();
  static const int kWidthFieldNumber = 3;
  float width() const;
  void set_width(float value);

  // optional float height = 4;
  void clear_height();
  static const int kHeightFieldNumber = 4;
  float height() const;
  void set_height(float value);

  // optional float roll = 5;
  void clear_roll();
  static const int kRollFieldNumber = 5;
  float roll() const;
  void set_roll(float value);

  // optional float pitch = 6;
  void clear_pitch();
  static const int kPitchFieldNumber = 6;
  float pitch() const;
  void set_pitch(float value);

  // optional float yaw = 7;
  void clear_yaw();
  static const int kYawFieldNumber = 7;
  float yaw() const;
  void set_yaw(float value);

  // @@protoc_insertion_point(class_scope:kinectperson.Face)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  float x_;
  float y_;
  float width_;
  float height_;
  float roll_;
  float pitch_;
  float yaw_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5fperson_2eproto();
  friend void protobuf_AssignDesc_kinect_5fperson_2eproto();
  friend void protobuf_ShutdownFile_kinect_5fperson_2eproto();

  void InitAsDefaultInstance();
  static Face* default_instance_;
};
// -------------------------------------------------------------------

class Person : public ::google::protobuf::Message {
 public:
  Person();
  virtual ~Person();

  Person(const Person& from);

  inline Person& operator=(const Person& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Person& default_instance();

  void Swap(Person* other);

  // implements Message ----------------------------------------------

  inline Person* New() const { return New(NULL); }

  Person* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Person& from);
  void MergeFrom(const Person& from);
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
  void InternalSwap(Person* other);
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

  // optional .kinectperson.Face face = 1;
  bool has_face() const;
  void clear_face();
  static const int kFaceFieldNumber = 1;
  const ::kinectperson::Face& face() const;
  ::kinectperson::Face* mutable_face();
  ::kinectperson::Face* release_face();
  void set_allocated_face(::kinectperson::Face* face);

  // optional bool speaking = 2;
  void clear_speaking();
  static const int kSpeakingFieldNumber = 2;
  bool speaking() const;
  void set_speaking(bool value);

  // optional bool looking = 3;
  void clear_looking();
  static const int kLookingFieldNumber = 3;
  bool looking() const;
  void set_looking(bool value);

  // optional float distance = 4;
  void clear_distance();
  static const int kDistanceFieldNumber = 4;
  float distance() const;
  void set_distance(float value);

  // optional int32 id = 5;
  void clear_id();
  static const int kIdFieldNumber = 5;
  ::google::protobuf::int32 id() const;
  void set_id(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:kinectperson.Person)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::kinectperson::Face* face_;
  bool speaking_;
  bool looking_;
  float distance_;
  ::google::protobuf::int32 id_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5fperson_2eproto();
  friend void protobuf_AssignDesc_kinect_5fperson_2eproto();
  friend void protobuf_ShutdownFile_kinect_5fperson_2eproto();

  void InitAsDefaultInstance();
  static Person* default_instance_;
};
// -------------------------------------------------------------------

class PersonStream : public ::google::protobuf::Message {
 public:
  PersonStream();
  virtual ~PersonStream();

  PersonStream(const PersonStream& from);

  inline PersonStream& operator=(const PersonStream& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PersonStream& default_instance();

  void Swap(PersonStream* other);

  // implements Message ----------------------------------------------

  inline PersonStream* New() const { return New(NULL); }

  PersonStream* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const PersonStream& from);
  void MergeFrom(const PersonStream& from);
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
  void InternalSwap(PersonStream* other);
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

  // optional int32 status = 1;
  void clear_status();
  static const int kStatusFieldNumber = 1;
  ::google::protobuf::int32 status() const;
  void set_status(::google::protobuf::int32 value);

  // repeated .kinectperson.Person data = 2;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 2;
  const ::kinectperson::Person& data(int index) const;
  ::kinectperson::Person* mutable_data(int index);
  ::kinectperson::Person* add_data();
  ::google::protobuf::RepeatedPtrField< ::kinectperson::Person >*
      mutable_data();
  const ::google::protobuf::RepeatedPtrField< ::kinectperson::Person >&
      data() const;

  // @@protoc_insertion_point(class_scope:kinectperson.PersonStream)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::RepeatedPtrField< ::kinectperson::Person > data_;
  ::google::protobuf::int32 status_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5fperson_2eproto();
  friend void protobuf_AssignDesc_kinect_5fperson_2eproto();
  friend void protobuf_ShutdownFile_kinect_5fperson_2eproto();

  void InitAsDefaultInstance();
  static PersonStream* default_instance_;
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

  // optional bool status = 1;
  void clear_status();
  static const int kStatusFieldNumber = 1;
  bool status() const;
  void set_status(bool value);

  // @@protoc_insertion_point(class_scope:kinectperson.Response)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  bool status_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5fperson_2eproto();
  friend void protobuf_AssignDesc_kinect_5fperson_2eproto();
  friend void protobuf_ShutdownFile_kinect_5fperson_2eproto();

  void InitAsDefaultInstance();
  static Response* default_instance_;
};
// -------------------------------------------------------------------

class Text : public ::google::protobuf::Message {
 public:
  Text();
  virtual ~Text();

  Text(const Text& from);

  inline Text& operator=(const Text& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Text& default_instance();

  void Swap(Text* other);

  // implements Message ----------------------------------------------

  inline Text* New() const { return New(NULL); }

  Text* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Text& from);
  void MergeFrom(const Text& from);
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
  void InternalSwap(Text* other);
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

  // optional string text = 1;
  void clear_text();
  static const int kTextFieldNumber = 1;
  const ::std::string& text() const;
  void set_text(const ::std::string& value);
  void set_text(const char* value);
  void set_text(const char* value, size_t size);
  ::std::string* mutable_text();
  ::std::string* release_text();
  void set_allocated_text(::std::string* text);

  // @@protoc_insertion_point(class_scope:kinectperson.Text)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::internal::ArenaStringPtr text_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_kinect_5fperson_2eproto();
  friend void protobuf_AssignDesc_kinect_5fperson_2eproto();
  friend void protobuf_ShutdownFile_kinect_5fperson_2eproto();

  void InitAsDefaultInstance();
  static Text* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Face

// optional float x = 1;
inline void Face::clear_x() {
  x_ = 0;
}
inline float Face::x() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.x)
  return x_;
}
inline void Face::set_x(float value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.x)
}

// optional float y = 2;
inline void Face::clear_y() {
  y_ = 0;
}
inline float Face::y() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.y)
  return y_;
}
inline void Face::set_y(float value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.y)
}

// optional float width = 3;
inline void Face::clear_width() {
  width_ = 0;
}
inline float Face::width() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.width)
  return width_;
}
inline void Face::set_width(float value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.width)
}

// optional float height = 4;
inline void Face::clear_height() {
  height_ = 0;
}
inline float Face::height() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.height)
  return height_;
}
inline void Face::set_height(float value) {
  
  height_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.height)
}

// optional float roll = 5;
inline void Face::clear_roll() {
  roll_ = 0;
}
inline float Face::roll() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.roll)
  return roll_;
}
inline void Face::set_roll(float value) {
  
  roll_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.roll)
}

// optional float pitch = 6;
inline void Face::clear_pitch() {
  pitch_ = 0;
}
inline float Face::pitch() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.pitch)
  return pitch_;
}
inline void Face::set_pitch(float value) {
  
  pitch_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.pitch)
}

// optional float yaw = 7;
inline void Face::clear_yaw() {
  yaw_ = 0;
}
inline float Face::yaw() const {
  // @@protoc_insertion_point(field_get:kinectperson.Face.yaw)
  return yaw_;
}
inline void Face::set_yaw(float value) {
  
  yaw_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Face.yaw)
}

// -------------------------------------------------------------------

// Person

// optional .kinectperson.Face face = 1;
inline bool Person::has_face() const {
  return !_is_default_instance_ && face_ != NULL;
}
inline void Person::clear_face() {
  if (GetArenaNoVirtual() == NULL && face_ != NULL) delete face_;
  face_ = NULL;
}
inline const ::kinectperson::Face& Person::face() const {
  // @@protoc_insertion_point(field_get:kinectperson.Person.face)
  return face_ != NULL ? *face_ : *default_instance_->face_;
}
inline ::kinectperson::Face* Person::mutable_face() {
  
  if (face_ == NULL) {
    face_ = new ::kinectperson::Face;
  }
  // @@protoc_insertion_point(field_mutable:kinectperson.Person.face)
  return face_;
}
inline ::kinectperson::Face* Person::release_face() {
  // @@protoc_insertion_point(field_release:kinectperson.Person.face)
  
  ::kinectperson::Face* temp = face_;
  face_ = NULL;
  return temp;
}
inline void Person::set_allocated_face(::kinectperson::Face* face) {
  delete face_;
  face_ = face;
  if (face) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:kinectperson.Person.face)
}

// optional bool speaking = 2;
inline void Person::clear_speaking() {
  speaking_ = false;
}
inline bool Person::speaking() const {
  // @@protoc_insertion_point(field_get:kinectperson.Person.speaking)
  return speaking_;
}
inline void Person::set_speaking(bool value) {
  
  speaking_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Person.speaking)
}

// optional bool looking = 3;
inline void Person::clear_looking() {
  looking_ = false;
}
inline bool Person::looking() const {
  // @@protoc_insertion_point(field_get:kinectperson.Person.looking)
  return looking_;
}
inline void Person::set_looking(bool value) {
  
  looking_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Person.looking)
}

// optional float distance = 4;
inline void Person::clear_distance() {
  distance_ = 0;
}
inline float Person::distance() const {
  // @@protoc_insertion_point(field_get:kinectperson.Person.distance)
  return distance_;
}
inline void Person::set_distance(float value) {
  
  distance_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Person.distance)
}

// optional int32 id = 5;
inline void Person::clear_id() {
  id_ = 0;
}
inline ::google::protobuf::int32 Person::id() const {
  // @@protoc_insertion_point(field_get:kinectperson.Person.id)
  return id_;
}
inline void Person::set_id(::google::protobuf::int32 value) {
  
  id_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Person.id)
}

// -------------------------------------------------------------------

// PersonStream

// optional int32 status = 1;
inline void PersonStream::clear_status() {
  status_ = 0;
}
inline ::google::protobuf::int32 PersonStream::status() const {
  // @@protoc_insertion_point(field_get:kinectperson.PersonStream.status)
  return status_;
}
inline void PersonStream::set_status(::google::protobuf::int32 value) {
  
  status_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.PersonStream.status)
}

// repeated .kinectperson.Person data = 2;
inline int PersonStream::data_size() const {
  return data_.size();
}
inline void PersonStream::clear_data() {
  data_.Clear();
}
inline const ::kinectperson::Person& PersonStream::data(int index) const {
  // @@protoc_insertion_point(field_get:kinectperson.PersonStream.data)
  return data_.Get(index);
}
inline ::kinectperson::Person* PersonStream::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:kinectperson.PersonStream.data)
  return data_.Mutable(index);
}
inline ::kinectperson::Person* PersonStream::add_data() {
  // @@protoc_insertion_point(field_add:kinectperson.PersonStream.data)
  return data_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::kinectperson::Person >*
PersonStream::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:kinectperson.PersonStream.data)
  return &data_;
}
inline const ::google::protobuf::RepeatedPtrField< ::kinectperson::Person >&
PersonStream::data() const {
  // @@protoc_insertion_point(field_list:kinectperson.PersonStream.data)
  return data_;
}

// -------------------------------------------------------------------

// Response

// optional bool status = 1;
inline void Response::clear_status() {
  status_ = false;
}
inline bool Response::status() const {
  // @@protoc_insertion_point(field_get:kinectperson.Response.status)
  return status_;
}
inline void Response::set_status(bool value) {
  
  status_ = value;
  // @@protoc_insertion_point(field_set:kinectperson.Response.status)
}

// -------------------------------------------------------------------

// Text

// optional string text = 1;
inline void Text::clear_text() {
  text_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Text::text() const {
  // @@protoc_insertion_point(field_get:kinectperson.Text.text)
  return text_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Text::set_text(const ::std::string& value) {
  
  text_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:kinectperson.Text.text)
}
inline void Text::set_text(const char* value) {
  
  text_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:kinectperson.Text.text)
}
inline void Text::set_text(const char* value, size_t size) {
  
  text_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:kinectperson.Text.text)
}
inline ::std::string* Text::mutable_text() {
  
  // @@protoc_insertion_point(field_mutable:kinectperson.Text.text)
  return text_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Text::release_text() {
  // @@protoc_insertion_point(field_release:kinectperson.Text.text)
  
  return text_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Text::set_allocated_text(::std::string* text) {
  if (text != NULL) {
    
  } else {
    
  }
  text_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), text);
  // @@protoc_insertion_point(field_set_allocated:kinectperson.Text.text)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace kinectperson

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_kinect_5fperson_2eproto__INCLUDED