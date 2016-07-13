// Generated by the gRPC protobuf plugin.
// If you make any local change, they will be lost.
// source: kinect_rgbd.proto
#ifndef GRPC_kinect_5frgbd_2eproto__INCLUDED
#define GRPC_kinect_5frgbd_2eproto__INCLUDED

#include "kinect_rgbd.pb.h"

#include <grpc++/impl/codegen/async_stream.h>
#include <grpc++/impl/codegen/async_unary_call.h>
#include <grpc++/impl/codegen/proto_utils.h>
#include <grpc++/impl/codegen/rpc_method.h>
#include <grpc++/impl/codegen/service_type.h>
#include <grpc++/impl/codegen/status.h>
#include <grpc++/impl/codegen/stub_options.h>
#include <grpc++/impl/codegen/sync_stream.h>

namespace grpc {
class CompletionQueue;
class Channel;
class RpcService;
class ServerCompletionQueue;
class ServerContext;
}  // namespace grpc

namespace kinectrgbd {

class KinectRgbd GRPC_FINAL {
 public:
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status CheckRequest(::grpc::ClientContext* context, const ::kinectrgbd::Header& request, ::kinectrgbd::Request* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Request>> AsyncCheckRequest(::grpc::ClientContext* context, const ::kinectrgbd::Header& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Request>>(AsyncCheckRequestRaw(context, request, cq));
    }
    virtual ::grpc::Status SendPoints(::grpc::ClientContext* context, const ::kinectrgbd::Points& request, ::kinectrgbd::Response* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>> AsyncSendPoints(::grpc::ClientContext* context, const ::kinectrgbd::Points& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>>(AsyncSendPointsRaw(context, request, cq));
    }
    virtual ::grpc::Status SendImage(::grpc::ClientContext* context, const ::kinectrgbd::Pixels& request, ::kinectrgbd::Response* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>> AsyncSendImage(::grpc::ClientContext* context, const ::kinectrgbd::Pixels& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>>(AsyncSendImageRaw(context, request, cq));
    }
    virtual ::grpc::Status ReturnPositionsFromPixels(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::kinectrgbd::Response* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>> AsyncReturnPositionsFromPixels(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>>(AsyncReturnPositionsFromPixelsRaw(context, request, cq));
    }
    virtual ::grpc::Status ReturnPixelBoundsFromSpaceBounds(::grpc::ClientContext* context, const ::kinectrgbd::BitStream& request, ::kinectrgbd::Response* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>> AsyncReturnPixelBoundsFromSpaceBounds(::grpc::ClientContext* context, const ::kinectrgbd::BitStream& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>>(AsyncReturnPixelBoundsFromSpaceBoundsRaw(context, request, cq));
    }
    virtual ::grpc::Status ReturnCognition(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::kinectrgbd::Response* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>> AsyncReturnCognition(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>>(AsyncReturnCognitionRaw(context, request, cq));
    }
  private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Request>* AsyncCheckRequestRaw(::grpc::ClientContext* context, const ::kinectrgbd::Header& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>* AsyncSendPointsRaw(::grpc::ClientContext* context, const ::kinectrgbd::Points& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>* AsyncSendImageRaw(::grpc::ClientContext* context, const ::kinectrgbd::Pixels& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>* AsyncReturnPositionsFromPixelsRaw(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>* AsyncReturnPixelBoundsFromSpaceBoundsRaw(::grpc::ClientContext* context, const ::kinectrgbd::BitStream& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::kinectrgbd::Response>* AsyncReturnCognitionRaw(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub GRPC_FINAL : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    ::grpc::Status CheckRequest(::grpc::ClientContext* context, const ::kinectrgbd::Header& request, ::kinectrgbd::Request* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Request>> AsyncCheckRequest(::grpc::ClientContext* context, const ::kinectrgbd::Header& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Request>>(AsyncCheckRequestRaw(context, request, cq));
    }
    ::grpc::Status SendPoints(::grpc::ClientContext* context, const ::kinectrgbd::Points& request, ::kinectrgbd::Response* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>> AsyncSendPoints(::grpc::ClientContext* context, const ::kinectrgbd::Points& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>>(AsyncSendPointsRaw(context, request, cq));
    }
    ::grpc::Status SendImage(::grpc::ClientContext* context, const ::kinectrgbd::Pixels& request, ::kinectrgbd::Response* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>> AsyncSendImage(::grpc::ClientContext* context, const ::kinectrgbd::Pixels& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>>(AsyncSendImageRaw(context, request, cq));
    }
    ::grpc::Status ReturnPositionsFromPixels(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::kinectrgbd::Response* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>> AsyncReturnPositionsFromPixels(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>>(AsyncReturnPositionsFromPixelsRaw(context, request, cq));
    }
    ::grpc::Status ReturnPixelBoundsFromSpaceBounds(::grpc::ClientContext* context, const ::kinectrgbd::BitStream& request, ::kinectrgbd::Response* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>> AsyncReturnPixelBoundsFromSpaceBounds(::grpc::ClientContext* context, const ::kinectrgbd::BitStream& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>>(AsyncReturnPixelBoundsFromSpaceBoundsRaw(context, request, cq));
    }
    ::grpc::Status ReturnCognition(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::kinectrgbd::Response* response) GRPC_OVERRIDE;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>> AsyncReturnCognition(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>>(AsyncReturnCognitionRaw(context, request, cq));
    }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Request>* AsyncCheckRequestRaw(::grpc::ClientContext* context, const ::kinectrgbd::Header& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>* AsyncSendPointsRaw(::grpc::ClientContext* context, const ::kinectrgbd::Points& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>* AsyncSendImageRaw(::grpc::ClientContext* context, const ::kinectrgbd::Pixels& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>* AsyncReturnPositionsFromPixelsRaw(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>* AsyncReturnPixelBoundsFromSpaceBoundsRaw(::grpc::ClientContext* context, const ::kinectrgbd::BitStream& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    ::grpc::ClientAsyncResponseReader< ::kinectrgbd::Response>* AsyncReturnCognitionRaw(::grpc::ClientContext* context, const ::kinectrgbd::DataStream& request, ::grpc::CompletionQueue* cq) GRPC_OVERRIDE;
    const ::grpc::RpcMethod rpcmethod_CheckRequest_;
    const ::grpc::RpcMethod rpcmethod_SendPoints_;
    const ::grpc::RpcMethod rpcmethod_SendImage_;
    const ::grpc::RpcMethod rpcmethod_ReturnPositionsFromPixels_;
    const ::grpc::RpcMethod rpcmethod_ReturnPixelBoundsFromSpaceBounds_;
    const ::grpc::RpcMethod rpcmethod_ReturnCognition_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status CheckRequest(::grpc::ServerContext* context, const ::kinectrgbd::Header* request, ::kinectrgbd::Request* response);
    virtual ::grpc::Status SendPoints(::grpc::ServerContext* context, const ::kinectrgbd::Points* request, ::kinectrgbd::Response* response);
    virtual ::grpc::Status SendImage(::grpc::ServerContext* context, const ::kinectrgbd::Pixels* request, ::kinectrgbd::Response* response);
    virtual ::grpc::Status ReturnPositionsFromPixels(::grpc::ServerContext* context, const ::kinectrgbd::DataStream* request, ::kinectrgbd::Response* response);
    virtual ::grpc::Status ReturnPixelBoundsFromSpaceBounds(::grpc::ServerContext* context, const ::kinectrgbd::BitStream* request, ::kinectrgbd::Response* response);
    virtual ::grpc::Status ReturnCognition(::grpc::ServerContext* context, const ::kinectrgbd::DataStream* request, ::kinectrgbd::Response* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_CheckRequest : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_CheckRequest() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_CheckRequest() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckRequest(::grpc::ServerContext* context, const ::kinectrgbd::Header* request, ::kinectrgbd::Request* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestCheckRequest(::grpc::ServerContext* context, ::kinectrgbd::Header* request, ::grpc::ServerAsyncResponseWriter< ::kinectrgbd::Request>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_SendPoints : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_SendPoints() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_SendPoints() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendPoints(::grpc::ServerContext* context, const ::kinectrgbd::Points* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSendPoints(::grpc::ServerContext* context, ::kinectrgbd::Points* request, ::grpc::ServerAsyncResponseWriter< ::kinectrgbd::Response>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_SendImage : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_SendImage() {
      ::grpc::Service::MarkMethodAsync(2);
    }
    ~WithAsyncMethod_SendImage() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendImage(::grpc::ServerContext* context, const ::kinectrgbd::Pixels* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSendImage(::grpc::ServerContext* context, ::kinectrgbd::Pixels* request, ::grpc::ServerAsyncResponseWriter< ::kinectrgbd::Response>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(2, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ReturnPositionsFromPixels : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_ReturnPositionsFromPixels() {
      ::grpc::Service::MarkMethodAsync(3);
    }
    ~WithAsyncMethod_ReturnPositionsFromPixels() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ReturnPositionsFromPixels(::grpc::ServerContext* context, const ::kinectrgbd::DataStream* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestReturnPositionsFromPixels(::grpc::ServerContext* context, ::kinectrgbd::DataStream* request, ::grpc::ServerAsyncResponseWriter< ::kinectrgbd::Response>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(3, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ReturnPixelBoundsFromSpaceBounds : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_ReturnPixelBoundsFromSpaceBounds() {
      ::grpc::Service::MarkMethodAsync(4);
    }
    ~WithAsyncMethod_ReturnPixelBoundsFromSpaceBounds() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ReturnPixelBoundsFromSpaceBounds(::grpc::ServerContext* context, const ::kinectrgbd::BitStream* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestReturnPixelBoundsFromSpaceBounds(::grpc::ServerContext* context, ::kinectrgbd::BitStream* request, ::grpc::ServerAsyncResponseWriter< ::kinectrgbd::Response>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(4, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ReturnCognition : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_ReturnCognition() {
      ::grpc::Service::MarkMethodAsync(5);
    }
    ~WithAsyncMethod_ReturnCognition() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ReturnCognition(::grpc::ServerContext* context, const ::kinectrgbd::DataStream* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestReturnCognition(::grpc::ServerContext* context, ::kinectrgbd::DataStream* request, ::grpc::ServerAsyncResponseWriter< ::kinectrgbd::Response>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(5, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_CheckRequest<WithAsyncMethod_SendPoints<WithAsyncMethod_SendImage<WithAsyncMethod_ReturnPositionsFromPixels<WithAsyncMethod_ReturnPixelBoundsFromSpaceBounds<WithAsyncMethod_ReturnCognition<Service > > > > > > AsyncService;
  template <class BaseClass>
  class WithGenericMethod_CheckRequest : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_CheckRequest() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_CheckRequest() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status CheckRequest(::grpc::ServerContext* context, const ::kinectrgbd::Header* request, ::kinectrgbd::Request* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_SendPoints : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_SendPoints() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_SendPoints() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendPoints(::grpc::ServerContext* context, const ::kinectrgbd::Points* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_SendImage : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_SendImage() {
      ::grpc::Service::MarkMethodGeneric(2);
    }
    ~WithGenericMethod_SendImage() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SendImage(::grpc::ServerContext* context, const ::kinectrgbd::Pixels* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ReturnPositionsFromPixels : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_ReturnPositionsFromPixels() {
      ::grpc::Service::MarkMethodGeneric(3);
    }
    ~WithGenericMethod_ReturnPositionsFromPixels() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ReturnPositionsFromPixels(::grpc::ServerContext* context, const ::kinectrgbd::DataStream* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ReturnPixelBoundsFromSpaceBounds : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_ReturnPixelBoundsFromSpaceBounds() {
      ::grpc::Service::MarkMethodGeneric(4);
    }
    ~WithGenericMethod_ReturnPixelBoundsFromSpaceBounds() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ReturnPixelBoundsFromSpaceBounds(::grpc::ServerContext* context, const ::kinectrgbd::BitStream* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ReturnCognition : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_ReturnCognition() {
      ::grpc::Service::MarkMethodGeneric(5);
    }
    ~WithGenericMethod_ReturnCognition() GRPC_OVERRIDE {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ReturnCognition(::grpc::ServerContext* context, const ::kinectrgbd::DataStream* request, ::kinectrgbd::Response* response) GRPC_FINAL GRPC_OVERRIDE {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
};

}  // namespace kinectrgbd


#endif  // GRPC_kinect_5frgbd_2eproto__INCLUDED
