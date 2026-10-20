// Copyright (c) 2026 The Franka Dual Arm Project
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <memory>
#include <optional>

namespace dual_arm_carry_task
{

enum class AsyncServiceStatus { IDLE, PENDING, READY, TIMEOUT };

// One outstanding service request per query object.
//
// `poll` only inspects the future's current state (`wait_for(0)`), so a slow or
// lost service never blocks the caller's reference/control loop. A request that
// exceeds the caller's single-query budget is abandoned explicitly, which also
// removes the client's pending-request bookkeeping.
template<class ClientT>
class AsyncServiceQuery
{
public:
  using Request = typename ClientT::Request;
  using SharedResponse = typename ClientT::SharedResponse;
  using FutureAndRequestId = typename ClientT::FutureAndRequestId;

  bool start(ClientT& client, std::chrono::steady_clock::time_point now)
  {
    if (future_) return false;
    if (!client.service_is_ready()) return false;
    future_.emplace(client.async_send_request(std::make_shared<Request>()));
    started_ = now;
    response_.reset();
    return true;
  }

  AsyncServiceStatus poll(std::chrono::steady_clock::time_point now,
                          std::chrono::milliseconds single_query_budget)
  {
    if (!future_) return AsyncServiceStatus::IDLE;
    if (future_->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      response_ = future_->get();
      future_.reset();
      return AsyncServiceStatus::READY;
    }
    if (now - started_ >= single_query_budget) return AsyncServiceStatus::TIMEOUT;
    return AsyncServiceStatus::PENDING;
  }

  // Drop a request that did not answer in time. A late answer is ignored.
  void abandon(ClientT& client)
  {
    if (!future_) return;
    client.remove_pending_request(*future_);
    future_.reset();
    response_.reset();
  }

  bool inFlight() const { return future_.has_value(); }
  SharedResponse response() const { return response_; }

private:
  std::optional<FutureAndRequestId> future_;
  SharedResponse response_;
  std::chrono::steady_clock::time_point started_{};
};

}  // namespace dual_arm_carry_task
