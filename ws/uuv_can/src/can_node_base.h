// Original code from: https://github.com/DemianMArin/my_vanttec_uuv.git
// Modified by: Abraham de Jesus Maldonado Mata
// Date: 06/02/2025

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <boost/asio.hpp>
#include "Vanttec_CANLib/CANMessage.h"

class CanNodeBase : public rclcpp::Node {
public:
  CanNodeBase(const std::string &name);

  void stop();
protected:
    void send_frame(uint32_t address, const vanttec::CANMessage &msg);
    void send_frame(const struct can_frame frame);
    virtual void parse_frame(const struct can_frame &frame) = 0;
    virtual void can_send_confirm_cb();

private:
  void can_listener(struct can_frame &frame,
                    boost::asio::posix::basic_stream_descriptor<> &stream);

private:
  boost::asio::io_service ios_;
  boost::asio::posix::basic_stream_descriptor<> can_stream_;
  boost::asio::signal_set signals_;

  int fd_;
  struct can_frame tmp_frame_;
};
