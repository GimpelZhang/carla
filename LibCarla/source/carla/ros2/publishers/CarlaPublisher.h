// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#define _GLIBCXX_USE_CXX11_ABI 0

#include <string>

namespace carla {
namespace ros2 {

  class CarlaPublisher {
    public:
      const std::string& frame_id() const { return _frame_id; }
      const std::string& name() const { return _name; }
      const std::string& parent() const { return _parent; }

      void frame_id(std::string&& frame_id) { _frame_id = std::move(frame_id); }
      void name(std::string&& name) { _name = std::move(name); }
      void parent(std::string&& parent) { _parent = std::move(parent); }

      virtual const char* type() const = 0;

    public:
      CarlaPublisher() = default;
      virtual ~CarlaPublisher() = default;

    protected:
      std::string _frame_id = "";
      std::string _name = "";
      std::string _parent = "";
  };
}
}
