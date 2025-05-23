// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/rpc/Location.h"

#include <cstdint>
#include <vector>
#include <numeric>

namespace carla {

namespace ros2 {
  class ROS2;
}

namespace sensor {

namespace s11n {
  class SemanticLidarSerializer;
  class SemanticLidarHeaderView;
}

namespace data {

  /// Helper class to store and serialize the data generated by a RawLidar.
  ///
  /// The header of a Lidar measurement consists of an array of uint32_t's in
  /// the following layout
  ///
  ///    {
  ///      Horizontal angle (float),
  ///      Channel count,
  ///      Point count of channel 0,
  ///      ...
  ///      Point count of channel n,
  ///    }
  ///
  /// The points are stored in an array of detections, each detection consist in
  /// four floats, the point detected and the angle between the casted ray and
  /// the normal of the hitted object, and two unsigned integers, the index
  /// and the semantic tag of the hitted object
  ///
  ///    {
  ///      X0, Y0, Z0, Cos(TH0), idx_0, tag_0
  ///      ...
  ///      Xn, Yn, Zn, Cos(THn), idx_n, tag_n
  ///    }
  ///

  #pragma pack(push, 1)
  class SemanticLidarDetection {
    public:
      geom::Location point{};
      float cos_inc_angle{};
      uint32_t object_idx{};
      uint32_t object_tag{};

      SemanticLidarDetection() = default;

      SemanticLidarDetection(float x, float y, float z, float cosTh, uint32_t idx, uint32_t tag) :
          point(x, y, z), cos_inc_angle{cosTh}, object_idx{idx}, object_tag{tag} { }
      SemanticLidarDetection(geom::Location p, float cosTh, uint32_t idx, uint32_t tag) :
          point(p), cos_inc_angle{cosTh}, object_idx{idx}, object_tag{tag} { }

      void WritePlyHeaderInfo(std::ostream& out) const{
        out << "property float32 x\n" \
           "property float32 y\n" \
           "property float32 z\n" \
           "property float32 CosAngle\n" \
           "property uint32 ObjIdx\n" \
           "property uint32 ObjTag";
      }

      void WriteDetection(std::ostream& out) const{
        out << point.x << ' ' << point.y << ' ' << point.z << ' ' \
          << cos_inc_angle << ' ' << object_idx << ' ' << object_tag;
      }
  };
  #pragma pack(pop)

  class SemanticLidarData {
    static_assert(sizeof(float) == sizeof(uint32_t), "Invalid float size");

  protected:
    enum Index : size_t {
      HorizontalAngle,
      ChannelCount,
      SIZE
    };

  public:
    explicit SemanticLidarData(uint32_t ChannelCount = 0u)
      : _header(Index::SIZE + ChannelCount, 0u) {
      _header[Index::ChannelCount] = ChannelCount;
    }

    SemanticLidarData &operator=(SemanticLidarData &&) = default;

    virtual ~SemanticLidarData() {}

    float GetHorizontalAngle() const {
      return reinterpret_cast<const float &>(_header[Index::HorizontalAngle]);
    }

    uint32_t GetidxPtsOneLaser() const {
      return reinterpret_cast<const uint32_t &>(M1_header[0]);
    }    
    void SetHorizontalAngle(float angle) {
      std::memcpy(&_header[Index::HorizontalAngle], &angle, sizeof(uint32_t));
    }

    void SetidxPtsOneLaser(int idxPtsOneLaser) {
      std::memcpy(&M1_header[0], &idxPtsOneLaser, sizeof(uint32_t));
    }        

    uint32_t GetChannelCount() const {
      return _header[Index::ChannelCount];
    }

    virtual void ResetMemory(std::vector<uint32_t> points_per_channel) {
      DEBUG_ASSERT(GetChannelCount() > points_per_channel.size());
      std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());
        uint32_t total_points = static_cast<uint32_t>(
          std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));
      _ser_points.clear();
      _ser_points.reserve(total_points);
    }

    virtual void WriteChannelCount(std::vector<uint32_t> points_per_channel) {
      for (auto idxChannel = 0u; idxChannel < GetChannelCount(); ++idxChannel)
        _header[Index::SIZE + idxChannel] = points_per_channel[idxChannel];
    }

    virtual void WritePointSync(SemanticLidarDetection &detection) {
      _ser_points.emplace_back(detection);
    }

  protected:
    std::vector<uint32_t> _header;
    std::vector<uint32_t> M1_header={0,0};
    uint32_t _max_channel_points;

  private:
    std::vector<SemanticLidarDetection> _ser_points;

  friend class s11n::SemanticLidarHeaderView;
  friend class s11n::SemanticLidarSerializer;
  friend class carla::ros2::ROS2;
  
  };

} // namespace s11n
} // namespace sensor
} // namespace carla
