#include "aero_halo_360/self_mask.hpp"

namespace aero_halo_360
{

bool MaskBox::contains(const PointXYZ & point) const
{
  return point.x >= min.x && point.x <= max.x &&
         point.y >= min.y && point.y <= max.y &&
         point.z >= min.z && point.z <= max.z;
}

void SelfMask::setBoxes(std::vector<MaskBox> boxes)
{
  boxes_ = std::move(boxes);
}

void SelfMask::addBox(const MaskBox & box)
{
  boxes_.push_back(box);
}

bool SelfMask::isMasked(const PointXYZ & point) const
{
  for (const auto & box : boxes_) {
    if (box.contains(point)) {
      return true;
    }
  }
  return false;
}

std::vector<PointXYZ> SelfMask::filter(const std::vector<PointXYZ> & points) const
{
  std::vector<PointXYZ> filtered;
  filtered.reserve(points.size());
  for (const auto & point : points) {
    if (!isMasked(point)) {
      filtered.push_back(point);
    }
  }
  return filtered;
}

const std::vector<MaskBox> & SelfMask::boxes() const
{
  return boxes_;
}

}  // namespace aero_halo_360

