#pragma once

#include <string>
#include <vector>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

struct MaskBox
{
  std::string name;
  PointXYZ min;
  PointXYZ max;

  bool contains(const PointXYZ & point) const;
};

class SelfMask
{
public:
  void setBoxes(std::vector<MaskBox> boxes);
  void addBox(const MaskBox & box);
  bool isMasked(const PointXYZ & point) const;
  std::vector<PointXYZ> filter(const std::vector<PointXYZ> & points) const;
  const std::vector<MaskBox> & boxes() const;

private:
  std::vector<MaskBox> boxes_;
};

}  // namespace aero_halo_360

