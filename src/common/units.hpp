#pragma once

namespace gripper::common {

// Distance measured in millimeters.
struct Millimeter {
  double value{0.0};
};

// Linear speed measured in millimeters per second.
struct MillimeterPerSecond {
  double value{0.0};
};

// Linear acceleration measured in millimeters per second squared.
struct MillimeterPerSecondSquared {
  double value{0.0};
};

// Angle measured in radians.
struct Radian {
  double value{0.0};
};

// Angular speed measured in radians per second.
struct RadianPerSecond {
  double value{0.0};
};

// Angular acceleration measured in radians per second squared.
struct RadianPerSecondSquared {
  double value{0.0};
};

// Electric current measured in amperes.
struct Ampere {
  double value{0.0};
};

// Torque measured in newton-meters.
struct NewtonMeter {
  double value{0.0};
};

// Force measured in newtons.
struct Newton {
  double value{0.0};
};

// Motor or mechanism output power measured in watts.
struct Watt {
  double value{0.0};
};

// Time measured in seconds for control parameters and timeouts.
struct Second {
  double value{0.0};
};

// Temperature measured in degrees Celsius.
struct Celsius {
  double value{0.0};
};

// Dimensionless ratio, coefficient, or normalized score.
struct Ratio {
  double value{0.0};
};

using Mm = Millimeter;
using MmPerS = MillimeterPerSecond;
using MmPerS2 = MillimeterPerSecondSquared;
using Rad = Radian;
using RadPerS = RadianPerSecond;
using RadPerS2 = RadianPerSecondSquared;
using A = Ampere;
using Nm = NewtonMeter;
using N = Newton;
using W = Watt;
using S = Second;
using DegC = Celsius;

}  // namespace gripper::common
