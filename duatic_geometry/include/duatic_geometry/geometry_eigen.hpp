#pragma once

#include <duatic_geometry/kinematic_variable.hpp>
#include <duatic_geometry/kinematic_variable_traits.hpp>
#include <duatic_geometry/kinematic_variable_operators.hpp>

// the eigen implementation
#include <duatic_geometry/kinematic_variable_3d_eigen.hpp>

namespace duatic::geometry
{

// Define the one basic implementation
template <typename ScalarT, KinematicOrder Order>
using KinematicVariable3DT = KinematicVariable3DEigen<ScalarT, Order>;

}  // namespace duatic::geometry

// include all derived type definitions
#include <duatic_geometry/kinematic_types.hpp>