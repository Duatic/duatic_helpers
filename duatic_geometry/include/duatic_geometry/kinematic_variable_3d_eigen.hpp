/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>
#include <ostream>
#include <utility>
#include <duatic_geometry/kinematic_variable.hpp>
#include <duatic_geometry/kinematic_variable_traits.hpp>

namespace duatic::geometry
{

template <typename ScalarT, KinematicOrder Order>
class KinematicVariable3DEigen;

// Pose concretisation: linear part is a cartesian vector, angular part is a quaternion
template <typename ScalarT>
class KinematicVariable3DEigen<ScalarT, KinematicOrder::Pose>
{
public:
  using ScalarType = ScalarT;
  template <KinematicOrder NewOrder>
  using SelfWithOrder = KinematicVariable3DEigen<ScalarType, NewOrder>;
  using Self = SelfWithOrder<KinematicOrder::Pose>;

  using LinearDataType = Eigen::Vector<ScalarType, 3>;
  using AngularDataType = Eigen::Quaternion<ScalarType>;

  static constexpr KinematicOrder kinematic_order = KinematicOrder::Pose;
  static_assert(Self::kinematic_order == KinematicOrder::Pose);

  inline constexpr KinematicVariable3DEigen() = default;
  inline constexpr KinematicVariable3DEigen(const Self& other) = default;  // NOLINT(runtime/explicit)
  inline constexpr KinematicVariable3DEigen(Self&& other) = default;  // NOLINT(runtime/explicit)

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename LinearCtor, typename AngularCtor>
  inline constexpr KinematicVariable3DEigen(const LinearCtor& position_init, const AngularCtor& orientation_init)
    : position(position_init), orientation(orientation_init)
  {
  }

  inline LinearDataType& linear()
  {
    return position;
  }
  inline const LinearDataType& linear() const
  {
    return position;
  }

  inline AngularDataType& angular()
  {
    return orientation;
  }
  inline const AngularDataType& angular() const
  {
    return orientation;
  }

  inline Self& setLinearNeutral()
  {
    position.setZero();
    return *this;
  }

  inline Self& setAngularNeutral()
  {
    orientation.setIdentity();
    return *this;
  }

  inline Self& setNeutral()
  {
    return setLinearNeutral().setAngularNeutral();
  }

  inline Self& operator+=(const SelfWithOrder<KinematicOrder::Twist>& other)
  {
    position += other.linear();
    const ScalarT angle_other = other.angular().norm();
    if (angle_other > std::numeric_limits<ScalarT>::epsilon()) {
      const Eigen::Quaternion<ScalarT> orientation_diff =
          Eigen::Quaternion<ScalarT>(Eigen::AngleAxis<ScalarT>(angle_other, other.angular() / angle_other));
      orientation = (orientation_diff * orientation).normalized();
    }
    return *this;
  }

  inline Self& operator-=(const SelfWithOrder<KinematicOrder::Twist>& other)
  {
    position -= other.linear();
    const ScalarT angle_other = other.angular().norm();
    if (angle_other > std::numeric_limits<ScalarT>::epsilon()) {
      const Eigen::Quaternion<ScalarT> orientation_diff =
          Eigen::Quaternion<ScalarT>(Eigen::AngleAxis<ScalarT>(angle_other, other.angular() / angle_other));
      orientation = (orientation_diff.conjugate() * orientation).normalized();
    }
    return *this;
  }

  Self operator-() const
  {
    return Self(-position, orientation.conjugate());
  }

private:
  LinearDataType position;
  AngularDataType orientation;
};

// Generic concretisation: a 3D linear/angular pair backed by a single 6D vector, used for every
// derivative order beyond the pose itself (twist, acceleration, jerk, snap, ...).
template <typename ScalarT, KinematicOrder Order>
class KinematicVariable3DEigen
{
public:
  using ScalarType = ScalarT;
  template <KinematicOrder NewOrder>
  using SelfWithOrder = KinematicVariable3DEigen<ScalarType, NewOrder>;
  using Self = SelfWithOrder<Order>;

  using DataType = Eigen::Vector<ScalarType, 6>;

  static constexpr KinematicOrder kinematic_order = Order;
  static_assert(Self::kinematic_order > KinematicOrder::Pose);

  inline constexpr KinematicVariable3DEigen() = default;
  inline constexpr KinematicVariable3DEigen(const Self& other) = default;  // NOLINT(runtime/explicit)
  inline constexpr KinematicVariable3DEigen(Self&& other) = default;  // NOLINT(runtime/explicit)

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename VectorCtor>
  inline explicit KinematicVariable3DEigen(const VectorCtor& vector_init) : vector_(vector_init)
  {
  }

  template <typename LinearCtor, typename AngularCtor>
  inline KinematicVariable3DEigen(const LinearCtor& linear_init, const AngularCtor& angular_init) : vector_()
  {
    linear() = linear_init;
    angular() = angular_init;
  }

  inline auto linear()
  {
    return vector_.segment(0, 3);
  }
  inline auto linear() const
  {
    return vector_.segment(0, 3);
  }

  inline auto angular()
  {
    return vector_.segment(3, 3);
  }
  inline auto angular() const
  {
    return vector_.segment(3, 3);
  }

  inline DataType& vector()
  {
    return vector_;
  }
  inline const DataType& vector() const
  {
    return vector_;
  }

  inline Self& setLinearNeutral()
  {
    linear().setZero();
    return *this;
  }

  inline Self& setAngularNeutral()
  {
    angular().setZero();
    return *this;
  }

  inline Self& setNeutral()
  {
    vector().setZero();
    return *this;
  }

  template <KinematicOrder OtherOrder>
    requires((OtherOrder == Order) || (OtherOrder == Order + 1))
  inline Self& operator+=(const SelfWithOrder<OtherOrder>& other)
  {
    // other.vector() (public), not other.vector_ (private): when OtherOrder != Order, other is a
    // different instantiation of this same class template, and template instantiations don't
    // implicitly grant each other access to private members.
    vector_ += other.vector();
    return *this;
  }

  template <KinematicOrder OtherOrder>
    requires((OtherOrder == Order) || (OtherOrder == Order + 1))
  inline Self& operator-=(const SelfWithOrder<OtherOrder>& other)
  {
    vector_ -= other.vector();
    return *this;
  }

  inline Self& operator*=(const ScalarType& scalar)
  {
    vector_ *= scalar;
    return *this;
  }

  inline Self operator-() const
  {
    return Self(-vector_);
  }

private:
  DataType vector_;
};

// special '-' operator declared outside to avoid infinite type recursion
template <typename ScalarT, KinematicOrder Order>
inline auto operator-(const KinematicVariable3DEigen<ScalarT, Order>& lhs,
                      const KinematicVariable3DEigen<ScalarT, Order>& rhs)
{
  using return_type = KinematicVariable3DEigen<ScalarT, Order + 1>;
  static_assert(kinematic_diff_of_helper<return_type, KinematicVariable3DEigen<ScalarT, Order>>());

  if constexpr (Order == KinematicOrder::Pose) {  // special case for pose
    const Eigen::AngleAxis<ScalarT> orientation_axis(lhs.angular() * rhs.angular().conjugate());
    return return_type(lhs.linear() - rhs.linear(), orientation_axis.angle() * orientation_axis.axis());
  } else {  // everything else
    return return_type(lhs.vector() - rhs.vector());
  }
}

}  // namespace duatic::geometry
