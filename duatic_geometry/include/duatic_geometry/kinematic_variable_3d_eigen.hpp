#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ostream>
#include <utility>
#include <duatic_geometry/kinematic_variable_3d.hpp>
#include <duatic_geometry/kinematic_variable_3d_traits.hpp>

namespace duatic::geometry
{

template <typename ScalarT, KinematicOrderT Order>
class KinematicVariable3DEigen;

// Pose concretisation: linear part is a cartesian vector, angular part is a quaternion
template <typename ScalarT>
class KinematicVariable3DEigen<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Pose)>
{
public:
  using ScalarType = ScalarT;
  using Self = KinematicVariable3DEigen<ScalarType, static_cast<KinematicOrderT>(KinematicOrder::Pose)>;

  using LinearDataType = Eigen::Vector<ScalarType, 3>;
  using AngularDataType = Eigen::Quaternion<ScalarType>;

  static constexpr KinematicOrderT kinematic_order = static_cast<KinematicOrderT>(KinematicOrder::Pose);
  static_assert(Self::kinematic_order == 0);

  inline constexpr KinematicVariable3DEigen() = default;
  inline constexpr KinematicVariable3DEigen(const Self& other) = default;
  inline constexpr KinematicVariable3DEigen(Self&& other) = default;

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

private:
  LinearDataType position;
  AngularDataType orientation;
};

// Generic concretisation: a 3D linear/angular pair backed by a single 6D vector, used for every
// derivative order beyond the pose itself (twist, acceleration, jerk, snap, ...).
template <typename ScalarT, KinematicOrderT Order>
class KinematicVariable3DEigen
{
public:
  using ScalarType = ScalarT;
  using Self = KinematicVariable3DEigen<ScalarType, Order>;

  using DataType = Eigen::Vector<ScalarType, 6>;

  static constexpr KinematicOrderT kinematic_order = Order;
  static_assert(Self::kinematic_order > 0);

  inline constexpr KinematicVariable3DEigen() = default;
  inline constexpr KinematicVariable3DEigen(const Self& other) = default;
  inline constexpr KinematicVariable3DEigen(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename VectorCtor>
  inline KinematicVariable3DEigen(const VectorCtor& vector_init) : vector_(vector_init)
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

  inline Self& operator+=(const Self& other)
  {
    vector_ += other.vector_;
    return *this;
  }

  inline Self& operator-=(const Self& other)
  {
    vector_ -= other.vector_;
    return *this;
  }

  inline Self& operator*=(const ScalarType& scalar)
  {
    vector_ *= scalar;
    return *this;
  }

private:
  DataType vector_;
};

// special '-' operator declared outside to avoid infinite type recursion
template <typename ScalarT, KinematicOrderT Order>
inline auto operator-(const KinematicVariable3DEigen<ScalarT, Order>& lhs,
                      const KinematicVariable3DEigen<ScalarT, Order>& rhs)
{
  using return_type = KinematicVariable3DEigen<ScalarT, Order + 1>;
  static_assert(kinematic_diff_of<return_type, KinematicVariable3DEigen<ScalarT, Order>>::value);

  if constexpr (Order == static_cast<KinematicOrderT>(KinematicOrder::Pose)) {  // special case for pose
    const Eigen::AngleAxis<ScalarT> orientation_axis(lhs.angular() * rhs.angular().conjugate());
    return return_type(lhs.linear() - rhs.linear(), orientation_axis.angle() * orientation_axis.axis());
  } else {  // everything else
    return return_type(lhs.vector() - rhs.vector());
  }
}

}  // namespace duatic::geometry
