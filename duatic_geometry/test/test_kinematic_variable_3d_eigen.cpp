#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <duatic_geometry/geometry_eigen.hpp>

namespace duatic::geometry
{
namespace
{

using Pose = Pose3Dd;
using Twist = Twist3Dd;
using Accel = Accel3Dd;
using Jerk = Jerk3Dd;
using Snap = Snap3Dd;

Eigen::Vector<double, 6> makeVector6(double a, double b, double c, double d, double e, double f)
{
  Eigen::Vector<double, 6> v;
  v << a, b, c, d, e, f;
  return v;
}

// ---------------------------------------------------------------------------
// Type definitions
// ---------------------------------------------------------------------------

TEST(KinematicVariable3DEigenTypes, ScalarTypeIsPropagated)
{
  static_assert(std::is_same_v<Pose::ScalarType, double>);
  static_assert(std::is_same_v<Twist::ScalarType, double>);
  static_assert(std::is_same_v<Accel::ScalarType, double>);
  static_assert(std::is_same_v<Jerk::ScalarType, double>);
  static_assert(std::is_same_v<Snap::ScalarType, double>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTypes, KinematicOrderValuesAreCorrect)
{
  static_assert(Pose::kinematic_order == static_cast<KinematicOrderT>(KinematicOrder::Pose));
  static_assert(Twist::kinematic_order == static_cast<KinematicOrderT>(KinematicOrder::Twist));
  static_assert(Accel::kinematic_order == static_cast<KinematicOrderT>(KinematicOrder::Accel));
  static_assert(Jerk::kinematic_order == static_cast<KinematicOrderT>(KinematicOrder::Jerk));
  static_assert(Snap::kinematic_order == static_cast<KinematicOrderT>(KinematicOrder::Snap));
  SUCCEED();
}

TEST(KinematicVariable3DEigenTypes, PoseUsesQuaternionAngularType)
{
  static_assert(std::is_same_v<Pose::LinearDataType, Eigen::Vector3d>);
  static_assert(std::is_same_v<Pose::AngularDataType, Eigen::Quaterniond>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTypes, DiffOrdersShareA6DVectorRepresentation)
{
  static_assert(std::is_same_v<Twist::DataType, Eigen::Vector<double, 6>>);
  static_assert(std::is_same_v<Accel::DataType, Eigen::Vector<double, 6>>);
  static_assert(std::is_same_v<Jerk::DataType, Eigen::Vector<double, 6>>);
  static_assert(std::is_same_v<Snap::DataType, Eigen::Vector<double, 6>>);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// Concept compliance
// ---------------------------------------------------------------------------

TEST(KinematicVariable3DEigenConcepts, PoseSatisfiesOnlyTheBaseConcept)
{
  static_assert(KinematicVariable3D<Pose>);
  static_assert(!KinematicDiffVariable3D<Pose>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenConcepts, HigherOrdersSatisfyTheDiffConcept)
{
  static_assert(KinematicDiffVariable3D<Twist>);
  static_assert(KinematicDiffVariable3D<Accel>);
  static_assert(KinematicDiffVariable3D<Jerk>);
  static_assert(KinematicDiffVariable3D<Snap>);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// Traits
// ---------------------------------------------------------------------------

TEST(KinematicVariable3DEigenTraits, LinearAndAngularTypeTraitsResolve)
{
  static_assert(std::is_same_v<kinematic_linear_type_t<Pose>, Eigen::Vector3d>);
  static_assert(std::is_same_v<kinematic_angular_type_t<Pose>, Eigen::Quaterniond>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTraits, VectorTypeTraitResolvesForDiffOrders)
{
  static_assert(std::is_same_v<kinematic_vector_type_t<Twist>, Eigen::Vector<double, 6>>);
  static_assert(std::is_same_v<kinematic_vector_type_t<Accel>, Eigen::Vector<double, 6>>);
  static_assert(std::is_same_v<kinematic_vector_type_t<Jerk>, Eigen::Vector<double, 6>>);
  static_assert(std::is_same_v<kinematic_vector_type_t<Snap>, Eigen::Vector<double, 6>>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTraits, DiffTypeTraitFollowsTheOrderChain)
{
  static_assert(std::is_same_v<kinematic_diff_type_t<Pose>, Twist>);
  static_assert(std::is_same_v<kinematic_diff_type_t<Twist>, Accel>);
  static_assert(std::is_same_v<kinematic_diff_type_t<Accel>, Jerk>);
  static_assert(std::is_same_v<kinematic_diff_type_t<Jerk>, Snap>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTraits, IsKinematicDiffOfMatchesTheOrderChain)
{
  static_assert(is_kinematic_diff_of_v<Twist, Pose>);
  static_assert(is_kinematic_diff_of_v<Accel, Twist>);
  static_assert(is_kinematic_diff_of_v<Jerk, Accel>);
  static_assert(is_kinematic_diff_of_v<Snap, Jerk>);
  static_assert(!is_kinematic_diff_of_v<Accel, Pose>);
  static_assert(!is_kinematic_diff_of_v<Pose, Twist>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTraits, IsSameKinematicOrderDistinguishesOrders)
{
  static_assert(is_same_kinematic_order_v<Pose, Pose>);
  static_assert(is_same_kinematic_order_v<Twist, Twist>);
  static_assert(!is_same_kinematic_order_v<Pose, Twist>);
  static_assert(!is_same_kinematic_order_v<Accel, Snap>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenTraits, IsKinematicVariable3DAcceptsAllFiveConcretisations)
{
  static_assert(is_kinematic_variable_3d_v<Pose>);
  static_assert(is_kinematic_variable_3d_v<Twist>);
  static_assert(is_kinematic_variable_3d_v<Accel>);
  static_assert(is_kinematic_variable_3d_v<Jerk>);
  static_assert(is_kinematic_variable_3d_v<Snap>);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// Basic operators
// ---------------------------------------------------------------------------

TEST(KinematicVariable3DEigenOperators, PoseSetLinearAngularAndFullNeutral)
{
  Pose pose;
  pose.linear() = Eigen::Vector3d(1, 2, 3);
  pose.angular() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);

  pose.setLinearNeutral();
  EXPECT_TRUE(pose.linear().isApprox(Eigen::Vector3d::Zero()));

  pose.setAngularNeutral();
  EXPECT_TRUE(pose.angular().isApprox(Eigen::Quaterniond::Identity()));

  pose.linear() = Eigen::Vector3d(1, 2, 3);
  pose.angular() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  pose.setNeutral();
  EXPECT_TRUE(pose.linear().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(pose.angular().isApprox(Eigen::Quaterniond::Identity()));
}

TEST(KinematicVariable3DEigenOperators, TwistSetNeutralZeroesTheWholeVector)
{
  Twist twist(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
  twist.setNeutral();
  EXPECT_TRUE(twist.vector().isApprox(Eigen::Vector<double, 6>::Zero()));
}

TEST(KinematicVariable3DEigenOperators, CompoundAssignmentPlusMinusTimes)
{
  Twist a(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
  const Twist b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));

  a += b;
  EXPECT_TRUE(a.vector().isApprox(makeVector6(2, 3, 4, 5, 6, 7)));

  a -= b;
  EXPECT_TRUE(a.vector().isApprox(makeVector6(1, 2, 3, 4, 5, 6)));

  a *= 2.0;
  EXPECT_TRUE(a.vector().isApprox(makeVector6(2, 4, 6, 8, 10, 12)));
}

TEST(KinematicVariable3DEigenOperators, FreeOperatorPlusAndTimes)
{
  const Twist a(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
  const Twist b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));

  const Twist sum = a + b;
  EXPECT_TRUE(sum.vector().isApprox(makeVector6(2, 3, 4, 5, 6, 7)));

  const Twist scaled = a * 2.0;
  EXPECT_TRUE(scaled.vector().isApprox(makeVector6(2, 4, 6, 8, 10, 12)));
}

TEST(KinematicVariable3DEigenOperators, MinusOperatorProducesTheNextHigherOrder)
{
  const Pose p1(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond::Identity());
  const Pose p2(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());

  const auto pose_diff = p1 - p2;
  static_assert(std::is_same_v<decltype(pose_diff), const Twist>);
  EXPECT_TRUE(pose_diff.linear().isApprox(Eigen::Vector3d(1, 2, 3)));

  const Twist t1(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2));
  const Twist t2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

  const auto twist_diff = t1 - t2;
  static_assert(std::is_same_v<decltype(twist_diff), const Accel>);
  EXPECT_TRUE(twist_diff.vector().isApprox(t1.vector()));
}

TEST(KinematicVariable3DEigenOperators, StreamingOperatorProducesNonEmptyOutput)
{
  Pose pose;
  pose.setNeutral();
  std::ostringstream pose_stream;
  pose_stream << pose;
  EXPECT_FALSE(pose_stream.str().empty());

  Twist twist;
  twist.setNeutral();
  std::ostringstream twist_stream;
  twist_stream << twist;
  EXPECT_FALSE(twist_stream.str().empty());
}

}  // namespace
}  // namespace duatic::geometry
