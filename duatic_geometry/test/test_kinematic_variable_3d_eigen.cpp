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
  static_assert(Pose::kinematic_order == KinematicOrder::Pose);
  static_assert(Twist::kinematic_order == KinematicOrder::Twist);
  static_assert(Accel::kinematic_order == KinematicOrder::Accel);
  static_assert(Jerk::kinematic_order == KinematicOrder::Jerk);
  static_assert(Snap::kinematic_order == KinematicOrder::Snap);
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
  static_assert(KinematicVariable<Pose>);
  static_assert(!KinematicDiffVariable<Pose>);
  SUCCEED();
}

TEST(KinematicVariable3DEigenConcepts, HigherOrdersSatisfyTheDiffConcept)
{
  static_assert(KinematicDiffVariable<Twist>);
  static_assert(KinematicDiffVariable<Accel>);
  static_assert(KinematicDiffVariable<Jerk>);
  static_assert(KinematicDiffVariable<Snap>);
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
  static_assert(is_kinematic_variable_v<Pose>);
  static_assert(is_kinematic_variable_v<Twist>);
  static_assert(is_kinematic_variable_v<Accel>);
  static_assert(is_kinematic_variable_v<Jerk>);
  static_assert(is_kinematic_variable_v<Snap>);
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

TEST(KinematicVariable3DEigenOperators, PlusOperatorAddsADiffBackOntoTheBaseOrder)
{
  const Pose p1(Eigen::Vector3d(1, 2, 3),
                Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 0, 0).normalized())));
  const Pose p2(Eigen::Vector3d(0, 0, 0),
                Eigen::Quaterniond(Eigen::AngleAxisd(0.9, Eigen::Vector3d(0, 1, 0).normalized())));

  const auto pose_diff = p1 - p2;
  static_assert(std::is_same_v<decltype(pose_diff), const Twist>);

  const auto reconstructed_pose = p2 + pose_diff;
  static_assert(std::is_same_v<decltype(reconstructed_pose), const Pose>);
  EXPECT_TRUE(reconstructed_pose.linear().isApprox(p1.linear()));
  // A quaternion and its negation represent the same rotation.
  EXPECT_TRUE(reconstructed_pose.angular().isApprox(p1.angular()) ||
              reconstructed_pose.angular().coeffs().isApprox(-p1.angular().coeffs()));

  const Twist t1(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2));
  const Twist t2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

  const auto twist_diff = t1 - t2;
  static_assert(std::is_same_v<decltype(twist_diff), const Accel>);

  const auto reconstructed_twist = t2 + twist_diff;
  static_assert(std::is_same_v<decltype(reconstructed_twist), const Twist>);
  EXPECT_TRUE(reconstructed_twist.vector().isApprox(t1.vector()));
}

TEST(KinematicVariable3DEigenOperators, PlusOperatorHandlesZeroRotationDiffWithoutNaN)
{
  const Pose p1(Eigen::Vector3d(5, 5, 5), Eigen::Quaterniond::Identity());
  const Pose p2(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());

  const auto pose_diff = p1 - p2;
  const auto reconstructed_pose = p2 + pose_diff;

  EXPECT_TRUE(reconstructed_pose.angular().coeffs().allFinite());
  EXPECT_TRUE(reconstructed_pose.linear().isApprox(p1.linear()));
  EXPECT_TRUE(reconstructed_pose.angular().isApprox(p1.angular()));
}

TEST(KinematicVariable3DEigenOperators, MinusOperatorSubtractsADiffFromTheBaseOrder)
{
  const Pose base(Eigen::Vector3d(1, 2, 3),
                  Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 0, 0).normalized())));
  const Twist diff(Eigen::Vector3d(0.5, -0.5, 1.0), Eigen::Vector3d(0.2, 0.1, -0.3));

  const auto reduced = base - diff;
  static_assert(std::is_same_v<decltype(reduced), const Pose>);

  const auto round_trip = reduced + diff;
  EXPECT_TRUE(round_trip.linear().isApprox(base.linear(), 1e-9));
  EXPECT_TRUE(round_trip.angular().isApprox(base.angular(), 1e-9) ||
              round_trip.angular().coeffs().isApprox(-base.angular().coeffs(), 1e-9));

  const Twist t(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
  const Accel a(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6));

  const auto reduced_twist = t - a;
  static_assert(std::is_same_v<decltype(reduced_twist), const Twist>);
  EXPECT_TRUE((reduced_twist + a).vector().isApprox(t.vector(), 1e-9));
}

TEST(KinematicVariable3DEigenOperators, UnaryMinusNegatesTheVector)
{
  const Twist t(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
  const Twist negated = -t;
  EXPECT_TRUE(negated.vector().isApprox(-t.vector()));
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
