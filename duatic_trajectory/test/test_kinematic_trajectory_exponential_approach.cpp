#include <gtest/gtest.h>

#include <duatic_trajectory/trajectory_eigen.hpp>

namespace duatic::trajectory
{
namespace
{

using PoseTrajectory = ExponentialApproachPose3DPoseStated;
using TwistTrajectory = ExponentialApproachPose3DTwistStated;

static_assert(KinematicTrajectory<PoseTrajectory>);
static_assert(KinematicTrajectory<TwistTrajectory>);

geometry::Pose3Dd makePose(double x, double y, double z, const Eigen::Quaterniond& orientation)
{
  return geometry::Pose3Dd(Eigen::Vector3d(x, y, z), orientation);
}

TEST(KinematicTrajectoryExponentialApproach, EvaluateAtUpdateTimeReproducesTheInitialPose)
{
  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const PoseTrajectory::KinematicUpdateState in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  PoseTrajectory traj;
  traj.update(in_state, goal);

  const auto eval = traj.evaluate(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
}

TEST(KinematicTrajectoryExponentialApproach, PoseDepthConvergesToTheGoalOverTime)
{
  const geometry::Pose3Dd x0 =
      makePose(5, -3, 2, Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 0, 1))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.1, 0.1, 0.1));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const PoseTrajectory::KinematicUpdateState in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  PoseTrajectory traj;
  traj.update(in_state, goal);

  const auto eval_far = traj.evaluate(rclcpp::Time(50, 0));
  // goal.linear() is the exact zero vector here, and Eigen's isApprox() is a *relative*
  // comparison that never succeeds against an exact zero reference no matter how close the
  // actual value is - so compare the absolute difference instead.
  EXPECT_LT((eval_far.pose().linear() - goal.linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(goal.angular(), 1e-4));
}

TEST(KinematicTrajectoryExponentialApproach, TwistDepthReproducesTheInitialPoseAndTwist)
{
  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0.05, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const TwistTrajectory::KinematicUpdateState in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  TwistTrajectory traj;
  traj.update(in_state, goal);

  const auto eval = traj.evaluate(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
  EXPECT_TRUE(eval.twist().linear().isApprox(v0.linear(), 1e-9));
  EXPECT_TRUE(eval.twist().angular().isApprox(v0.angular(), 1e-9));
}

TEST(KinematicTrajectoryExponentialApproach, TwistDepthConvergesToTheGoalWithVanishingVelocity)
{
  const geometry::Pose3Dd x0 =
      makePose(5, -3, 2, Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 0, 1))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.1, 0.1, 0.1));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const TwistTrajectory::KinematicUpdateState in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  TwistTrajectory traj;
  traj.update(in_state, goal);

  const auto eval_far = traj.evaluate(rclcpp::Time(50, 0));
  // goal.linear() is the exact zero vector here, and Eigen's isApprox() is a *relative*
  // comparison that never succeeds against an exact zero reference no matter how close the
  // actual value is - so compare the absolute difference instead.
  EXPECT_LT((eval_far.pose().linear() - goal.linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(goal.angular(), 1e-4));
  EXPECT_LT(eval_far.twist().linear().norm(), 1e-4);
  EXPECT_LT(eval_far.twist().angular().norm(), 1e-4);
}

TEST(KinematicTrajectoryExponentialApproach, OutParamAndByValueEvaluateAgree)
{
  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const TwistTrajectory::KinematicUpdateState in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  TwistTrajectory traj;
  traj.update(in_state, goal);

  const rclcpp::Time query_time(3, 0);
  const auto by_value = traj.evaluate(query_time);

  TwistTrajectory::KinematicEvalState out_param;
  traj.evaluate(query_time, out_param);

  EXPECT_TRUE(out_param.pose().linear().isApprox(by_value.pose().linear(), 1e-12));
  EXPECT_TRUE(out_param.pose().angular().isApprox(by_value.pose().angular(), 1e-12));
  EXPECT_TRUE(out_param.twist().vector().isApprox(by_value.twist().vector(), 1e-12));
}

}  // namespace
}  // namespace duatic::trajectory
