#include <gtest/gtest.h>

#include <duatic_trajectory/trajectory_eigen.hpp>

namespace duatic::trajectory
{
namespace
{

using Trajectory = ExponentialApproachPose3Dd;

static_assert(KinematicTrajectory<Trajectory>);

constexpr double convergence_horizon_seconds = 120.0;
constexpr double simulation_horizon_seconds = 90.0;
constexpr double simulation_samples_per_second = 1000.0;
constexpr double simulation_result_relative_tolerance = 1.01;  // 1%

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
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval = traj.evaluate<geometry::KinematicOrder::Pose>(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
}

TEST(KinematicTrajectoryExponentialApproach, PoseDepthConvergesToTheGoalOverTime)
{
  const geometry::Pose3Dd x0 = makePose(5, -3, 2, Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 0, 1))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.1, 0.1, 0.1));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval_far = traj.evaluate<geometry::KinematicOrder::Pose>(rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
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
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval = traj.evaluate<geometry::KinematicOrder::Twist>(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
  EXPECT_TRUE(eval.twist().linear().isApprox(v0.linear(), 1e-9));
  EXPECT_TRUE(eval.twist().angular().isApprox(v0.angular(), 1e-9));
}

TEST(KinematicTrajectoryExponentialApproach, TwistDepthConvergesToTheGoalWithVanishingVelocity)
{
  const geometry::Pose3Dd x0 = makePose(5, -3, 2, Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 0, 1))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.1, 0.1, 0.1));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval_far = traj.evaluate<geometry::KinematicOrder::Twist>(rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
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
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const rclcpp::Time query_time(3, 0);
  const auto by_value = traj.evaluate<geometry::KinematicOrder::Twist>(query_time);

  Trajectory::KinematicState<geometry::KinematicOrder::Twist> out_param;
  traj.evaluate<geometry::KinematicOrder::Twist>(query_time, out_param);

  EXPECT_TRUE(out_param.pose().linear().isApprox(by_value.pose().linear(), 1e-12));
  EXPECT_TRUE(out_param.pose().angular().isApprox(by_value.pose().angular(), 1e-12));
  EXPECT_TRUE(out_param.twist().vector().isApprox(by_value.twist().vector(), 1e-12));
}

TEST(KinematicTrajectoryExponentialApproach, CalculateNeutralHoldsCurrentStateAndConverges)
{
  const geometry::Pose3Dd x0 =
      makePose(2, -1, 0.5, Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0, 1, 0))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.2, 0.1, -0.1), Eigen::Vector3d(0.05, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);

  Trajectory traj;
  traj.calculate_neutral(in_state);

  // right at the update time, the trajectory must reproduce the given state exactly
  const auto eval_now = traj.evaluate<geometry::KinematicOrder::Twist>(start_time);
  EXPECT_TRUE(eval_now.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval_now.pose().angular().isApprox(x0.angular(), 1e-9));
  EXPECT_TRUE(eval_now.twist().linear().isApprox(v0.linear(), 1e-9));
  EXPECT_TRUE(eval_now.twist().angular().isApprox(v0.angular(), 1e-9));

  // "neutral" means the goal is the current pose itself, so far in the future it must still be
  // sitting at x0 with vanishing velocity, rather than having moved anywhere else.
  const auto eval_far = traj.evaluate<geometry::KinematicOrder::Twist>(rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - x0.linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(x0.angular(), 1e-4));
  EXPECT_LT(eval_far.twist().linear().norm(), 1e-4);
  EXPECT_LT(eval_far.twist().angular().norm(), 1e-4);
}

TEST(KinematicTrajectoryExponentialApproach, UpdateReplansContinuouslyFromOwnPredictedState)
{
  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd first_goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, first_goal);

  const rclcpp::Time query_time(2, 0);
  const auto pre_update_state = traj.evaluate<geometry::KinematicOrder::Twist>(query_time);

  // update() must replan using the trajectory's own predicted state at query_time, so evaluating
  // right at query_time immediately afterwards must show no jump.
  const geometry::Pose3Dd second_goal = makePose(5, 5, 5, Eigen::Quaterniond::Identity());
  traj.update(query_time, second_goal);

  const auto post_update_state = traj.evaluate<geometry::KinematicOrder::Twist>(query_time);
  EXPECT_LT((post_update_state.pose().linear() - pre_update_state.pose().linear()).norm(), 1e-9);
  EXPECT_TRUE(post_update_state.pose().angular().isApprox(pre_update_state.pose().angular(), 1e-9));
  EXPECT_LT((post_update_state.twist().linear() - pre_update_state.twist().linear()).norm(), 1e-9);
  EXPECT_TRUE(post_update_state.twist().angular().isApprox(pre_update_state.twist().angular(), 1e-9));

  // and it now heads toward the new goal
  const auto eval_far = traj.evaluate<geometry::KinematicOrder::Pose>(rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - second_goal.linear()).norm(), 1e-3);
}

TEST(KinematicTrajectoryExponentialApproach, UpdateNeutralFreezesAtOwnPredictedPose)
{
  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const rclcpp::Time query_time(2, 0);
  const auto predicted_pose = traj.evaluate<geometry::KinematicOrder::Pose>(query_time);

  traj.update_neutral(query_time);

  // far in the future the trajectory must have settled at its own predicted pose from query_time,
  // not at the original goal.
  const auto eval_far = traj.evaluate<geometry::KinematicOrder::Pose>(rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - predicted_pose.pose().linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(predicted_pose.pose().angular(), 1e-4));
}

TEST(KinematicTrajectoryExponentialApproach, UpdateFromReplansUsingOtherTrajectoryNotOwnState)
{
  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd first_goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory source;
  source.calculate(in_state, first_goal);

  // 'stale' simulates a triple-buffer slot that has never had calculate()/update() applied to it
  // (e.g. its default-constructed start_time_ is on a different rclcpp::Time clock type than
  // whatever timestamp update_from() is given) -- exactly the bug update_from() exists to avoid.
  Trajectory stale;

  const rclcpp::Time query_time(3, 0);
  const geometry::Pose3Dd second_goal = makePose(4, 4, 4, Eigen::Quaterniond::Identity());
  stale.update_from(source, query_time, second_goal);

  // right at query_time, 'stale' must continue smoothly from 'source's predicted state, not from
  // its own (never-initialized) internal state.
  const auto expected_state = source.evaluate<geometry::KinematicOrder::Twist>(query_time);
  const auto actual_state = stale.evaluate<geometry::KinematicOrder::Twist>(query_time);
  EXPECT_LT((actual_state.pose().linear() - expected_state.pose().linear()).norm(), 1e-9);
  EXPECT_TRUE(actual_state.pose().angular().isApprox(expected_state.pose().angular(), 1e-9));
  EXPECT_LT((actual_state.twist().linear() - expected_state.twist().linear()).norm(), 1e-9);
  EXPECT_TRUE(actual_state.twist().angular().isApprox(expected_state.twist().angular(), 1e-9));

  // and it now heads toward the new goal
  const auto eval_far = stale.evaluate<geometry::KinematicOrder::Pose>(rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - second_goal.linear()).norm(), 1e-3);
}

/*
 * Simulation tests: densely sample evaluate() across the whole approach and verify the peak
 * observed linear/angular speed stays close to the configured velocity_limit_linear()/
 * velocity_limit_angular() settings. Per the "Approximate V-Limit" derivation documented above
 * KinematicTrajectoryExponentialApproach, v_max is an upper *estimate* of the peak speed, not an
 * exact bound, so a modest tolerance above the configured limit is allowed. Both trajectories
 * start at rest so the approximation isn't skewed by an already-excessive initial velocity.
 */
struct SampledSpeeds
{
  double max_linear_speed;
  double max_angular_speed;
};

SampledSpeeds sample_max_speeds(const Trajectory& traj, const double horizon_s, const double samples_per_second)
{
  SampledSpeeds result{ 0.0, 0.0 };
  for (double t = 0.0; t <= horizon_s; t += 1.0 / samples_per_second) {
    const rclcpp::Time sample_time(static_cast<int64_t>(t * 1.0e9));
    const auto eval = traj.evaluate<geometry::KinematicOrder::Twist>(sample_time);
    result.max_linear_speed = std::max(result.max_linear_speed, eval.twist().linear().norm());
    result.max_angular_speed = std::max(result.max_angular_speed, eval.twist().angular().norm());
  }
  return result;
}

TEST(KinematicTrajectoryExponentialApproach, LinearVelocityStaysWithinConfiguredLimit)
{
  auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
  ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
  ASSERT_TRUE(settings->set_omega_limits(0.01, 10.0));

  const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(3, 0, 0, Eigen::Quaterniond::Identity());  // pure linear displacement

  Trajectory traj(settings);
  traj.calculate(in_state, goal);

  const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);

  EXPECT_LE(speeds.max_linear_speed, settings->velocity_limit_linear() * simulation_result_relative_tolerance);
  EXPECT_GT(speeds.max_linear_speed, 0.0);    // sanity: the trajectory must actually move
  EXPECT_LE(speeds.max_angular_speed, 1e-9);  // no orientation change was requested
}

TEST(KinematicTrajectoryExponentialApproach, AngularVelocityStaysWithinConfiguredLimit)
{
  auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
  ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
  ASSERT_TRUE(settings->set_omega_limits(0.01, 10.0));

  const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal =
      makePose(0, 0, 0, Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d(0, 0, 1))));  // pure rotation

  Trajectory traj(settings);
  traj.calculate(in_state, goal);

  const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);

  EXPECT_LE(speeds.max_angular_speed, settings->velocity_limit_angular() * simulation_result_relative_tolerance);
  EXPECT_GT(speeds.max_angular_speed, 0.0);  // sanity: the trajectory must actually rotate
  EXPECT_LE(speeds.max_linear_speed, 1e-9);  // no positional change was requested
}

TEST(KinematicTrajectoryExponentialApproach, CombinedLinearAndAngularVelocityStayWithinConfiguredLimits)
{
  auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
  ASSERT_TRUE(settings->set_velocity_limits(0.4, 0.8));
  ASSERT_TRUE(settings->set_omega_limits(0.01, 10.0));

  const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
  const geometry::StateTwist3Dd current_state(x0, v0);

  const rclcpp::Time start_time(0, 0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal =
      makePose(2, -1, 0.5, Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(0, 1, 0))));

  Trajectory traj(settings);
  traj.calculate(in_state, goal);

  const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);

  EXPECT_LE(speeds.max_linear_speed, settings->velocity_limit_linear() * simulation_result_relative_tolerance);
  EXPECT_LE(speeds.max_angular_speed, settings->velocity_limit_angular() * simulation_result_relative_tolerance);
}

}  // namespace
}  // namespace duatic::trajectory
