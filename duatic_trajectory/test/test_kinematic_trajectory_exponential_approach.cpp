#include <gtest/gtest.h>

#include <duatic_trajectory/trajectory_eigen.hpp>

namespace duatic::trajectory
{
namespace
{

constexpr double convergence_horizon_seconds = 120.0;
constexpr double simulation_horizon_seconds = 90.0;
constexpr double simulation_samples_per_second = 1000.0;
constexpr double simulation_result_relative_tolerance = 1.01;  // 1%

geometry::Pose3Dd makePose(double x, double y, double z, const Eigen::Quaterniond& orientation)
{
  return geometry::Pose3Dd(Eigen::Vector3d(x, y, z), orientation);
}

/*
 * Builds a Trajectory::UpdateStateType from a pose, twist and (only if the trajectory under test
 * actually matches it, i.e. the C2/Accel-continuity variant) an initial acceleration -- so every
 * test body below is written once against a generic Trajectory and works unchanged for every
 * continuity order under test, each of which requires a differently-shaped initial state. Adding a
 * further continuity order later only means adding one more "else if constexpr" branch here.
 */
template <typename Trajectory>
typename Trajectory::UpdateStateType makeUpdateState(
    const rclcpp::Time& time, const geometry::Pose3Dd& pose, const geometry::Twist3Dd& twist,
    const geometry::Accel3Dd& accel = geometry::Accel3Dd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
{
  using DataType = typename Trajectory::UpdateStateType::DataType;
  if constexpr (DataType::kinematic_order_depth == geometry::KinematicOrder::Twist) {
    return typename Trajectory::UpdateStateType(time, pose, twist);
  } else {
    static_assert(DataType::kinematic_order_depth == geometry::KinematicOrder::Accel,
                  "makeUpdateState() doesn't know how to build this trajectory's initial state yet.");
    return typename Trajectory::UpdateStateType(time, pose, twist, accel);
  }
}

/*
 * Simulation helper (shared by the "stays within the configured limit" tests below): densely
 * samples evaluate() across the whole approach and returns the peak observed linear/angular speed.
 */
struct SampledSpeeds
{
  double max_linear_speed;
  double max_angular_speed;
};

template <typename Trajectory>
SampledSpeeds sample_max_speeds(const Trajectory& traj, const double horizon_s, const double samples_per_second)
{
  SampledSpeeds result{ 0.0, 0.0 };
  for (double t = 0.0; t <= horizon_s; t += 1.0 / samples_per_second) {
    const rclcpp::Time sample_time(static_cast<int64_t>(t * 1.0e9));
    const auto eval = traj.template evaluate<geometry::KinematicOrder::Twist>(sample_time);
    result.max_linear_speed = std::max(result.max_linear_speed, eval.twist().linear().norm());
    result.max_angular_speed = std::max(result.max_angular_speed, eval.twist().angular().norm());
  }
  return result;
}

/*
 * Shared behavioral contract of every KinematicTrajectoryExponentialApproach continuity order: run
 * exactly the same test bodies against each concrete trajectory-template instantiation listed in
 * TrajectoryTypes below, so C1, C2 and any future continuity order are all held to the same bar.
 */
template <typename TrajectoryT>
class KinematicTrajectoryExponentialApproachTest : public ::testing::Test
{
public:
  using Trajectory = TrajectoryT;

  static_assert(KinematicTrajectory<Trajectory>);
};

using TrajectoryTypes = ::testing::Types<ExponentialApproachPose3DC1d, ExponentialApproachPose3DC2d>;

class TrajectoryTypeNames
{
public:
  template <typename Trajectory>
  static std::string GetName(int index)
  {
    if constexpr (std::is_same_v<Trajectory, ExponentialApproachPose3DC1d>) {
      return "C1";
    } else if constexpr (std::is_same_v<Trajectory, ExponentialApproachPose3DC2d>) {
      return "C2";
    } else {
      return std::to_string(index);
    }
  }
};

TYPED_TEST_SUITE(KinematicTrajectoryExponentialApproachTest, TrajectoryTypes, TrajectoryTypeNames);

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, EvaluateAtUpdateTimeReproducesTheInitialPose)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval = traj.template evaluate<geometry::KinematicOrder::Pose>(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, PoseDepthConvergesToTheGoalOverTime)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(5, -3, 2, Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 0, 1))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.1, 0.1, 0.1));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval_far = traj.template evaluate<geometry::KinematicOrder::Pose>(
      rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  // goal.linear() is the exact zero vector here, and Eigen's isApprox() is a *relative*
  // comparison that never succeeds against an exact zero reference no matter how close the
  // actual value is - so compare the absolute difference instead.
  EXPECT_LT((eval_far.pose().linear() - goal.linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(goal.angular(), 1e-4));
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, TwistDepthReproducesTheInitialPoseAndTwist)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0.05, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval = traj.template evaluate<geometry::KinematicOrder::Twist>(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
  EXPECT_TRUE(eval.twist().linear().isApprox(v0.linear(), 1e-9));
  EXPECT_TRUE(eval.twist().angular().isApprox(v0.angular(), 1e-9));
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, TwistDepthConvergesToTheGoalWithVanishingVelocity)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(5, -3, 2, Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 0, 1))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.1, 0.1, 0.1));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval_far = traj.template evaluate<geometry::KinematicOrder::Twist>(
      rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  // goal.linear() is the exact zero vector here, and Eigen's isApprox() is a *relative*
  // comparison that never succeeds against an exact zero reference no matter how close the
  // actual value is - so compare the absolute difference instead.
  EXPECT_LT((eval_far.pose().linear() - goal.linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(goal.angular(), 1e-4));
  EXPECT_LT(eval_far.twist().linear().norm(), 1e-4);
  EXPECT_LT(eval_far.twist().angular().norm(), 1e-4);
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, OutParamAndByValueEvaluateAgree)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const rclcpp::Time query_time(3, 0);
  const auto by_value = traj.template evaluate<geometry::KinematicOrder::Twist>(query_time);

  typename Trajectory::template KinematicState<geometry::KinematicOrder::Twist> out_param;
  traj.template evaluate<geometry::KinematicOrder::Twist>(query_time, out_param);

  EXPECT_TRUE(out_param.pose().linear().isApprox(by_value.pose().linear(), 1e-12));
  EXPECT_TRUE(out_param.pose().angular().isApprox(by_value.pose().angular(), 1e-12));
  EXPECT_TRUE(out_param.twist().vector().isApprox(by_value.twist().vector(), 1e-12));
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, CalculateNeutralHoldsCurrentStateAndConverges)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 =
      makePose(2, -1, 0.5, Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0, 1, 0))));
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.2, 0.1, -0.1), Eigen::Vector3d(0.05, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);

  Trajectory traj;
  traj.calculate_neutral(in_state);

  // right at the update time, the trajectory must reproduce the given state exactly
  const auto eval_now = traj.template evaluate<geometry::KinematicOrder::Twist>(start_time);
  EXPECT_TRUE(eval_now.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval_now.pose().angular().isApprox(x0.angular(), 1e-9));
  EXPECT_TRUE(eval_now.twist().linear().isApprox(v0.linear(), 1e-9));
  EXPECT_TRUE(eval_now.twist().angular().isApprox(v0.angular(), 1e-9));

  // "neutral" means the goal is the current pose itself, so far in the future it must still be
  // sitting at x0 with vanishing velocity, rather than having moved anywhere else.
  const auto eval_far = traj.template evaluate<geometry::KinematicOrder::Twist>(
      rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - x0.linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(x0.angular(), 1e-4));
  EXPECT_LT(eval_far.twist().linear().norm(), 1e-4);
  EXPECT_LT(eval_far.twist().angular().norm(), 1e-4);
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, UpdateReplansContinuouslyFromOwnPredictedState)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd first_goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, first_goal);

  const rclcpp::Time query_time(2, 0);
  const auto pre_update_state = traj.template evaluate<geometry::KinematicOrder::Twist>(query_time);

  // update() must replan using the trajectory's own predicted state at query_time, so evaluating
  // right at query_time immediately afterwards must show no jump.
  const geometry::Pose3Dd second_goal = makePose(5, 5, 5, Eigen::Quaterniond::Identity());
  traj.update(query_time, second_goal);

  const auto post_update_state = traj.template evaluate<geometry::KinematicOrder::Twist>(query_time);
  EXPECT_LT((post_update_state.pose().linear() - pre_update_state.pose().linear()).norm(), 1e-9);
  EXPECT_TRUE(post_update_state.pose().angular().isApprox(pre_update_state.pose().angular(), 1e-9));
  EXPECT_LT((post_update_state.twist().linear() - pre_update_state.twist().linear()).norm(), 1e-9);
  EXPECT_TRUE(post_update_state.twist().angular().isApprox(pre_update_state.twist().angular(), 1e-9));

  // and it now heads toward the new goal
  const auto eval_far = traj.template evaluate<geometry::KinematicOrder::Pose>(
      rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - second_goal.linear()).norm(), 1e-3);
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, UpdateNeutralFreezesAtOwnPredictedPose)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const rclcpp::Time query_time(2, 0);
  const auto predicted_pose = traj.template evaluate<geometry::KinematicOrder::Pose>(query_time);

  traj.update_neutral(query_time);

  // far in the future the trajectory must have settled at its own predicted pose from query_time,
  // not at the original goal.
  const auto eval_far = traj.template evaluate<geometry::KinematicOrder::Pose>(
      rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - predicted_pose.pose().linear()).norm(), 1e-4);
  EXPECT_TRUE(eval_far.pose().angular().isApprox(predicted_pose.pose().angular(), 1e-4));
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, UpdateFromReplansUsingOtherTrajectoryNotOwnState)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
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
  const auto expected_state = source.template evaluate<geometry::KinematicOrder::Twist>(query_time);
  const auto actual_state = stale.template evaluate<geometry::KinematicOrder::Twist>(query_time);
  EXPECT_LT((actual_state.pose().linear() - expected_state.pose().linear()).norm(), 1e-9);
  EXPECT_TRUE(actual_state.pose().angular().isApprox(expected_state.pose().angular(), 1e-9));
  EXPECT_LT((actual_state.twist().linear() - expected_state.twist().linear()).norm(), 1e-9);
  EXPECT_TRUE(actual_state.twist().angular().isApprox(expected_state.twist().angular(), 1e-9));

  // and it now heads toward the new goal
  const auto eval_far = stale.template evaluate<geometry::KinematicOrder::Pose>(
      rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
  EXPECT_LT((eval_far.pose().linear() - second_goal.linear()).norm(), 1e-3);
}

/*
 * Simulation tests: densely sample evaluate() across the whole approach and verify the peak
 * observed linear/angular speed stays close to the configured velocity_limit_linear()/
 * velocity_limit_angular() settings. Per the "Approximate V-Limit" derivation documented above
 * each KinematicTrajectoryExponentialApproach continuity order, v_max is an upper *estimate* of
 * the peak speed, not an exact bound, so a modest tolerance above the configured limit is
 * allowed. Both trajectories start at rest so the approximation isn't skewed by an already-
 * excessive initial velocity.
 */
TYPED_TEST(KinematicTrajectoryExponentialApproachTest, LinearVelocityStaysWithinConfiguredLimit)
{
  using Trajectory = TypeParam;

  auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
  ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
  ASSERT_TRUE(settings->set_omega_limits(0.01, 10.0));

  const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(3, 0, 0, Eigen::Quaterniond::Identity());  // pure linear displacement

  Trajectory traj(settings);
  traj.calculate(in_state, goal);

  const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);

  EXPECT_LE(speeds.max_linear_speed, settings->velocity_limit_linear() * simulation_result_relative_tolerance);
  EXPECT_GT(speeds.max_linear_speed, 0.0);    // sanity: the trajectory must actually move
  EXPECT_LE(speeds.max_angular_speed, 1e-9);  // no orientation change was requested
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, AngularVelocityStaysWithinConfiguredLimit)
{
  using Trajectory = TypeParam;

  auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
  ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
  ASSERT_TRUE(settings->set_omega_limits(0.01, 10.0));

  const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal =
      makePose(0, 0, 0, Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d(0, 0, 1))));  // pure rotation

  Trajectory traj(settings);
  traj.calculate(in_state, goal);

  const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);

  EXPECT_LE(speeds.max_angular_speed, settings->velocity_limit_angular() * simulation_result_relative_tolerance);
  EXPECT_GT(speeds.max_angular_speed, 0.0);  // sanity: the trajectory must actually rotate
  EXPECT_LE(speeds.max_linear_speed, 1e-9);  // no positional change was requested
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, CombinedLinearAndAngularVelocityStayWithinConfiguredLimits)
{
  using Trajectory = TypeParam;

  auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
  ASSERT_TRUE(settings->set_velocity_limits(0.4, 0.8));
  ASSERT_TRUE(settings->set_omega_limits(0.01, 10.0));

  const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal =
      makePose(2, -1, 0.5, Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(0, 1, 0))));

  Trajectory traj(settings);
  traj.calculate(in_state, goal);

  const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);

  EXPECT_LE(speeds.max_linear_speed, settings->velocity_limit_linear() * simulation_result_relative_tolerance);
  EXPECT_LE(speeds.max_angular_speed, settings->velocity_limit_angular() * simulation_result_relative_tolerance);
}

/*
 * C2-specific: matching the initial acceleration too is exactly what distinguishes the C2
 * (Accel-continuity) variant from C1, so this checks it directly rather than via the shared
 * fixture above (C1 has no acceleration continuity guarantee to hold to the same bar).
 */
TEST(KinematicTrajectoryExponentialApproachC2, EvaluateAtUpdateTimeReproducesTheInitialAccel)
{
  using Trajectory = ExponentialApproachPose3DC2d;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));
  const geometry::Accel3Dd a0(Eigen::Vector3d(0.4, 0.1, -0.2), Eigen::Vector3d(0.02, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const geometry::StateAccel3Dd current_state(x0, v0, a0);
  const Trajectory::UpdateStateType in_state(start_time, current_state);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  Trajectory traj;
  traj.calculate(in_state, goal);

  const auto eval = traj.evaluate<geometry::KinematicOrder::Accel>(start_time);
  EXPECT_TRUE(eval.pose().linear().isApprox(x0.linear(), 1e-9));
  EXPECT_TRUE(eval.pose().angular().isApprox(x0.angular(), 1e-9));
  EXPECT_TRUE(eval.twist().linear().isApprox(v0.linear(), 1e-9));
  EXPECT_TRUE(eval.twist().angular().isApprox(v0.angular(), 1e-9));
  EXPECT_TRUE(eval.accel().linear().isApprox(a0.linear(), 1e-9));
  EXPECT_TRUE(eval.accel().angular().isApprox(a0.angular(), 1e-9));
}

}  // namespace
}  // namespace duatic::trajectory
