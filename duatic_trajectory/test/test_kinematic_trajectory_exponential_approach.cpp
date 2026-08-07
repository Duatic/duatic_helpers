#include <gtest/gtest.h>

#include <array>
#include <numbers>
#include <random>

#include <duatic_trajectory/trajectory_eigen.hpp>

namespace duatic::trajectory
{
namespace
{

constexpr double convergence_horizon_seconds = 120.0;
constexpr uint number_of_simulations = 1000;
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
typename Trajectory::UpdateStateType
makeUpdateState(const rclcpp::Time& time, const geometry::Pose3Dd& pose, const geometry::Twist3Dd& twist,
                const geometry::Accel3Dd& accel = geometry::Accel3Dd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
{
  using DataType = typename Trajectory::UpdateStateType::DataType;
  if constexpr (DataType::kinematic_order_depth == geometry::KinematicOrder::Twist) {
    return typename Trajectory::UpdateStateType(time, pose, twist);
  } else {
    static_assert(DataType::kinematic_order_depth == geometry::KinematicOrder::Accel, "makeUpdateState() doesn't know "
                                                                                      "how to build this trajectory's "
                                                                                      "initial state yet.");
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
 * Verifies the "Approximate V-Limit" invariant against an already-calculate()'d trajectory: the
 * peak sampled speed must stay close to max(configured limit, initial speed) -- whichever is
 * larger acts as the floor. v_max is only a meaningful ceiling once the initial speed already
 * respects it; below that, x'(0) itself is an unavoidable floor regardless of omega (see the
 * InitialVelocityExceedingLimitDoesNotDivergeWithNonzeroAccel regression this generalizes). Per
 * the proofs document, v_max is an upper *estimate*, not an exact bound, so a modest relative
 * tolerance is allowed above whichever floor applies. traj.settings supplies the configured
 * velocity_limit_linear()/angular(); the initial speeds aren't recoverable from traj after the
 * fact, so the caller passes them in directly.
 */
template <typename Trajectory>
void verifyMaxVelocityInvariant(const Trajectory& traj, double initial_linear_speed, double initial_angular_speed,
                                double horizon_s = simulation_horizon_seconds,
                                double samples_per_second = simulation_samples_per_second,
                                double relative_tolerance = simulation_result_relative_tolerance)
{
  const SampledSpeeds speeds = sample_max_speeds(traj, horizon_s, samples_per_second);
  const double linear_floor = std::max(traj.settings->velocity_limit_linear(), initial_linear_speed);
  const double angular_floor = std::max(traj.settings->velocity_limit_angular(), initial_angular_speed);

  EXPECT_LE(speeds.max_linear_speed, linear_floor * relative_tolerance)
      << "max_linear_speed=" << speeds.max_linear_speed
      << " exceeded max(v_max_linear=" << traj.settings->velocity_limit_linear()
      << ", |v0_linear|=" << initial_linear_speed << ")";
  EXPECT_LE(speeds.max_angular_speed, angular_floor * relative_tolerance)
      << "max_angular_speed=" << speeds.max_angular_speed
      << " exceeded max(v_max_angular=" << traj.settings->velocity_limit_angular()
      << ", |v0_angular|=" << initial_angular_speed << ")";
}

/*
 * Simulation helper (shared by the C1-only "stays within the configured acceleration limit" tests
 * below): densely samples evaluate<Accel>() across the whole approach and returns the peak observed
 * linear/angular acceleration magnitude.
 */
struct SampledAccels
{
  double max_linear_accel;
  double max_angular_accel;
};

template <typename Trajectory>
SampledAccels sample_max_accels(const Trajectory& traj, const double horizon_s, const double samples_per_second)
{
  SampledAccels result{ 0.0, 0.0 };
  for (double t = 0.0; t <= horizon_s; t += 1.0 / samples_per_second) {
    const rclcpp::Time sample_time(static_cast<int64_t>(t * 1.0e9));
    const auto eval = traj.template evaluate<geometry::KinematicOrder::Accel>(sample_time);
    result.max_linear_accel = std::max(result.max_linear_accel, eval.accel().linear().norm());
    result.max_angular_accel = std::max(result.max_angular_accel, eval.accel().angular().norm());
  }
  return result;
}

/*
 * Verifies the "Approximate A-Limit" invariant (see determine_acc_omega() in
 * kinematic_trajectory_exponential_approach_C1.hpp) against an already-calculate()'d trajectory:
 * the peak sampled acceleration must stay close to max(configured limit, the trajectory's own
 * initial acceleration) -- mirroring verifyMaxVelocityInvariant()'s v0 floor above. Unlike v0,
 * though, the initial acceleration isn't a raw external input C1 has to tolerate (C1 has no
 * acceleration continuity to match) -- it's a consequence of the omega determine_acc_omega() itself
 * chose, so the floor is measured directly from the trajectory rather than passed in by the caller.
 */
template <typename Trajectory>
void verifyMaxAccelerationInvariant(const Trajectory& traj, const rclcpp::Time& start_time,
                                    double horizon_s = simulation_horizon_seconds,
                                    double samples_per_second = simulation_samples_per_second,
                                    double relative_tolerance = simulation_result_relative_tolerance)
{
  const auto initial = traj.template evaluate<geometry::KinematicOrder::Accel>(start_time);
  const double initial_linear_accel = initial.accel().linear().norm();
  const double initial_angular_accel = initial.accel().angular().norm();

  const SampledAccels accels = sample_max_accels(traj, horizon_s, samples_per_second);
  const double linear_floor = std::max(traj.settings->acceleration_limit_linear(), initial_linear_accel);
  const double angular_floor = std::max(traj.settings->acceleration_limit_angular(), initial_angular_accel);

  EXPECT_LE(accels.max_linear_accel, linear_floor * relative_tolerance)
      << "max_linear_accel=" << accels.max_linear_accel
      << " exceeded max(a_max_linear=" << traj.settings->acceleration_limit_linear()
      << ", |a0_linear|=" << initial_linear_accel << ")";
  EXPECT_LE(accels.max_angular_accel, angular_floor * relative_tolerance)
      << "max_angular_accel=" << accels.max_angular_accel
      << " exceeded max(a_max_angular=" << traj.settings->acceleration_limit_angular()
      << ", |a0_angular|=" << initial_angular_accel << ")";
}

/*
 * Uniform helpers for the randomized stress test below. A fixed seed keeps failures reproducible
 * instead of flaky.
 */
Eigen::Vector3d randomVector3d(std::mt19937& rng, double range)
{
  std::uniform_real_distribution<double> dist(-range, range);
  return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

Eigen::Quaterniond randomQuaternion(std::mt19937& rng)
{
  std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * std::numbers::pi);
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle_dist(rng), randomVector3d(rng, 1.0).normalized()));
}

/*
 * Explicit, adequately-fast settings profiles for the convergence-behavior tests below. These
 * previously relied on KinematicTrajectorySettingsDefault's own defaults, which broke once that
 * default's v_max_lin_ dropped low enough that it no longer comfortably exceeds these tests'
 * hardcoded initial speeds (up to ~1.7 m/s): once v0 drastically exceeds v_max, determine_lin_omega()
 * falls back to omega_min, whose decay time constant can dwarf any fixed test horizon -- the
 * trajectory then coasts near-linearly instead of genuinely converging. Constructing settings
 * explicitly, and running each test against several distinct profiles rather than one hardcoded
 * value, decouples these tests from any particular default and exercises a spread of
 * velocity/acceleration/omega configurations instead.
 */
struct ConvergenceSettingsProfile
{
  const char* name;
  double v_max_lin, v_max_ang;
  double a_max_lin, a_max_ang;
  double omega_min, omega_max;
};

// omega_min/omega_max here must respect KinematicTrajectorySettingsExponentialApproachDefault's
// own omega_min_upper_bound/omega_max_lower_bound (omega_min <= 1e-6, omega_max >= 1e3); this
// doesn't hurt convergence -- these are just the allowed *range*, and each profile's v_max is
// generous enough (relative to these tests' hardcoded initial speeds) that the actual omega picked
// comes from the smooth formula, not the omega_min floor, regardless of how tiny that floor is.
constexpr std::array<ConvergenceSettingsProfile, 3> convergence_settings_profiles{ {
    { "moderate", 1.0, 2.0 * std::numbers::pi, 1.0, 2.0 * std::numbers::pi, 1e-6, 1e3 },
    { "brisk", 2.5, 5.0, 2.5, 5.0, 1e-7, 2e3 },
    { "generous", 6.0, 12.0, 6.0, 12.0, 1e-8, 5e3 },
} };

template <typename Trajectory>
std::shared_ptr<typename Trajectory::KinematicTrajectorySettingsType>
makeConvergenceSettings(const ConvergenceSettingsProfile& profile)
{
  auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
  EXPECT_TRUE(settings->set_velocity_limits(profile.v_max_lin, profile.v_max_ang));
  EXPECT_TRUE(settings->set_acceleration_limits(profile.a_max_lin, profile.a_max_ang));
  EXPECT_TRUE(settings->set_omega_limits(profile.omega_min, profile.omega_max));
  return settings;
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

  for (const auto& profile : convergence_settings_profiles) {
    SCOPED_TRACE(::testing::Message() << "settings=" << profile.name);

    Trajectory traj(makeConvergenceSettings<Trajectory>(profile));
    traj.calculate(in_state, goal);

    const auto eval_far = traj.template evaluate<geometry::KinematicOrder::Pose>(
        rclcpp::Time(static_cast<int32_t>(convergence_horizon_seconds), 0));
    // goal.linear() is the exact zero vector here, and Eigen's isApprox() is a *relative*
    // comparison that never succeeds against an exact zero reference no matter how close the
    // actual value is - so compare the absolute difference instead.
    EXPECT_LT((eval_far.pose().linear() - goal.linear()).norm(), 1e-4);
    EXPECT_TRUE(eval_far.pose().angular().isApprox(goal.angular(), 1e-4));
  }
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

  for (const auto& profile : convergence_settings_profiles) {
    SCOPED_TRACE(::testing::Message() << "settings=" << profile.name);

    Trajectory traj(makeConvergenceSettings<Trajectory>(profile));
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

  for (const auto& profile : convergence_settings_profiles) {
    SCOPED_TRACE(::testing::Message() << "settings=" << profile.name);

    Trajectory traj(makeConvergenceSettings<Trajectory>(profile));
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
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, UpdateNeutralFreezesAtOwnPredictedPose)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  for (const auto& profile : convergence_settings_profiles) {
    SCOPED_TRACE(::testing::Message() << "settings=" << profile.name);

    Trajectory traj(makeConvergenceSettings<Trajectory>(profile));
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
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, UpdateFromReplansUsingOtherTrajectoryNotOwnState)
{
  using Trajectory = TypeParam;

  const geometry::Pose3Dd x0 = makePose(1, 2, 3, Eigen::Quaterniond::Identity());
  const geometry::Twist3Dd v0(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0, 0, 0));

  const rclcpp::Time start_time(0, 0);
  const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
  const geometry::Pose3Dd first_goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

  for (const auto& profile : convergence_settings_profiles) {
    SCOPED_TRACE(::testing::Message() << "settings=" << profile.name);

    Trajectory source(makeConvergenceSettings<Trajectory>(profile));
    source.calculate(in_state, first_goal);

    // 'stale' simulates a triple-buffer slot that has never had calculate()/update() applied to it
    // (e.g. its default-constructed start_time_ is on a different rclcpp::Time clock type than
    // whatever timestamp update_from() is given) -- exactly the bug update_from() exists to avoid.
    Trajectory stale(makeConvergenceSettings<Trajectory>(profile));

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
}

/*
 * Simulation tests: densely sample evaluate() across the whole approach and verify the
 * "Approximate V-Limit" invariant (see verifyMaxVelocityInvariant() above and the proofs
 * document) holds -- both for a fixed set of hand-picked examples (including the v0-already-
 * exceeds-the-limit regression that motivated generalizing this check into its own function) and,
 * as a broader stress test, across a large number of pseudo-random trajectories.
 */
TYPED_TEST(KinematicTrajectoryExponentialApproachTest, KnownExamplesRespectVelocityLimit)
{
  using Trajectory = TypeParam;
  const rclcpp::Time start_time(0, 0);

  {
    SCOPED_TRACE("pure linear displacement from rest");
    auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal = makePose(3, 0, 0, Eigen::Quaterniond::Identity());

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxVelocityInvariant(traj, /*initial_linear_speed=*/0.0, /*initial_angular_speed=*/0.0);
    const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);
    EXPECT_GT(speeds.max_linear_speed, 0.0);    // sanity: the trajectory must actually move
    EXPECT_LE(speeds.max_angular_speed, 1e-9);  // no orientation change was requested
  }

  {
    SCOPED_TRACE("pure rotation from rest");
    auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal =
        makePose(0, 0, 0, Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d(0, 0, 1))));

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxVelocityInvariant(traj, /*initial_linear_speed=*/0.0, /*initial_angular_speed=*/0.0);
    const SampledSpeeds speeds = sample_max_speeds(traj, simulation_horizon_seconds, simulation_samples_per_second);
    EXPECT_GT(speeds.max_angular_speed, 0.0);  // sanity: the trajectory must actually rotate
    EXPECT_LE(speeds.max_linear_speed, 1e-9);  // no positional change was requested
  }

  {
    SCOPED_TRACE("combined linear and angular displacement from rest");
    auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(0.4, 0.8));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal =
        makePose(2, -1, 0.5, Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(0, 1, 0))));

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxVelocityInvariant(traj, /*initial_linear_speed=*/0.0, /*initial_angular_speed=*/0.0);
  }

  // Regression case: initial velocity already exceeds the configured limit, with a nonzero initial
  // acceleration -- see InitialVelocityExceedingLimitDoesNotDivergeWithNonzeroAccel's original bug
  // report. makeUpdateState() simply drops the acceleration for C1 (no matching continuity order),
  // so this exercises the analogous "v0 already too fast" case for both trajectory types uniformly.
  const double v0_mag = 4.231045752802854, a0_mag = 1.020853192875898, a_offset = 0.2653719173757657;
  for (const double v_max : { 0.5, 1.0, 2.0 }) {  // all well below v0_mag: the pathological regime
    SCOPED_TRACE(::testing::Message() << "initial velocity exceeding the limit, v_max=" << v_max);
    auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(v_max, 1000.0));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(a_offset, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d(v0_mag, 0, 0), Eigen::Vector3d::Zero());
    const geometry::Accel3Dd a0(Eigen::Vector3d(a0_mag, 0, 0), Eigen::Vector3d::Zero());
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0, a0);
    const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    // this specific pathological case needs a much longer horizon: a too-small omega barely
    // decays, so the (now fixed) divergence this guards against would only show up after a long
    // time -- the default horizon isn't long enough to have caught it.
    verifyMaxVelocityInvariant(traj, v0_mag, 0.0, /*horizon_s=*/200.0, /*samples_per_second=*/2000.0);
  }
}

TYPED_TEST(KinematicTrajectoryExponentialApproachTest, RandomTrajectoriesRespectVelocityLimit)
{
  using Trajectory = TypeParam;

  std::mt19937 rng(1337);  // fixed seed: failures must be reproducible, not flaky
  std::uniform_real_distribution<double> v_max_dist(0.1, 5.0);
  // omega_min/omega_max must respect the settings' own omega_min_upper_bound (1e-6) /
  // omega_max_lower_bound (1e3); omega_min is drawn from a tiny sliver below that upper bound, and
  // omega_max_dist's span dwarfs it, so the sum below still lands safely above omega_max_lower_bound.
  std::uniform_real_distribution<double> omega_min_dist(1e-9, 1e-6);
  std::uniform_real_distribution<double> omega_max_dist(1e3, 5e3);

  // a coarser horizon/sample rate than the hand-picked examples above, to keep number_of_simulations
  // trajectories running in reasonable time while still covering a broad parameter space, including
  // (since v0 is drawn from a wider range than v_max) the "v0 already exceeds the limit" regime.
  constexpr double random_horizon_s = 60.0;
  constexpr double random_samples_per_second = 50.0;

  for (unsigned int trial = 0; trial < number_of_simulations; ++trial) {
    SCOPED_TRACE(::testing::Message() << "trial=" << trial);

    auto settings = std::make_shared<typename Trajectory::KinematicTrajectorySettingsType>();
    const double omega_min = omega_min_dist(rng);
    ASSERT_TRUE(settings->set_velocity_limits(v_max_dist(rng), v_max_dist(rng)));
    ASSERT_TRUE(settings->set_omega_limits(omega_min, omega_min + omega_max_dist(rng)));

    const geometry::Pose3Dd x0(randomVector3d(rng, 5.0), randomQuaternion(rng));
    const geometry::Twist3Dd v0(randomVector3d(rng, 10.0), randomVector3d(rng, 10.0));
    const geometry::Accel3Dd a0(randomVector3d(rng, 5.0), randomVector3d(rng, 5.0));
    const geometry::Pose3Dd goal(randomVector3d(rng, 5.0), randomQuaternion(rng));

    const rclcpp::Time start_time(0, 0);
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0, a0);

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxVelocityInvariant(traj, v0.linear().norm(), v0.angular().norm(), random_horizon_s,
                               random_samples_per_second);
  }
}

/*
 * C1-specific: determine_acc_omega()'s acceleration-limit merge (see
 * kinematic_trajectory_exponential_approach_C1.hpp) only exists on the C1 variant so far -- C2
 * doesn't fold an acceleration limit into its own omega selection yet -- so these mirror
 * KnownExamplesRespectVelocityLimit/RandomTrajectoriesRespectVelocityLimit above directly against
 * ExponentialApproachPose3DC1d rather than via the shared TYPED_TEST fixture.
 */
TEST(KinematicTrajectoryExponentialApproachC1, KnownExamplesRespectAccelerationLimit)
{
  using Trajectory = ExponentialApproachPose3DC1d;
  const rclcpp::Time start_time(0, 0);

  {
    SCOPED_TRACE("pure linear displacement from rest");
    auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
    ASSERT_TRUE(settings->set_acceleration_limits(0.2, 0.4));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal = makePose(3, 0, 0, Eigen::Quaterniond::Identity());

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxAccelerationInvariant(traj, start_time);
    const SampledAccels accels = sample_max_accels(traj, simulation_horizon_seconds, simulation_samples_per_second);
    EXPECT_GT(accels.max_linear_accel, 0.0);    // sanity: the trajectory must actually accelerate
    EXPECT_LE(accels.max_angular_accel, 1e-9);  // no orientation change was requested
  }

  {
    SCOPED_TRACE("pure rotation from rest");
    auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(0.5, 1.0));
    ASSERT_TRUE(settings->set_acceleration_limits(0.2, 0.4));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal =
        makePose(0, 0, 0, Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d(0, 0, 1))));

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxAccelerationInvariant(traj, start_time);
    const SampledAccels accels = sample_max_accels(traj, simulation_horizon_seconds, simulation_samples_per_second);
    EXPECT_GT(accels.max_angular_accel, 0.0);  // sanity: the trajectory must actually angularly accelerate
    EXPECT_LE(accels.max_linear_accel, 1e-9);  // no positional change was requested
  }

  {
    SCOPED_TRACE("combined linear and angular displacement from rest");
    auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(0.4, 0.8));
    ASSERT_TRUE(settings->set_acceleration_limits(0.15, 0.3));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());  // start at rest
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal =
        makePose(2, -1, 0.5, Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(0, 1, 0))));

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxAccelerationInvariant(traj, start_time);
  }

  // Large-offset case: exercises verifyMaxAccelerationInvariant()'s floor (which measures the
  // trajectory's actual resulting initial acceleration) against a big offset and a small a_max.
  // Note: this used to demonstrate the structurally-infeasible regime from determine_acc_omega()'s
  // comment (a large omega_min forcing a t=0 jump beyond a_max) directly via a raised omega_min, but
  // omega_min is now capped at omega_min_upper_bound (1e-6) by set_omega_limits() itself, and at that
  // scale the natural solve is comfortably feasible again for any offset/a_max combination sane
  // enough to write down here -- so this no longer reaches that particular regime.
  for (const double a_max : { 0.05, 0.1, 0.5 }) {
    SCOPED_TRACE(::testing::Message() << "large offset, a_max=" << a_max);
    auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
    ASSERT_TRUE(settings->set_velocity_limits(1000.0, 1000.0));
    ASSERT_TRUE(settings->set_acceleration_limits(a_max, 1000.0));
    ASSERT_TRUE(settings->set_omega_limits(1e-6, 1e3));

    const geometry::Pose3Dd x0 = makePose(50.0, 0, 0, Eigen::Quaterniond::Identity());
    const geometry::Twist3Dd v0(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);
    const geometry::Pose3Dd goal = makePose(0, 0, 0, Eigen::Quaterniond::Identity());

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxAccelerationInvariant(traj, start_time, /*horizon_s=*/200.0, /*samples_per_second=*/2000.0);
  }
}

TEST(KinematicTrajectoryExponentialApproachC1, RandomTrajectoriesRespectAccelerationLimit)
{
  using Trajectory = ExponentialApproachPose3DC1d;

  std::mt19937 rng(1337);  // fixed seed: failures must be reproducible, not flaky
  std::uniform_real_distribution<double> v_max_dist(0.1, 5.0);
  std::uniform_real_distribution<double> a_max_dist(0.05, 5.0);
  // omega_min/omega_max must respect the settings' own omega_min_upper_bound (1e-6) /
  // omega_max_lower_bound (1e3); see RandomTrajectoriesRespectVelocityLimit's identical comment.
  std::uniform_real_distribution<double> omega_min_dist(1e-9, 1e-6);
  std::uniform_real_distribution<double> omega_max_dist(1e3, 5e3);

  constexpr double random_horizon_s = 60.0;
  constexpr double random_samples_per_second = 50.0;

  for (unsigned int trial = 0; trial < number_of_simulations; ++trial) {
    SCOPED_TRACE(::testing::Message() << "trial=" << trial);

    auto settings = std::make_shared<Trajectory::KinematicTrajectorySettingsType>();
    const double omega_min = omega_min_dist(rng);
    ASSERT_TRUE(settings->set_velocity_limits(v_max_dist(rng), v_max_dist(rng)));
    ASSERT_TRUE(settings->set_acceleration_limits(a_max_dist(rng), a_max_dist(rng)));
    ASSERT_TRUE(settings->set_omega_limits(omega_min, omega_min + omega_max_dist(rng)));

    const geometry::Pose3Dd x0(randomVector3d(rng, 5.0), randomQuaternion(rng));
    const geometry::Twist3Dd v0(randomVector3d(rng, 10.0), randomVector3d(rng, 10.0));
    const geometry::Pose3Dd goal(randomVector3d(rng, 5.0), randomQuaternion(rng));

    const rclcpp::Time start_time(0, 0);
    const auto in_state = makeUpdateState<Trajectory>(start_time, x0, v0);

    Trajectory traj(settings);
    traj.calculate(in_state, goal);

    verifyMaxAccelerationInvariant(traj, start_time, random_horizon_s, random_samples_per_second);
  }
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

/*
 * Settings validation: the setters below are documented to return false (and leave all state
 * unchanged) for invalid input, and true (having applied the new values) for valid input. These
 * exercise both sides of that contract directly against the settings types, rather than indirectly
 * through a trajectory.
 */
TEST(KinematicTrajectorySettingsDefaultTest, SetVelocityLimitsAcceptsValidRejectsInvalid)
{
  trajectory::KinematicTrajectorySettingsDefault<double> settings;

  // valid: any non-negative pair (including zero) is accepted and actually applied
  EXPECT_TRUE(settings.set_velocity_limits(0.0, 0.0));
  EXPECT_TRUE(settings.set_velocity_limits(1.5, 2.5));
  EXPECT_DOUBLE_EQ(settings.velocity_limit_linear(), 1.5);
  EXPECT_DOUBLE_EQ(settings.velocity_limit_angular(), 2.5);

  // invalid: any negative component is rejected, leaving the previous (valid) values untouched
  EXPECT_FALSE(settings.set_velocity_limits(-1.0, 2.5));
  EXPECT_FALSE(settings.set_velocity_limits(1.5, -2.5));
  EXPECT_FALSE(settings.set_velocity_limits(-1.0, -2.5));
  EXPECT_DOUBLE_EQ(settings.velocity_limit_linear(), 1.5);
  EXPECT_DOUBLE_EQ(settings.velocity_limit_angular(), 2.5);
}

TEST(KinematicTrajectorySettingsDefaultTest, SetAccelerationLimitsAcceptsValidRejectsInvalid)
{
  trajectory::KinematicTrajectorySettingsDefault<double> settings;

  // valid: any non-negative pair (including zero) is accepted and actually applied
  EXPECT_TRUE(settings.set_acceleration_limits(0.0, 0.0));
  EXPECT_TRUE(settings.set_acceleration_limits(0.8, 1.2));
  EXPECT_DOUBLE_EQ(settings.acceleration_limit_linear(), 0.8);
  EXPECT_DOUBLE_EQ(settings.acceleration_limit_angular(), 1.2);

  // invalid: any negative component is rejected, leaving the previous (valid) values untouched
  EXPECT_FALSE(settings.set_acceleration_limits(-0.1, 1.2));
  EXPECT_FALSE(settings.set_acceleration_limits(0.8, -1.2));
  EXPECT_FALSE(settings.set_acceleration_limits(-0.1, -1.2));
  EXPECT_DOUBLE_EQ(settings.acceleration_limit_linear(), 0.8);
  EXPECT_DOUBLE_EQ(settings.acceleration_limit_angular(), 1.2);
}

TEST(KinematicTrajectorySettingsExponentialApproachDefaultTest, SetOmegaLimitsAcceptsValidRejectsInvalid)
{
  using Settings = trajectory::KinematicTrajectorySettingsExponentialApproachDefault<double>;
  Settings settings;
  constexpr double upper = Settings::omega_min_upper_bound;  // omega_min must fall in (0, upper]
  constexpr double lower = Settings::omega_max_lower_bound;  // omega_max must be >= lower (and >= omega_min)

  // valid: omega_min in (0, upper], omega_max >= max(omega_min, lower)
  EXPECT_TRUE(settings.set_omega_limits(upper, lower));
  EXPECT_DOUBLE_EQ(settings.omega_min(), upper);
  EXPECT_DOUBLE_EQ(settings.omega_max(), lower);
  EXPECT_TRUE(settings.set_omega_limits(upper / 10.0, lower * 5.0));
  EXPECT_DOUBLE_EQ(settings.omega_min(), upper / 10.0);
  EXPECT_DOUBLE_EQ(settings.omega_max(), lower * 5.0);

  // invalid: omega_min <= 0
  EXPECT_FALSE(settings.set_omega_limits(0.0, lower));
  EXPECT_FALSE(settings.set_omega_limits(-upper, lower));
  // invalid: omega_min above the upper bound
  EXPECT_FALSE(settings.set_omega_limits(upper * 10.0, lower));
  // invalid: omega_max below omega_min
  EXPECT_FALSE(settings.set_omega_limits(upper, upper / 2.0));
  // invalid: omega_max below the lower bound, even though omega_max >= omega_min
  EXPECT_FALSE(settings.set_omega_limits(upper / 10.0, lower / 10.0));

  // none of the invalid calls above should have changed the settings from their last valid state
  EXPECT_DOUBLE_EQ(settings.omega_min(), upper / 10.0);
  EXPECT_DOUBLE_EQ(settings.omega_max(), lower * 5.0);
}

}  // namespace
}  // namespace duatic::trajectory
