#pragma once

#include <duatic_geometry/kinematic_variable_3d.hpp>

#include <ostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

namespace duatic::geometry
{

template <typename ScalarT, KinematicOrderT OrderDepth,
          template <typename, KinematicOrderT> typename KinematicVariable3DTT = KinematicVariable3DT>
class KinematicState3D
{
public:
  using ScalarType = ScalarT;
  using Self = KinematicState3D<ScalarType, OrderDepth, KinematicVariable3DTT>;

  static constexpr KinematicOrderT KinematicOrderDepth = OrderDepth;

  template <KinematicOrderT Order>
  using KinematicVariable3DType = KinematicVariable3DTT<ScalarType, Order>;

  using PoseType = std::enable_if_t<(KinematicOrderDepth >= static_cast<KinematicOrderT>(KinematicOrder::Pose)),
                                     KinematicVariable3DType<static_cast<KinematicOrderT>(KinematicOrder::Pose)>>;
  using TwistType = std::enable_if_t<(KinematicOrderDepth >= static_cast<KinematicOrderT>(KinematicOrder::Twist)),
                                      KinematicVariable3DType<static_cast<KinematicOrderT>(KinematicOrder::Twist)>>;
  using AccelType = std::enable_if_t<(KinematicOrderDepth >= static_cast<KinematicOrderT>(KinematicOrder::Accel)),
                                      KinematicVariable3DType<static_cast<KinematicOrderT>(KinematicOrder::Accel)>>;
  using JerkType = std::enable_if_t<(KinematicOrderDepth >= static_cast<KinematicOrderT>(KinematicOrder::Jerk)),
                                     KinematicVariable3DType<static_cast<KinematicOrderT>(KinematicOrder::Jerk)>>;
  using SnapType = std::enable_if_t<(KinematicOrderDepth >= static_cast<KinematicOrderT>(KinematicOrder::Snap)),
                                     KinematicVariable3DType<static_cast<KinematicOrderT>(KinematicOrder::Snap)>>;

  inline constexpr KinematicState3D() = default;
  inline constexpr KinematicState3D(const Self& other) = default;
  inline constexpr KinematicState3D(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename... Args>
  inline constexpr KinematicState3D(Args&&... args)
    requires(sizeof...(Args) == OrderDepth + 1)
    : variables_(std::forward<Args>(args)...)
  {
  }

  template <KinematicOrderT Order>
  inline KinematicVariable3DType<Order>& variable()
  {
    return std::get<Order>(variables_);
  }
  template <KinematicOrderT Order>
  inline const KinematicVariable3DType<Order>& variable() const
  {
    return std::get<Order>(variables_);
  }

  inline auto& pose()
    requires(KinematicOrderDepth >= KinematicOrder::Pose)
  {
    return variable<KinematicOrder::Pose>();
  }
  inline const auto& pose() const
    requires(KinematicOrderDepth >= KinematicOrder::Pose)
  {
    return variable<KinematicOrder::Pose>();
  }

  inline auto& twist()
    requires(KinematicOrderDepth >= KinematicOrder::Twist)
  {
    return variable<KinematicOrder::Twist>();
  }
  inline const auto& twist() const
    requires(KinematicOrderDepth >= KinematicOrder::Twist)
  {
    return variable<KinematicOrder::Twist>();
  }

  inline auto& accel()
    requires(KinematicOrderDepth >= KinematicOrder::Accel)
  {
    return variable<KinematicOrder::Accel>();
  }
  inline const auto& accel() const
    requires(KinematicOrderDepth >= KinematicOrder::Accel)
  {
    return variable<KinematicOrder::Accel>();
  }

  inline auto& jerk()
    requires(KinematicOrderDepth >= KinematicOrder::Jerk)
  {
    return variable<KinematicOrder::Jerk>();
  }
  inline const auto& jerk() const
    requires(KinematicOrderDepth >= KinematicOrder::Jerk)
  {
    return variable<KinematicOrder::Jerk>();
  }

  inline auto& snap()
    requires(KinematicOrderDepth >= KinematicOrder::Snap)
  {
    return variable<KinematicOrder::Snap>();
  }
  inline const auto& snap() const
    requires(KinematicOrderDepth >= KinematicOrder::Snap)
  {
    return variable<KinematicOrder::Snap>();
  }

  template <KinematicOrderT... Orders>
  Self& setNeutral()
  {
    (variable<Orders>().setNeutral(), ...);
    return *this;
  }

  Self& setNeutral()
  {
    std::apply([](auto&... vars) { (vars.setNeutral(), ...); }, variables_);
    return *this;
  }

private:
  template <typename OrderSequence>
  struct VariablesTupleHelper;

  template <KinematicOrderT... Orders>
  struct VariablesTupleHelper<std::integer_sequence<KinematicOrderT, Orders...>>
  {
    using type = std::tuple<KinematicVariable3DType<Orders>...>;
    static_assert(is_kinematic_variable_3d_v<type>);
  };

  using VariablesTuple =
      typename VariablesTupleHelper<std::make_integer_sequence<KinematicOrderT, OrderDepth + 1>>::type;

  VariablesTuple variables_;
};

// Type Defs
template <typename ScalarT = double>
using TimedState3D = Timed<State3D<ScalarT>>;
template <typename ScalarT = double>
using StampedState3D = Stamped<State3D<ScalarT>>;

// Explicit Types
using State3Dd = State3D<double>;
using TimedState3Dd = TimedState3D<double>;
using StampedState3Dd = StampedState3D<double>;

// streaming
template <typename ScalarT, KinematicOrderT OrderDepth,
          template <typename, KinematicOrderT> typename KinematicVariable3DTT, KinematicOrderT... Orders>
inline void streamKinematicState3DVariables(std::ostream& os,
                                            const KinematicState3D<ScalarT, OrderDepth, KinematicVariable3DTT>& state,
                                            std::integer_sequence<KinematicOrderT, Orders...>)
{
  ((os << "  - Variable " << std::to_string(Orders) << ": " << state.template variable<Orders>() << std::endl), ...);
}

template <typename ScalarT, KinematicOrderT OrderDepth,
          template <typename, KinematicOrderT> typename KinematicVariable3DTT>
inline std::ostream& operator<<(std::ostream& os,
                                const KinematicState3D<ScalarT, OrderDepth, KinematicVariable3DTT>& state)
{
  os << "KinematicState3D:" << std::endl;
  streamKinematicState3DVariables(os, state, std::make_integer_sequence<KinematicOrderT, OrderDepth + 1>{});
  return os;
}

}  // namespace duatic::geometry
