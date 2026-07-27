#pragma once

#include <duatic_geometry/kinematic_variable.hpp>

#include <ostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

namespace duatic::geometry
{

template <typename ScalarT, KinematicOrder OrderDepth,
          template <typename, KinematicOrder> typename KinematicVariable3DTT = KinematicVariable3DT>
class KinematicState
{
public:
  using ScalarType = ScalarT;
  using Self = KinematicState<ScalarType, OrderDepth, KinematicVariable3DTT>;

  static constexpr KinematicOrder KinematicOrderDepth = OrderDepth;

  template <KinematicOrder Order>
  using KinematicVariable3DType = KinematicVariable3DTT<ScalarType, Order>;

  inline constexpr KinematicState() = default;
  inline constexpr KinematicState(const Self& other) = default;
  inline constexpr KinematicState(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename... Args>
  inline constexpr KinematicState(Args&&... args)
    requires(sizeof...(Args) == to_number(OrderDepth) + 1)
    : variables_(std::forward<Args>(args)...)
  {
  }

  template <KinematicOrder Order>
  inline KinematicVariable3DType<Order>& variable()
  {
    return std::get<to_number(Order)>(variables_);
  }
  template <KinematicOrder Order>
  inline const KinematicVariable3DType<Order>& variable() const
  {
    return std::get<to_number(Order)>(variables_);
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

  template <KinematicOrder... Orders>
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
  using KinematicOrderT = std::underlying_type_t<KinematicOrder>;

  template <typename OrderSequence>
  struct VariablesTupleHelper;

  template <KinematicOrderT... Orders>
  struct VariablesTupleHelper<std::integer_sequence<KinematicOrderT, Orders...>>
  {
    using type = std::tuple<KinematicVariable3DType<static_cast<KinematicOrder>(Orders)>...>;
    static_assert((is_kinematic_variable_v<KinematicVariable3DType<static_cast<KinematicOrder>(Orders)>> && ...));
  };

  using VariablesTuple =
      typename VariablesTupleHelper<std::make_integer_sequence<KinematicOrderT, to_number(OrderDepth) + 1>>::type;

  VariablesTuple variables_;
};

// streaming
template <typename ScalarT, KinematicOrder OrderDepth,
          template <typename, KinematicOrder> typename KinematicVariable3DTT,
          std::underlying_type_t<KinematicOrder>... Orders>
inline void streamKinematicStateVariables(std::ostream& os,
                                          const KinematicState<ScalarT, OrderDepth, KinematicVariable3DTT>& state,
                                          std::integer_sequence<std::underlying_type_t<KinematicOrder>, Orders...>)
{
  ((os << "  - Variable " << std::to_string(Orders) << ": "
       << state.template variable<static_cast<KinematicOrder>(Orders)>() << std::endl),
   ...);
}

template <typename ScalarT, KinematicOrder OrderDepth,
          template <typename, KinematicOrder> typename KinematicVariable3DTT>
inline std::ostream& operator<<(std::ostream& os,
                                const KinematicState<ScalarT, OrderDepth, KinematicVariable3DTT>& state)
{
  os << "KinematicState:" << std::endl;
  using KinematicOrderT = std::underlying_type_t<KinematicOrder>;
  streamKinematicStateVariables(os, state, std::make_integer_sequence<KinematicOrderT, to_number(OrderDepth) + 1>{});
  return os;
}

}  // namespace duatic::geometry
