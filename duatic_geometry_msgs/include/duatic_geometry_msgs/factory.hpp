#pragma once

#include <type_traits>
#include <duatic_geometry/annotation.hpp>
#include <duatic_geometry_msgs/encoder.hpp>

using namespace duatic::geometry;

// Kept as the flat `duatic_geometry_msgs` namespace (matching the rosidl-generated
// `duatic_geometry_msgs::msg::*` types living in the same package) rather than the
// `duatic::geometry_msgs` nesting used elsewhere, so that references to the unrelated,
// external `::geometry_msgs` package never get shadowed by our own namespace.
namespace duatic_geometry_msgs
{

// std::conditional_t substitutes both branches unconditionally, so it can't be used
// directly with typename U::DataType (U may not have one); dispatch via specialization
// instead. Must live at namespace scope, not nested in Factory<T>: see
// KinematicVariableMsgTypeHelper in encoder_kinematic_variable.hpp for the same reason.
template <typename U, bool is_annotated>
struct FactoryDataTypeHelper
{
  using type = U;
};
template <typename U>
struct FactoryDataTypeHelper<U, true>
{
  using type = typename U::DataType;
};

// ---------------------------------------------------------------------------
// Factory
//
// Converts between a duatic_geometry data type T and its ROS 2 message
// counterparts: `msg` (no header) and `msg_stamped` (with header). T may be a
// plain KinematicVariable, a KinematicState,
// or either of those wrapped in TimedData<> / StampedData<> --
// whether T satisfies the Timed/Stamped concepts is evaluated automatically and,
// if so, decode()/encode() additionally transfer the message header's stamp (and
// frame_id, if stamped).
// ---------------------------------------------------------------------------
template <typename T, template <typename> typename EncoderT = FactoryEncoder>
class Factory
{
public:
  static constexpr bool is_timed = is_timed_v<T>;
  static constexpr bool is_stamped = is_stamped_v<T>;

  static constexpr bool is_annotated = is_timed || is_stamped;

private:
  using DataType = typename FactoryDataTypeHelper<T, is_annotated>::type;

  using EncoderImpl = EncoderT<DataType>;
  static_assert(Encoder<EncoderImpl>);

public:
  using msg_stamped = typename EncoderImpl::msg_stamped;
  using msg = std::conditional_t<is_annotated, msg_stamped, typename EncoderImpl::msg>;

  template <typename MSG>
    requires std::is_base_of_v<msg, MSG> || std::is_base_of_v<msg_stamped, MSG>
  static void encode(const T& data, MSG& msg)
  {
    // header handling
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      if constexpr (is_timed) {
        msg.header.stamp = data.time();
      }
      if constexpr (is_stamped) {
        static_assert(is_timed);
        msg.header.frame_id = data.frame_id();
      }
    }
    // data handling
    if constexpr (is_annotated) {
      EncoderImpl::encode(data.data(), msg);
    } else {
      EncoderImpl::encode(data, msg);
    }
  }

  template <typename MSG>
    requires std::is_base_of_v<msg, MSG> || std::is_base_of_v<msg_stamped, MSG>
  static void decode(const MSG& msg, T& data)
  {
    // header handling
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      if constexpr (is_timed) {
        data.time() = msg.header.stamp;
      }
      if constexpr (is_stamped) {
        static_assert(is_timed);
        data.frame_id() = msg.header.frame_id;
      }
    }
    // data handling
    if constexpr (is_annotated) {
      EncoderImpl::decode(msg, data.data());
    } else {
      EncoderImpl::decode(msg, data);
    }
  }
};

}  // namespace duatic_geometry_msgs
