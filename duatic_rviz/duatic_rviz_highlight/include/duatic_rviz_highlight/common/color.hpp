/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace duatic::rviz_highlight
{

/** @brief An RGB color, each channel from 0 to 255. */
struct Color
{
  std::uint8_t red;
  std::uint8_t green;
  std::uint8_t blue;

  /** @brief Same as `Color{ red, green, blue }`, but allows common editors to preview the color. */
  static consteval Color rgb(int red, int green, int blue)
  {
    if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255) {
      throw "a color channel is not in 0..255";
    }
    return { static_cast<std::uint8_t>(red), static_cast<std::uint8_t>(green), static_cast<std::uint8_t>(blue) };
  }

  bool operator==(const Color& other) const = default;
};

/** @brief Link name -> color to show the link in. Links that are not in the map keep their own look. */
using LinkColors = std::map<std::string, Color>;

/** @brief Common colors */
namespace palette
{
inline constexpr Color kRed = Color::rgb(255, 0, 0);
inline constexpr Color kOrange = Color::rgb(255, 140, 0);
inline constexpr Color kYellow = Color::rgb(255, 230, 0);
inline constexpr Color kGreen = Color::rgb(0, 204, 0);
inline constexpr Color kBlue = Color::rgb(0, 102, 255);
inline constexpr Color kPurple = Color::rgb(153, 51, 204);
inline constexpr Color kWhite = Color::rgb(255, 255, 255);
inline constexpr Color kGray = Color::rgb(128, 128, 128);
inline constexpr Color kBlack = Color::rgb(0, 0, 0);
}  // namespace palette

}  // namespace duatic::rviz_highlight
