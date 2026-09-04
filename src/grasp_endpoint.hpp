// SPDX-FileCopyrightText: 2026 Sunghyun Kwon
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <string_view>

namespace grasp_synergy_adapter
{

inline std::string ControllerName(std::string_view grasp_name)
{
    const auto valid_character = [](unsigned char character)
    { return std::isalnum(character) != 0 || character == '_'; };
    if (grasp_name.empty() ||
        !(std::isalpha(static_cast<unsigned char>(grasp_name.front())) != 0 ||
          grasp_name.front() == '_') ||
        !std::ranges::all_of(grasp_name, valid_character))
    {
        throw std::invalid_argument("grasp name must be a valid ROS name token");
    }
    return std::string{grasp_name} + "_controller";
}

} // namespace grasp_synergy_adapter
