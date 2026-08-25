/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

// registers the KinematicVariable/KinematicState FactoryEncoder specializations, then pulls in the
// generic Factory-based encode()/decode()/msg_t/msg_stamped_t facade that uses them.
#include <duatic_geometry_encoder/encoder_kinematic_state.hpp>
#include <duatic_geometry_encoder/encoder_kinematic_variable.hpp>
#include <duatic_data_encoding/duatic_data_encoding.hpp>

// Compile-time verification: a plain KinematicVariable and a plain KinematicState must each compile
// against the generic encode()/decode() facade in duatic_data_encoding.hpp, both with and without a
// header (msg_t/msg_stamped_t)
#include <duatic_geometry/geometry.hpp>

static_assert(
    requires(const duatic::geometry::Pose3Dd& data, duatic::geometry::Pose3Dd& mutable_data,
             duatic::data_encoding::msg_t<duatic::geometry::Pose3Dd>& message,
             duatic::data_encoding::msg_stamped_t<duatic::geometry::Pose3Dd>& message_stamped) {
      duatic::data_encoding::encode(data, message);
      duatic::data_encoding::encode(data, message_stamped);
      duatic::data_encoding::decode(message, mutable_data);
      duatic::data_encoding::decode(message_stamped, mutable_data);
    },
    "a KinematicVariable (duatic::geometry::Pose3Dd) does not compile against the generic "
    "duatic::data_encoding encode()/decode() functions");

static_assert(
    requires(const duatic::geometry::StatePose3Dd& data, duatic::geometry::StatePose3Dd& mutable_data,
             duatic::data_encoding::msg_t<duatic::geometry::StatePose3Dd>& message,
             duatic::data_encoding::msg_stamped_t<duatic::geometry::StatePose3Dd>& message_stamped) {
      duatic::data_encoding::encode(data, message);
      duatic::data_encoding::encode(data, message_stamped);
      duatic::data_encoding::decode(message, mutable_data);
      duatic::data_encoding::decode(message_stamped, mutable_data);
    },
    "a KinematicState (duatic::geometry::StatePose3Dd) does not compile against the generic "
    "duatic::data_encoding encode()/decode() functions");
