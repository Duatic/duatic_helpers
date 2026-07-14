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

#include <gtest/gtest.h>
#include <utility>

#include <duatic_concurrency/unidirectional_buffer.hpp>

using duatic::concurrency::UnidirectionalBuffer;

TEST(UnidirectionalBuffer, RvalueConstruction)
{
  UnidirectionalBuffer<int> buf(42);
  EXPECT_EQ(buf.read(), 42);
}

TEST(UnidirectionalBuffer, LvalueConstruction)
{
  int x = 7;
  UnidirectionalBuffer<int> buf(x);
  EXPECT_EQ(buf.read(), 7);
}

TEST(UnidirectionalBuffer, PublishUpdate)
{
  UnidirectionalBuffer<int> buf(10);
  EXPECT_EQ(buf.read(), 10);
  buf.publish_write(11);
  EXPECT_EQ(buf.update_read(), 11);
  buf.publish_write(12);
  buf.publish_write(13);
  EXPECT_EQ(buf.update_read(), 13);
}

TEST(UnidirectionalBuffer, WriteAndPublish)
{
  UnidirectionalBuffer<int> buf(10);
  EXPECT_EQ(buf.read(), 10);
  buf.write() = 11;
  EXPECT_EQ(buf.update_read(), 10);
  buf.publish_write();
  EXPECT_EQ(buf.update_read(), 11);
  buf.write() = 12;
  buf.write() = 13;
  EXPECT_EQ(buf.update_read(), 11);
  buf.publish_write();
  EXPECT_EQ(buf.update_read(), 13);
}

TEST(UnidirectionalBuffer, Alignment)
{
  EXPECT_EQ(alignof(UnidirectionalBuffer<int>), static_cast<size_t>(std::hardware_destructive_interference_size));
}
