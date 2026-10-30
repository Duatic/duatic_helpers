# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Waiting on a node without rclpy's global executor.

rclpy.spin_once() and rclpy.spin_until_future_complete() share one executor per process, so
two threads waiting at the same time get "Executor is already spinning", as soon as several
nodes in one process use these helpers. A private executor per wait does not share.
"""

from rclpy.executors import SingleThreadedExecutor


def spin_once(node, timeout_sec: float | None = None) -> None:
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        executor.spin_once(timeout_sec=timeout_sec)
    finally:
        executor.remove_node(node)
        executor.shutdown()


def spin_until_future_complete(node, future, timeout_sec: float | None = None) -> None:
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
    finally:
        executor.remove_node(node)
        executor.shutdown()
