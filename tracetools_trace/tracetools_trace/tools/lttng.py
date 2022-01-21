# Copyright 2019 Robert Bosch GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Interface for tracing with LTTng."""

import platform
import subprocess
import sys
from typing import List
from typing import Optional

try:
    from . import lttng_impl

    _lttng = lttng_impl  # type: ignore

    # Check lttng module version
    from distutils.version import StrictVersion
    current_version = _lttng.get_version()
    LTTNG_MIN_VERSION = '2.10.7'
    if current_version is None or current_version < StrictVersion(LTTNG_MIN_VERSION):
        print(
            f'lttng module version >={LTTNG_MIN_VERSION} required, found {str(current_version)}',
            file=sys.stderr,
        )
except ImportError:
    # Fall back on stub functions so that this still passes linter checks
    from . import lttng_stub

    _lttng = lttng_stub  # type: ignore

from .names import DEFAULT_CONTEXT
from .names import DEFAULT_EVENTS_KERNEL
from .names import DEFAULT_EVENTS_ROS


def lttng_init(
    session_name: str,
    base_path: str,
    ros_events: List[str] = DEFAULT_EVENTS_ROS,
    kernel_events: List[str] = DEFAULT_EVENTS_KERNEL,
    context_names: List[str] = DEFAULT_CONTEXT,
) -> Optional[str]:
    """
    Set up and start LTTng session.

    :param session_name: the name of the session
    :param base_path: the path to the directory in which to create the tracing session directory
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param context_names: list of context elements to enable
    :return: the full path to the trace directory
    """
    trace_directory = _lttng.setup(
        session_name,
        base_path,
        ros_events,
        kernel_events,
        context_names,
    )
    _lttng.start(session_name)
    return trace_directory


def lttng_fini(
    session_name: str,
) -> None:
    """
    Stop and destroy LTTng session.

    :param session_name: the name of the session
    """
    _lttng.stop(session_name)
    _lttng.destroy(session_name)


def is_lttng_installed() -> bool:
    """
    Check if LTTng is installed.

    It first checks if the OS can support LTTng.
    If so, it then simply checks if LTTng is installed using the 'lttng' command.

    :return: True if it is installed, False otherwise
    """
    message_doc = (
        'Cannot trace. See documentation at: '
        'https://gitlab.com/ros-tracing/ros2_tracing'
    )
    system = platform.system()
    if 'Linux' != system:
        print(f"System '{system}' does not support LTTng.\n{message_doc}", file=sys.stderr)
        return False
    try:
        process = subprocess.Popen(
            ['lttng', '--version'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _, stderr = process.communicate()
        if 0 != process.returncode:
            raise RuntimeError(stderr.decode())
        return True
    except (RuntimeError, FileNotFoundError) as e:
        print(f'LTTng not found: {e}\n{message_doc}', file=sys.stderr)
        return False
