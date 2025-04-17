# Copyright 2019 Apex.AI, Inc.
# Copyright 2021 Christophe Bedard
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

"""Module for the LdPreload action."""

import platform
import subprocess
from typing import List
from typing import Optional
from typing import Text

from launch import logging
from launch.action import Action
from launch.actions import AppendEnvironmentVariable
from launch.launch_context import LaunchContext


class LdPreload(Action):
    """Action that adds an AppendEnvironmentVariable action to preload a library."""

    ENV_VAR_LD_PRELOAD = 'LD_PRELOAD'

    _logger = logging.get_logger(__name__)

    def __init__(
        self,
        lib_name: str,
        **kwargs,
    ) -> None:
        """
        Create an LdPreload action.

        :param lib_name: the name of the library (e.g., 'lib.so')
        """
        super().__init__(**kwargs)
        self._lib_name = lib_name
        self._env_action: Optional[Action] = None
        # Try to find lib
        self._lib_path = self.get_shared_lib_path(self._lib_name)
        # And create action if found
        if self._lib_path is not None:
            self._logger.debug(f"Shared library for '{lib_name}' found at: {self._lib_path}")
            self._env_action = AppendEnvironmentVariable(
                self.ENV_VAR_LD_PRELOAD,
                self._lib_path,
            )
        else:
            self._logger.warning(
                f"Could not find shared library for '{lib_name}': {self._lib_name}")

    @property
    def lib_name(self) -> str:
        return self._lib_name

    @property
    def lib_path(self) -> Optional[str]:
        """
        Get the resolved shared lib path, if found.

        :return: the full path to the shared lib if found, or `None` if not found
        """
        return self._lib_path

    def lib_found(self) -> bool:
        """
        Check if the shared lib was found.

        :return: `True` if the shared lib was found, or `False` if not found
        """
        return self._env_action is not None

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        if self.lib_found():
            return [self._env_action]  # type: ignore[list-item]
        return None

    @classmethod
    def get_shared_lib_path(cls, lib_name: str) -> Optional[str]:
        """
        Get the full path to a given shared library, if possible.

        This will not work on non-Linux systems.

        :param lib_name: the name of the shared library (e.g., 'lib.so' or just 'lib')
        :return: the full path if found, `None` otherwise
        """
        # Do not continue if not on Linux
        if 'Linux' != platform.system():
            return None
        (exit_code, output) = subprocess.getstatusoutput(f'whereis -b {lib_name}')
        cls._logger.debug(f"whereis command for '{lib_name}' exited with {exit_code}: {output}")
        if 0 != exit_code:
            return None
        # Output of whereis is:
        #   <lib_name>:[ <full lib path, if found>[ <alternative full lib path>]]
        # Example when we do get results:
        #   lib.so: /path/to/lib.so /path/to/lib.a
        # Example when we do not get any results:
        #   lib.so:
        # Split on the separator between lib name and paths
        output_split = list(filter(None, output.split(': ')))
        if 2 != len(output_split):
            return None
        # Assuming that there are no spaces in paths (which should be valid for Linux libs)
        paths = output_split[1].split(' ')
        cls._logger.debug(f"lib paths for '{lib_name}': {paths}")
        # Try to find a shared library
        # Paths could contain: shared lib (.so), static lib (.a), or libtools text file (.la)
        shared_lib_paths = [path for path in paths if path.endswith('.so')]
        if not shared_lib_paths:
            return None
        shared_lib = shared_lib_paths[0]
        return shared_lib

    def __repr__(self) -> Text:
        return (
            'LdPreload('
            f'lib_name={self._lib_name}, '
            f'lib found={self.lib_found()}, '
            f'lib_path={self._lib_path})'
        )
