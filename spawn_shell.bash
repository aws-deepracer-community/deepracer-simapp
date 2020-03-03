#!/bin/bash
# Copyright 2018 Shane Loretz
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

if [ "${COLCON_SPAWN_SHELL_GET_WORKSPACE_INFO}" = "workspace_name" ] ;
then
  # script is being sourced to get info and should not spawn a shell
  COLCON_SPAWN_SHELL_WORKSPACE_NAME='colcon'
  unset COLCON_SPAWN_SHELL_GET_WORKSPACE_INFO
  return
fi

# Get absolute path to install root ( directory this script is in)
_CCSSB_dir=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
# Append workspace to list of workspaces (for chaining workspaces)
_CCSSB_workspaces=$COLCON_SPAWN_SHELL_BASH:$_CCSSB_dir

# Need to source user's rcfiles first because --rcfile causes them to be ignored
if [ -f /etc/bash.bashrc ]
then
  _CCSSB_rcfile="$_CCSSB_rcfile . /etc/bash.bashrc ;"
fi
if [ -f ~/.bashrc ]
then
  _CCSSB_rcfile="$_CCSSB_rcfile . ~/.bashrc ;"
fi

# Build a prompt prefix showing the order workspaces have been chained
_CCSSB_ps1_prefix=""

# Make code to source all colcon workspaces
while read -d ':' _CCSSB_ws_dir; do
  if [ -z "$_CCSSB_ws_dir" ]
  then
    # No text before first ':' so ignore it
    continue
  fi
  # make sure shell sources the workspace on startup
  _CCSSB_rcfile="$_CCSSB_rcfile . $_CCSSB_ws_dir/local_setup.bash ;"

  # Source the workspace here to get the workspace name
  COLCON_SPAWN_SHELL_GET_WORKSPACE_INFO=workspace_name
  . $_CCSSB_ws_dir/spawn_shell.bash
  unset COLCON_SPAWN_SHELL_GET_WORKSPACE_INFO

  if [ -z "$_CCSSB_ps1_prefix" ]
  then
    # First workspace is separated from PS1 by "|"
    _CCSSB_ps1_prefix="${COLCON_SPAWN_SHELL_WORKSPACE_NAME}|"
  else
    # Chained workspaces are separated with "<-"
    _CCSSB_ps1_prefix="${COLCON_SPAWN_SHELL_WORKSPACE_NAME}<-$_CCSSB_ps1_prefix"
  fi
  unset COLCON_SPAWN_SHELL_WORKSPACE_NAME
done <<< "$_CCSSB_workspaces:"

# Support chaining by setting a variable with the list of spawned workspaces
_CCSSB_rcfile="$_CCSSB_rcfile COLCON_SPAWN_SHELL_BASH=\"$_CCSSB_workspaces\" ;"

# Set prompt to indicate sourced workspaces
_CCSSB_rcfile="$_CCSSB_rcfile export PS1=\"$_CCSSB_ps1_prefix\$PS1\" ;"

# Spawn a child shell using custom startup commands
$SHELL --rcfile <(echo "$_CCSSB_rcfile")

# Cleanup the variables used
unset _CCSSB_workspaces
unset _CCSSB_rcfile
unset _CCSSB_ps1_prefix
unset _CCSSB_ws_dir
unset _CCSSB_dir
