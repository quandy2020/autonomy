#!/usr/bin/env bash
#
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Bash tab-completion for automsgs-msgs

AUTOMSGS_MSGS_COMPLETION_LIST="
  -i --info
  -l --list
  -h --help
  --version
"

function _automsgs_msgs
{
  if [[ ${COMP_WORDS[COMP_CWORD]} == -* ]]; then
    COMPREPLY=($(compgen -W "$AUTOMSGS_MSGS_COMPLETION_LIST" \
      -- "${COMP_WORDS[COMP_CWORD]}"))
    return
  else
    COMPREPLY=($(compgen -o default -- "${COMP_WORDS[COMP_CWORD]}"))
    return
  fi
}

function _automsgs_msgs_flags
{
  for word in $AUTOMSGS_MSGS_COMPLETION_LIST; do
    echo "$word"
  done
}
