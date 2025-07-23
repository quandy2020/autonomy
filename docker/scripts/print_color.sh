#!/bin/bash -e

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

function print_color 
{
    tput setaf $1
    echo "$2"
    tput sgr0
}

function print_error 
{
    print_color 1 "$1"
}

function print_warning 
{
    print_color 3 "$1"
}

function print_info 
{
    print_color 2 "$1"
}
