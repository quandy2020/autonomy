#!/bin/sh
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0

# Fail when Doxygen wrote warnings. autocmake_documentation() sends them to
# autocmake-doxygen.warn in the build directory. Pass another path as $1.

warn_file="${1:-autocmake-doxygen.warn}"

if [ ! -f "$warn_file" ]; then
  echo "No Doxygen warning file at $warn_file. Skipping doc check."
  exit 0
fi

if [ -s "$warn_file" ]; then
  echo "Doxygen warnings in $warn_file:"
  cat "$warn_file"
  exit 1
fi

echo "No Doxygen warnings in $warn_file."
exit 0
