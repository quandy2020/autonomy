/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#version 330 core

layout(location = 0) in vec3 vertPos;
uniform mat4 mvp;
uniform mat4 model;  // Transform matrix for this link
uniform vec3 color;
out vec3 Color;

void main(void)
{
    vec4 worldPos = model * vec4(vertPos, 1.0);
    gl_Position = mvp * worldPos;
    Color = color;
}
