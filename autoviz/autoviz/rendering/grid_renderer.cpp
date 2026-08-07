/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/grid_renderer.hpp"

#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QtMath>
#include <vector>

#include "autoviz/rendering/render_settings.hpp"

namespace autoviz {
namespace rendering {

QMatrix4x4 CameraState::viewMatrix() const {
  const float x = distance * qCos(yaw) * qCos(pitch);
  const float y = distance * qSin(yaw) * qCos(pitch);
  const float z = distance * qSin(pitch);
  const QVector3D eye = target + QVector3D(x, y, z);
  QMatrix4x4 view;
  view.lookAt(eye, target, QVector3D(0.f, 0.f, 1.f));
  return view;
}

QMatrix4x4 CameraState::projectionMatrix(float aspect_ratio) const {
  QMatrix4x4 projection;
  projection.perspective(45.f, aspect_ratio, 0.05f, 500.f);
  return projection;
}

namespace {

constexpr char kGridVertexShader[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uMvp;
void main() { gl_Position = uMvp * vec4(aPos, 1.0); }
)";

constexpr char kGridFragmentShader[] = R"(#version 330 core
uniform vec3 uColor;
out vec4 fragColor;
void main() { fragColor = vec4(uColor, 1.0); }
)";

constexpr char kColorVertexShader[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMvp;
out vec3 vColor;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
  vColor = aColor;
}
)";

constexpr char kColorFragmentShader[] = R"(#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";

unsigned CompileShader(unsigned type, const char* source) {
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  const unsigned shader = gl->glCreateShader(type);
  gl->glShaderSource(shader, 1, &source, nullptr);
  gl->glCompileShader(shader);
  return shader;
}

unsigned LinkProgram(const char* vs, const char* fs) {
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  const unsigned v_shader = CompileShader(0x8B31, vs);
  const unsigned f_shader = CompileShader(0x8B30, fs);
  const unsigned program = gl->glCreateProgram();
  gl->glAttachShader(program, v_shader);
  gl->glAttachShader(program, f_shader);
  gl->glLinkProgram(program);
  gl->glDeleteShader(v_shader);
  gl->glDeleteShader(f_shader);
  return program;
}

}  // namespace

void GridRenderer::ensureProgram() {
  if (grid_program_ != 0) {
    return;
  }
  grid_program_ = LinkProgram(kGridVertexShader, kGridFragmentShader);
  axis_program_ = LinkProgram(kColorVertexShader, kColorFragmentShader);
}

void GridRenderer::setReferenceGridSettings(
    const ReferenceGridSettings& settings) {
  if (grid_settings_ == settings) {
    return;
  }
  grid_settings_ = settings;
  if (initialized_) {
    buildGridGeometry();
  }
}

void GridRenderer::buildGridGeometry() {
  QOpenGLExtraFunctions* gl =
      QOpenGLContext::currentContext()->extraFunctions();

  std::vector<float> grid_vertices;
  const int cell_count = std::max(1, grid_settings_.plane_cell_count);
  const float step = std::max(0.001f, grid_settings_.cell_length);
  const float extent = static_cast<float>(cell_count) * step * 0.5f;
  for (int i = 0; i <= cell_count; ++i) {
    const float p = extent - static_cast<float>(i) * step;
    grid_vertices.insert(grid_vertices.end(),
                         {-extent, p, 0.f, extent, p, 0.f});
    grid_vertices.insert(grid_vertices.end(),
                         {p, -extent, 0.f, p, extent, 0.f});
  }
  grid_vertex_count_ = static_cast<int>(grid_vertices.size() / 3);

  if (grid_vao_ == 0) {
    gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&grid_vao_));
    gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&grid_vbo_));
  }
  gl->glBindVertexArray(grid_vao_);
  gl->glBindBuffer(0x8892, grid_vbo_);
  gl->glBufferData(0x8892,
                   static_cast<int>(grid_vertices.size() * sizeof(float)),
                   grid_vertices.data(), 0x88E4);
  gl->glEnableVertexAttribArray(0);
  gl->glVertexAttribPointer(0, 3, 0x1406, 0, 3 * sizeof(float), nullptr);

  const float axis_len = grid_settings_.axis_length;
  const float axis_vertices[] = {
      0.f, 0.f, 0.f, 1.f, 0.f, 0.f, axis_len, 0.f, 0.f, 1.f, 0.f, 0.f,
      0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, axis_len, 0.f, 0.f, 1.f, 0.f,
      0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, axis_len, 0.f, 0.f, 1.f,
  };
  if (axis_vao_ == 0) {
    gl->glGenVertexArrays(1, reinterpret_cast<unsigned*>(&axis_vao_));
    gl->glGenBuffers(1, reinterpret_cast<unsigned*>(&axis_vbo_));
  }
  gl->glBindVertexArray(axis_vao_);
  gl->glBindBuffer(0x8892, axis_vbo_);
  gl->glBufferData(0x8892, sizeof(axis_vertices), axis_vertices, 0x88E4);
  gl->glEnableVertexAttribArray(0);
  gl->glVertexAttribPointer(0, 3, 0x1406, 0, 6 * sizeof(float), nullptr);
  gl->glEnableVertexAttribArray(1);
  gl->glVertexAttribPointer(1, 3, 0x1406, 0, 6 * sizeof(float),
                            reinterpret_cast<void*>(3 * sizeof(float)));
  gl->glBindVertexArray(0);
}

void GridRenderer::initialize() {
  if (initialized_) {
    return;
  }
  ensureProgram();
  buildGridGeometry();
  initialized_ = true;
}

void GridRenderer::resize(int width, int height) {
  viewport_width_ = std::max(1, width);
  viewport_height_ = std::max(1, height);
}

void GridRenderer::setBackgroundColor(const QColor& color) {
  background_color_ = color;
}

void GridRenderer::render(const QMatrix4x4& view, const QMatrix4x4& projection) {
  if (!initialized_) {
    return;
  }

  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();

  gl->glViewport(0, 0, viewport_width_, viewport_height_);
  gl->glClearColor(background_color_.redF(), background_color_.greenF(),
                  background_color_.blueF(), 1.f);
  gl->glClear(0x00004000 | 0x00000100);

  if (!visible_) {
    return;
  }

  const QMatrix4x4 mvp = projection * view;

  gl->glUseProgram(grid_program_);
  gl->glUniformMatrix4fv(gl->glGetUniformLocation(grid_program_, "uMvp"), 1, 0,
                          mvp.constData());
  gl->glUniform3f(gl->glGetUniformLocation(grid_program_, "uColor"),
                  grid_settings_.color.redF(), grid_settings_.color.greenF(),
                  grid_settings_.color.blueF());
  gl_extra->glBindVertexArray(grid_vao_);
  gl->glDrawArrays(0x0001, 0, grid_vertex_count_);

  if (grid_settings_.show_axes) {
    gl->glLineWidth(2.f);
    gl->glUseProgram(axis_program_);
    gl->glUniformMatrix4fv(gl->glGetUniformLocation(axis_program_, "uMvp"), 1,
                            0, mvp.constData());
    gl_extra->glBindVertexArray(axis_vao_);
    gl->glDrawArrays(0x0001, 0, 6);
    gl->glLineWidth(1.f);
  }
  gl_extra->glBindVertexArray(0);
}

void GridRenderer::shutdown() {
  if (!initialized_) {
    return;
  }
  QOpenGLExtraFunctions* gl =
      QOpenGLContext::currentContext()->extraFunctions();
  QOpenGLFunctions* gl_base = QOpenGLContext::currentContext()->functions();
  if (grid_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&grid_vao_));
    grid_vao_ = 0;
  }
  if (grid_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&grid_vbo_));
    grid_vbo_ = 0;
  }
  if (axis_vao_ != 0) {
    gl->glDeleteVertexArrays(1, reinterpret_cast<unsigned*>(&axis_vao_));
    axis_vao_ = 0;
  }
  if (axis_vbo_ != 0) {
    gl->glDeleteBuffers(1, reinterpret_cast<unsigned*>(&axis_vbo_));
    axis_vbo_ = 0;
  }
  if (grid_program_ != 0) {
    gl_base->glDeleteProgram(grid_program_);
    grid_program_ = 0;
  }
  if (axis_program_ != 0) {
    gl_base->glDeleteProgram(axis_program_);
    axis_program_ = 0;
  }
  initialized_ = false;
}

void GridRenderer::render(const CameraState& camera) {
  const float aspect =
      static_cast<float>(viewport_width_) / static_cast<float>(viewport_height_);
  render(camera.viewMatrix(), camera.projectionMatrix(aspect));
}

}  // namespace rendering
}  // namespace autoviz
