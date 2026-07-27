/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_context.hpp"

namespace autoviz {
namespace common {

void DisplayContext::queueRender() {
  if (request_redraw) {
    request_redraw();
  }
}

}  // namespace common
}  // namespace autoviz
