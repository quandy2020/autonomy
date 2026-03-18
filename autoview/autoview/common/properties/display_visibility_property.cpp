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

#include "autoview/common/properties/display_visibility_property.hpp"

#include "autoview/common/properties/bool_property.hpp"
#include "autoview/common/display_context.hpp"
#include "autoview/common/bit_allocator.hpp"
#include "autoview/common/display.hpp"
#include "autoview/common/display_group.hpp"

namespace autoview {
namespace common {
namespace properties {

DisplayVisibilityProperty::DisplayVisibilityProperty(
uint32_t vis_bit,
Display * display,
const QString & name,
bool default_value,
const QString & description,
Property * parent,
const char * changed_slot,
QObject * receiver)
: BoolProperty(name, default_value, description, parent, changed_slot, receiver),
vis_bit_(vis_bit), display_(display)
{
    custom_name_ = (name.size() != 0);
    update();
}

DisplayVisibilityProperty::~DisplayVisibilityProperty() = default;

void DisplayVisibilityProperty::update()
{
    // update name, unless we had a custom name given in the constructor
    if (!custom_name_ && getName() != display_->getName()) {
        setName(display_->getName());
    }
    if (getBool() && (getViewFlags(0) & Qt::ItemIsEnabled)) {
        display_->setVisibilityBits(vis_bit_);
    } else {
        display_->unsetVisibilityBits(vis_bit_);
    }
}

bool DisplayVisibilityProperty::setValue(const QVariant & new_value)
{
    if (Property::setValue(new_value)) {
        update();
        return true;
    }
    return false;
}

bool DisplayVisibilityProperty::getBool() const
{
    if (!display_->isEnabled()) {
        return false;
    }
    return BoolProperty::getBool();
}

Qt::ItemFlags DisplayVisibilityProperty::getViewFlags(int column) const
{
    if (!display_->isEnabled()) {
        return Qt::ItemIsSelectable;
    }
    return BoolProperty::getViewFlags(column);
}

}  // namespace properties
}  // namespace common
}  // namespace autoview