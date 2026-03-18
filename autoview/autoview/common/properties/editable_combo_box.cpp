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

#include "autoview/common/properties/editable_combo_box.hpp"

#include <cstdio>

#include <QCompleter>  
#include <QKeyEvent>
#include <QLineEdit> 
namespace autoview {
namespace common {
namespace properties {

EditableComboBox::EditableComboBox(QWidget * parent)
: ComboBox(parent)
{
    setEditable(true);
    if (completer()) {
        completer()->setCompletionMode(QCompleter::PopupCompletion);
        completer()->setCaseSensitivity(Qt::CaseInsensitive);
    }
}

QString findMaxCommonPrefix(const QStringList & strings)
{
    if (strings.size() == 0) {
        return "";
    }
    if (strings.size() == 1) {
        return strings[0];
    }
    QString common_prefix;
    int char_index = 0;

    // loop over character index
    while (true) {
        if (char_index >= strings[0].size()) {
            return common_prefix;
        }
        const QChar c = strings[0][char_index];

        // loop over strings
        for (int string_index = 1; string_index < strings.size(); string_index++) {
            const QString & str = strings[string_index];
            if (char_index >= str.size() ||
                str[char_index] != c) {
                return common_prefix;
            }
        }
        common_prefix += c;
        char_index++;
    }
    return "";  // just to satisfy compiler... I know it will never reach this.
}

bool EditableComboBox::event(QEvent * event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent * k = dynamic_cast<QKeyEvent *>(event);
        if (k->key() == Qt::Key_Tab && k->modifiers() == Qt::NoModifier) {
            QCompleter * comp = completer();

            if (comp) {
            QStringList completions;
            for (int i = 0; comp->setCurrentRow(i); i++) {
                completions.push_back(comp->currentCompletion());
            }
            QString max_common_prefix = findMaxCommonPrefix(completions);
            if (max_common_prefix.size() > currentText().size()) {
                setEditText(max_common_prefix);
                lineEdit()->setCursorPosition(max_common_prefix.size());
            }

            event->accept();
            return true;
            }
        }
    }
    return ComboBox::event(event);
}

}  // namespace properties
}  // namespace common
}  // namespace autoview