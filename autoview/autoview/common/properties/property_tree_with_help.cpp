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

 #include "autoview/common/properties/property_tree_with_help.hpp"

 #include <QTextBrowser>  // NOLINT: cpplint is unable to handle the include order here
 
 #include "autoview/common/properties/property_tree_widget.hpp"
 #include "autoview/common/properties/property.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 PropertyTreeWithHelp::PropertyTreeWithHelp(QWidget * parent)
 : QSplitter(parent)
 {
   setOrientation(Qt::Vertical);
 
   property_tree_ = new PropertyTreeWidget;
   property_tree_->setObjectName("TreeWithHelp/PropertyTree");
 
   help_ = new QTextBrowser;
   help_->setOpenExternalLinks(true);
 
   addWidget(property_tree_);
   addWidget(help_);
 
   setStretchFactor(0, 1000);
   setCollapsible(0, false);
 
   QList<int> _sizes;
   _sizes.push_back(1000);
   _sizes.push_back(1);
   setSizes(_sizes);
 
   connect(
     property_tree_, SIGNAL(currentPropertyChanged(const Property*)),
     this, SLOT(showHelpForProperty(const Property*)));
 }
 
 void PropertyTreeWithHelp::showHelpForProperty(const Property * property)
 {
   if (property) {
     QString body_text = property->getDescription();
     QString heading = property->getName();
     body_text.replace("\n", "<br>");
     QString html = "<html><body><strong>" + heading + "</strong><br>" +
       body_text + "</body></html>";
     help_->setHtml(html);
   } else {
     help_->setHtml("");
   }
 }
 
 void PropertyTreeWithHelp::save(Config config) const
 {
   property_tree_->save(config.mapMakeChild("Property Tree Widget"));
 
   QList<int> _sizes = sizes();
   config.mapSetValue("Tree Height", _sizes.at(0));
   config.mapSetValue("Help Height", _sizes.at(1));
 }
 
 void PropertyTreeWithHelp::load(const Config & config)
 {
   property_tree_->load(config.mapGetChild("Property Tree Widget"));
 
   int tree_height;
   int help_height;
   if (config.mapGetInt("Tree Height", &tree_height) &&
     config.mapGetInt("Help Height", &help_height))
   {
     QList<int> _sizes;
     _sizes.push_back(tree_height);
     _sizes.push_back(help_height);
     setSizes(_sizes);
   }
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 