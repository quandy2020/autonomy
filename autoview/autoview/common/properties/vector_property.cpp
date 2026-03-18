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

 #include "autoview/common/properties/vector_property.hpp"

 #include <QStringList>
 
 namespace autoview {
 namespace common {
 namespace properties {

 
 VectorProperty::VectorProperty(
   const QString & name,
   const Ogre::Vector3 & default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : Property(name, QVariant(), description, parent, changed_slot, receiver),
   vector_(default_value),
   ignore_child_updates_(false)
 {
   x_ = new Property("X", vector_.x, "X coordinate", this);
   y_ = new Property("Y", vector_.y, "Y coordinate", this);
   z_ = new Property("Z", vector_.z, "Z coordinate", this);
   updateString();
   connect(x_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(y_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(z_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(x_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
   connect(y_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
   connect(z_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
 }
 
 bool VectorProperty::setVector(const Ogre::Vector3 & new_vector)
 {
   if (new_vector != vector_) {
     Q_EMIT aboutToChange();
     vector_ = new_vector;
     ignore_child_updates_ = true;
     x_->setValue(vector_.x);
     y_->setValue(vector_.y);
     z_->setValue(vector_.z);
     ignore_child_updates_ = false;
     updateString();
     Q_EMIT changed();
     return true;
   }
   return false;
 }
 
 Ogre::Vector3 VectorProperty::getVector() const
 {
   return vector_;
 }
 
 bool VectorProperty::add(const Ogre::Vector3 & offset)
 {
   return setVector(getVector() + offset);
 }
 
 bool VectorProperty::setValue(const QVariant & new_value)
 {
   QStringList strings = new_value.toString().split(';');
   if (strings.size() >= 3) {
     bool x_ok = true;
     float x = strings[0].toFloat(&x_ok);
     bool y_ok = true;
     float y = strings[1].toFloat(&y_ok);
     bool z_ok = true;
     float z = strings[2].toFloat(&z_ok);
     if (x_ok && y_ok && z_ok) {
       return setVector(Ogre::Vector3(x, y, z));
     }
   }
   return false;
 }
 
 void VectorProperty::updateFromChildren()
 {
   if (!ignore_child_updates_) {
     vector_.x = x_->getValue().toFloat();
     vector_.y = y_->getValue().toFloat();
     vector_.z = z_->getValue().toFloat();
     updateString();
     Q_EMIT changed();
   }
 }
 
 void VectorProperty::emitAboutToChange()
 {
   if (!ignore_child_updates_) {
     Q_EMIT aboutToChange();
   }
 }
 
 void VectorProperty::updateString()
 {
   value_ = QString("%1; %2; %3")
     .arg(vector_.x, 0, 'g', 5)
     .arg(vector_.y, 0, 'g', 5)
     .arg(vector_.z, 0, 'g', 5);
 }
 
 void VectorProperty::load(const Config & config)
 {
   float x, y, z;
   if (config.mapGetFloat("X", &x) &&
     config.mapGetFloat("Y", &y) &&
     config.mapGetFloat("Z", &z))
   {
     // Calling setVector() once explicitly is better than letting the
     // Property class load the X, Y, and Z children independently,
     // which would result in at least 3 calls to setVector().
     setVector(Ogre::Vector3(x, y, z));
   }
 }
 
 void VectorProperty::save(Config config) const
 {
   // Saving the child values explicitly avoids having Property::save()
   // save the summary string version of the property.
   config.mapSetValue("X", x_->getValue());
   config.mapSetValue("Y", y_->getValue());
   config.mapSetValue("Z", z_->getValue());
 }
 
 void VectorProperty::setReadOnly(bool read_only)
 {
   Property::setReadOnly(read_only);
   x_->setReadOnly(read_only);
   y_->setReadOnly(read_only);
   z_->setReadOnly(read_only);
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 