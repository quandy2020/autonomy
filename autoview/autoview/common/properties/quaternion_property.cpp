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

 #include "autoview/common/properties/quaternion_property.hpp"

 #include <QStringList>
 
 namespace autoview {
 namespace common {
 namespace properties {

 
 QuaternionProperty::QuaternionProperty(
   const QString & name,
   const Ogre::Quaternion & default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : Property(name, QVariant(), description, parent, changed_slot, receiver),
   quaternion_(default_value),
   ignore_child_updates_(false)
 {
   x_ = new Property("X", quaternion_.x, "X coordinate", this);
   y_ = new Property("Y", quaternion_.y, "Y coordinate", this);
   z_ = new Property("Z", quaternion_.z, "Z coordinate", this);
   w_ = new Property("W", quaternion_.w, "W coordinate", this);
   updateString();
   connect(x_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(y_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(z_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(w_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
   connect(x_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
   connect(y_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
   connect(z_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
   connect(w_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
 }
 
 bool QuaternionProperty::setQuaternion(const Ogre::Quaternion & new_quaternion)
 {
   if (new_quaternion != quaternion_) {
     Q_EMIT aboutToChange();
     quaternion_ = new_quaternion;
     ignore_child_updates_ = true;
     x_->setValue(quaternion_.x);
     y_->setValue(quaternion_.y);
     z_->setValue(quaternion_.z);
     w_->setValue(quaternion_.w);
     ignore_child_updates_ = false;
     updateString();
     Q_EMIT changed();
     return true;
   }
   return false;
 }
 
 bool QuaternionProperty::setValue(const QVariant & new_value)
 {
   QStringList strings = new_value.toString().split(';');
   if (strings.size() >= 4) {
     bool x_ok = true;
     float x = strings[0].toFloat(&x_ok);
     bool y_ok = true;
     float y = strings[1].toFloat(&y_ok);
     bool z_ok = true;
     float z = strings[2].toFloat(&z_ok);
     bool w_ok = true;
     float w = strings[3].toFloat(&w_ok);
     if (x_ok && y_ok && z_ok && w_ok) {
       return setQuaternion(Ogre::Quaternion(w, x, y, z));
     }
   }
   return false;
 }
 
 void QuaternionProperty::updateFromChildren()
 {
   if (!ignore_child_updates_) {
     quaternion_.x = x_->getValue().toFloat();
     quaternion_.y = y_->getValue().toFloat();
     quaternion_.z = z_->getValue().toFloat();
     quaternion_.w = w_->getValue().toFloat();
     updateString();
     Q_EMIT changed();
   }
 }
 
 void QuaternionProperty::emitAboutToChange()
 {
   if (!ignore_child_updates_) {
     Q_EMIT aboutToChange();
   }
 }
 
 void QuaternionProperty::updateString()
 {
   value_ = QString("%1; %2; %3; %4")
     .arg(quaternion_.x, 0, 'g', 5)
     .arg(quaternion_.y, 0, 'g', 5)
     .arg(quaternion_.z, 0, 'g', 5)
     .arg(quaternion_.w, 0, 'g', 5);
 }
 
 void QuaternionProperty::load(const Config & config)
 {
   float x, y, z, w;
   if (config.mapGetFloat("X", &x) &&
     config.mapGetFloat("Y", &y) &&
     config.mapGetFloat("Z", &z) &&
     config.mapGetFloat("W", &w))
   {
     // Calling setQuaternion() once explicitly is better than letting
     // the Property class load the X, Y, Z, and W children
     // independently, which would result in at least 4 calls to
     // setQuaternion().
     setQuaternion(Ogre::Quaternion(w, x, y, z));
   }
 }
 
 void QuaternionProperty::save(Config config) const
 {
   // Saving the child values explicitly avoids having Property::save()
   // save the summary string version of the property.
   config.mapSetValue("X", x_->getValue() );
   config.mapSetValue("Y", y_->getValue() );
   config.mapSetValue("Z", z_->getValue() );
   config.mapSetValue("W", w_->getValue() );
 }
 
 void QuaternionProperty::setReadOnly(bool read_only)
 {
   Property::setReadOnly(read_only);
   x_->setReadOnly(read_only);
   y_->setReadOnly(read_only);
   z_->setReadOnly(read_only);
   w_->setReadOnly(read_only);
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 