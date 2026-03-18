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

#pragma once

#include <OgreVector3.h>

 #include "autoview/common/properties/property.hpp"

 namespace autoview {
 namespace common {
 namespace properties {

 
 class VectorProperty : public Property
 {
   Q_OBJECT
 
 public:
   explicit
   VectorProperty(
     const QString & name = QString(),
     const Ogre::Vector3 & default_value = Ogre::Vector3::ZERO,
     const QString & description = QString(),
     Property * parent = 0,
     const char * changed_slot = 0,
     QObject * receiver = 0);
 
   virtual
   bool
   setVector(const Ogre::Vector3 & vector);
 
   virtual
   Ogre::Vector3
   getVector() const;
 
   bool
   add(const Ogre::Vector3 & offset);
 
   virtual
   bool
   setValue(const QVariant & new_value);
 
   virtual
   void
   load(const Config & config);
 
   virtual
   void
   save(Config config) const;
 
   /// Overridden from Property to propagate read-only-ness to children.
   virtual
   void
   setReadOnly(bool read_only);
 
 private Q_SLOTS:
   void
   updateFromChildren();
 
   void
   emitAboutToChange();
 
 private:
   void
   updateString();
 
   Ogre::Vector3 vector_;
   Property * x_;
   Property * y_;
   Property * z_;
 
   bool ignore_child_updates_;
 };

}  // namespace properties
}  // namespace common
}  // namespace autoview
