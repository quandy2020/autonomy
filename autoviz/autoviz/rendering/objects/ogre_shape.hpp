/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_shape.hpp
 * @brief Primitive Entity shapes (cone/cube/cylinder/sphere/capsule).
 *
 * rviz_rendering::Shape subset used as building blocks for @ref OgreArrow and
 * Entity-based displays via MeshManager @c aviz_*.mesh resources.
 *
 * @see OgreArrow
 * @see ensureAvizPrimitiveMeshes()
 * @see OgreMeshLoader
 */

#pragma once

#include <string>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

namespace Ogre {
class Any;
class Entity;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @class OgreShape
 * @brief rviz_rendering::Shape subset — Entity + @c aviz_*.mesh primitives.
 *
 * Hierarchy: root @c scene_node_ → @c offset_node_ → Entity. Color is applied
 * through a unique material instance.
 */
class OgreShape {
 public:
  /**
   * @enum Type
   * @brief Supported unit primitive kinds.
   */
  enum Type {
    kCone,     /**< Unit cone. */
    kCube,     /**< Unit cube. */
    kCylinder, /**< Unit cylinder. */
    kSphere,   /**< Unit sphere. */
    kCapsule   /**< Unit capsule. */
  };

  /**
   * @brief Creates a shaped Entity under an optional parent.
   *
   * @param shape_type Primitive type.
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent node; @c nullptr uses the root.
   */
  OgreShape(Type shape_type, Ogre::SceneManager* scene_manager,
            Ogre::SceneNode* parent_node = nullptr);

  /** @brief Destroys entity, material, and nodes. */
  ~OgreShape();

  /**
   * @brief Returns the primitive type.
   * @return Shape @ref Type.
   */
  Type type() const { return type_; }

  /**
   * @brief Sets the offset node translation (shape origin adjustment).
   * @param offset Local offset.
   */
  void setOffset(const Ogre::Vector3& offset);

  /**
   * @brief Sets RGBA color on the shape material.
   */
  void setColor(float r, float g, float b, float a);

  /**
   * @brief Sets color from an Ogre colour value.
   * @param color RGBA colour.
   */
  void setColor(const Ogre::ColourValue& color);

  /**
   * @brief Sets the root node position.
   * @param position Translation.
   */
  void setPosition(const Ogre::Vector3& position);

  /**
   * @brief Sets the root node orientation.
   * @param orientation Rotation.
   */
  void setOrientation(const Ogre::Quaternion& orientation);

  /**
   * @brief Sets the root node scale.
   * @param scale Non-uniform scale.
   */
  void setScale(const Ogre::Vector3& scale);

  /** @brief Returns root node position. */
  const Ogre::Vector3& position() const;

  /** @brief Returns root node orientation. */
  const Ogre::Quaternion& orientation() const;

  /**
   * @brief Returns the root scene node.
   * @return Non-null root node.
   */
  Ogre::SceneNode* rootNode() { return scene_node_; }

  /**
   * @brief Returns the Ogre Entity.
   * @return Entity pointer.
   */
  Ogre::Entity* entity() { return entity_; }

  /**
   * @brief Returns the per-instance material.
   * @return Material pointer.
   */
  Ogre::MaterialPtr material() { return material_; }

  /**
   * @brief Attaches user data to the entity.
   * @param data Ogre Any payload.
   */
  void setUserData(const Ogre::Any& data);

  /**
   * @brief Factory: creates an Entity for @p shape_type without an @ref OgreShape.
   *
   * @param name Unique entity name.
   * @param shape_type Primitive type.
   * @param scene_manager Scene manager.
   * @return New Entity, or @c nullptr on failure.
   */
  static Ogre::Entity* createEntity(const std::string& name, Type shape_type,
                                    Ogre::SceneManager* scene_manager);

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::SceneNode* offset_node_ = nullptr;
  Ogre::Entity* entity_ = nullptr;
  Ogre::MaterialPtr material_;
  std::string material_name_;
  Type type_ = kCube;
};

}  // namespace rendering
}  // namespace autoviz

