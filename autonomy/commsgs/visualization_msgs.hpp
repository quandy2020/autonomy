/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once 


#include "autonomy/common/port.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace visualization_msgs {

struct MenuEntry
{
    // MenuEntry message.
    //
    // Each InteractiveMarker message has an array of MenuEntry messages.
    // A collection of MenuEntries together describe a
    // menu/submenu/subsubmenu/etc tree, though they are stored in a flat
    // array.  The tree structure is represented by giving each menu entry
    // an ID number and a "parent_id" field.  Top-level entries are the
    // ones with parent_id = 0.  Menu entries are ordered within their
    // level the same way they are ordered in the containing array.  Parent
    // entries must appear before their children.
    //
    // Example:
    // - id = 3
    //   parent_id = 0
    //   title = "fun"
    // - id = 2
    //   parent_id = 0
    //   title = "robot"
    // - id = 4
    //   parent_id = 2
    //   title = "pr2"
    // - id = 5
    //   parent_id = 2
    //   title = "turtle"
    //
    // Gives a menu tree like this:
    //  - fun
    //  - robot
    //    - pr2
    //    - turtle

    // ID is a number for each menu entry.  Must be unique within the
    // control, and should never be 0.
    uint32 id;

    // ID of the parent of this menu entry, if it is a submenu.  If this
    // menu entry is a top-level entry, set parent_id to 0.
    uint32 parent_id;

    // menu / entry title
    std::string title;

    // Arguments to command indicated by command_type (below)
    std::string command;

    // // Command_type stores the type of response desired when this menu
    // // entry is clicked.
    // // FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
    // // ROSRUN: execute "rosrun" with arguments given in the command field (above).
    // // ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
    // uint8 FEEDBACK=0
    // uint8 ROSRUN=1
    // uint8 ROSLAUNCH=2
    // uint8 command_type
};

struct MeshFile
{
    // Used to send raw mesh files.

    // The filename is used for both debug purposes and to provide a file extension
    // for whatever parser is used.
    std::string filename;

    // This stores the raw text of the mesh file.
    std::vector<uint8> data;
};

struct UVCoordinate
{
    // Location of the pixel as a ratio of the width of a 2D texture.
    // Values should be in range: [0.0-1.0].
    float u;
    float v;
};

struct ImageMarker
{
    // int32 CIRCLE=0
    // int32 LINE_STRIP=1
    // int32 LINE_LIST=2
    // int32 POLYGON=3
    // int32 POINTS=4

    // int32 ADD=0
    // int32 REMOVE=1

    std_msgs::Header header;

    // Namespace which is used with the id to form a unique id.
    std::string ns;

    // Unique id within the namespace.
    int32 id;

    // One of the above types, e.g. CIRCLE, LINE_STRIP, etc.
    int32 type;

    // Either ADD or REMOVE.
    int32 action;

    // Two-dimensional coordinate position, in pixel-coordinates.
    geometry_msgs::Point position;

    // The scale of the object, e.g. the diameter for a CIRCLE.
    float scale;

    // The outline color of the marker.
    std_msgs::ColorRGBA outline_color;

    // Whether or not to fill in the shape with color.
    uint8 filled;

    // Fill color; in the range: [0.0-1.0]
    std_msgs::ColorRGBA fill_color;

    // How long the object should last before being automatically deleted.
    // 0 indicates forever.
    builtin_interfaces::Duration lifetime;

    // Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.
    std::vector<geometry_msgs::Point> points;

    // The color for each line, point, etc. in the points field.
    std::vector<std_msgs::ColorRGBA> outline_colors;
};

struct Marker
{
    // See:
    //  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker
    //  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes
    //
    // for more information on using this message with rviz.

    // int32 ARROW=0
    // int32 CUBE=1
    // int32 SPHERE=2
    // int32 CYLINDER=3
    // int32 LINE_STRIP=4
    // int32 LINE_LIST=5
    // int32 CUBE_LIST=6
    // int32 SPHERE_LIST=7
    // int32 POINTS=8
    // int32 TEXT_VIEW_FACING=9
    // int32 MESH_RESOURCE=10
    // int32 TRIANGLE_LIST=11
    // int32 ARROW_STRIP=12

    // int32 ADD=0
    // int32 MODIFY=0
    // int32 DELETE=2
    // int32 DELETEALL=3

    // Header for timestamp and frame id.
    std_msgs::Header header;

    // Namespace in which to place the object.
    // Used in conjunction with id to create a unique name for the object.
    std::string ns;
    
    // Object ID used in conjunction with the namespace for manipulating and deleting the object later.
    int32 id;

    // Type of object.
    int32 type;

    // Action to take; one of:
    //  - 0 add/modify an object
    //  - 1 (deprecated)
    //  - 2 deletes an object (with the given ns and id)
    //  - 3 deletes all objects (or those with the given ns if any)
    int32 action;

    // Pose of the object with respect the frame_id specified in the header.
    geometry_msgs::Pose pose;

    // Scale of the object; 1,1,1 means default (usually 1 meter square).
    geometry_msgs::Vector3 scale;

    // Color of the object; in the range: [0.0-1.0]
    std_msgs::ColorRGBA color;

    // How long the object should last before being automatically deleted.
    // 0 indicates forever.
    builtin_interfaces::Duration lifetime;

    // If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
    bool frame_locked;

    // Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)
    std::vector<geometry_msgs::Point> points;

    // Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)
    // The number of colors provided must either be 0 or equal to the number of points provided.
    // NOTE: alpha is not yet used
    std::vector<std_msgs::ColorRGBA> colors;

    // Texture resource is a special URI that can either reference a texture file in
    // a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]
    // or an embedded texture via a string matching the format:
    //   "embedded://texture_name"
    std::string texture_resource;

    // An image to be loaded into the rendering engine as the texture for this marker.
    // This will be used iff texture_resource is set to embedded.
    sensor_msgs::CompressedImage texture;

    // Location of each vertex within the texture; in the range: [0.0-1.0]
    std::vector<UVCoordinate> uv_coordinates;

    // Only used for text markers
    std::string text;

    // Only used for MESH_RESOURCE markers.
    // Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.
    // Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,
    // use the following format for mesh_resource:
    //   "embedded://mesh_name"
    std::string mesh_resource;
    MeshFile mesh_file;
    bool mesh_use_embedded_materials;
};

// Represents a control that is to be displayed together with an interactive marker
struct InteractiveMarkerControl
{

    // Identifying string for this control.
    // You need to assign a unique value to this to receive feedback from the GUI
    // on what actions the user performs on this control (e.g. a button click).
    std::string name;


    // Defines the local coordinate frame (relative to the pose of the parent
    // interactive marker) in which is being rotated and translated.
    // Default: Identity
    geometry_msgs::Quaternion orientation;


    // // Orientation mode: controls how orientation changes.
    // // INHERIT: Follow orientation of interactive marker
    // // FIXED: Keep orientation fixed at initial state
    // // VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).
    // uint8 INHERIT = 0
    // uint8 FIXED = 1
    // uint8 VIEW_FACING = 2

    uint8 orientation_mode;

    // // Interaction mode for this control
    // //
    // // NONE: This control is only meant for visualization; no context menu.
    // // MENU: Like NONE, but right-click menu is active.
    // // BUTTON: Element can be left-clicked.
    // // MOVE_AXIS: Translate along local x-axis.
    // // MOVE_PLANE: Translate in local y-z plane.
    // // ROTATE_AXIS: Rotate around local x-axis.
    // // MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
    // uint8 NONE = 0
    // uint8 MENU = 1
    // uint8 BUTTON = 2
    // uint8 MOVE_AXIS = 3
    // uint8 MOVE_PLANE = 4
    // uint8 ROTATE_AXIS = 5
    // uint8 MOVE_ROTATE = 6
    // // "3D" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.
    // // MOVE_3D: Translate freely in 3D space.
    // // ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
    // // MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.
    // uint8 MOVE_3D = 7
    // uint8 ROTATE_3D = 8
    // uint8 MOVE_ROTATE_3D = 9

    uint8 interaction_mode;

    // If true, the contained markers will also be visible
    // when the gui is not in interactive mode.
    bool always_visible;


    // Markers to be displayed as custom visual representation.
    // Leave this empty to use the default control handles.
    //
    // Note:
    // - The markers can be defined in an arbitrary coordinate frame,
    //   but will be transformed into the local frame of the interactive marker.
    // - If the header of a marker is empty, its pose will be interpreted as
    //   relative to the pose of the parent interactive marker.
    std::vector<Marker> markers;

    // In VIEW_FACING mode, set this to true if you don't want the markers
    // to be aligned with the camera view point. The markers will show up
    // as in INHERIT mode.
    bool independent_marker_orientation;


    // Short description (< 40 characters) of what this control does,
    // e.g. "Move the robot".
    // Default: A generic description based on the interaction mode
    std::string description;
};

struct InteractiveMarkerFeedback
{
    // Time/frame info.
    std_msgs::Header header;

    // Identifying string. Must be unique in the topic namespace.
    std::string client_id;

    // Feedback message sent back from the GUI, e.g.
    // when the status of an interactive marker was modified by the user.

    // Specifies which interactive marker and control this message refers to
    std::string marker_name;
    std::string control_name;

    // // Type of the event
    // // KEEP_ALIVE: sent while dragging to keep up control of the marker
    // // MENU_SELECT: a menu entry has been selected
    // // BUTTON_CLICK: a button control has been clicked
    // // POSE_UPDATE: the pose has been changed using one of the controls
    // uint8 KEEP_ALIVE = 0
    // uint8 POSE_UPDATE = 1
    // uint8 MENU_SELECT = 2
    // uint8 BUTTON_CLICK = 3

    // uint8 MOUSE_DOWN = 4
    // uint8 MOUSE_UP = 5

    uint8 event_type;

    // Current pose of the marker
    // Note: Has to be valid for all feedback types.
    geometry_msgs::Pose pose;

    // Contains the ID of the selected menu entry
    // Only valid for MENU_SELECT events.
    uint32 menu_entry_id;

    // If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point
    // may contain the 3 dimensional position of the event on the
    // control.  If it does, mouse_point_valid will be true.  mouse_point
    // will be relative to the frame listed in the header.
    geometry_msgs::Point mouse_point;
    bool mouse_point_valid;
};

struct InteractiveMarker
{
    // Time/frame info.
    // If header.time is set to 0, the marker will be retransformed into
    // its frame on each timestep. You will receive the pose feedback
    // in the same frame.
    // Otherwise, you might receive feedback in a different frame.
    // For rviz, this will be the current 'fixed frame' set by the user.
    std_msgs::Header header;

    // Initial pose. Also, defines the pivot point for rotations.
    geometry_msgs::Pose pose;

    // Identifying string. Must be globally unique in
    // the topic that this message is sent through.
    std::string name;

    // Short description (< 40 characters).
    std::string description;

    // Scale to be used for default controls (default=1).
    float scale;

    // All menu and submenu entries associated with this marker.
    std::vector<MenuEntry> menu_entries;

    // List of controls displayed for this marker.
    std::vector<InteractiveMarkerControl> controls;
};

struct InteractiveMarkerInit
{
    // Identifying string. Must be unique in the topic namespace
    // that this server works on.
    std::string server_id;

    // Sequence number.
    // The client will use this to detect if it has missed a subsequent
    // update.  Every update message will have the same sequence number as
    // an init message.  Clients will likely want to unsubscribe from the
    // init topic after a successful initialization to avoid receiving
    // duplicate data.
    uint64 seq_num;

    // All markers.
    std::vector<InteractiveMarker> markers;
};

struct InteractiveMarkerPose
{
    // Time/frame info.
    std_msgs::Header header;

    // Initial pose. Also, defines the pivot point for rotations.
    geometry_msgs::Pose pose;

    // Identifying string. Must be globally unique in
    // the topic that this message is sent through.
    std::string name;
};

struct InteractiveMarkerUpdate
{
    // Identifying string. Must be unique in the topic namespace
    // that this server works on.
    std::string server_id;

    // Sequence number.
    // The client will use this to detect if it has missed an update.
    uint64 seq_num;

    // Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.
    // UPDATE: Incremental update to previous state.
    //         The sequence number must be 1 higher than for
    //         the previous update.
    // KEEP_ALIVE: Indicates the that the server is still living.
    //             The sequence number does not increase.
    //             No payload data should be filled out (markers, poses, or erases).
    // uint8 KEEP_ALIVE = 0
    // uint8 UPDATE = 1

    uint8 type;

    // Note: No guarantees on the order of processing.
    //       Contents must be kept consistent by sender.

    // Markers to be added or updated
    std::vector<InteractiveMarker> markers;

    // Poses of markers that should be moved
    std::vector<InteractiveMarkerPose> poses;

    // Names of markers to be erased
    std::vector<std::string> erases;
};

struct MarkerArray
{
    std::vector<Marker> markers;
};


}  // namespace visualization_msgs
}  // namespace commsgs
}  // namespace autonomy