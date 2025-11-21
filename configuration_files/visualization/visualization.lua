-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.


AUTONOMY_VISUALIZATION = {
    -- Server configuration
    host = "0.0.0.0",
    port = 8765,
    write_mcap = false,
    mcap_filename = "/workspace/autonomy/build/test.mcap",
    
    -- Topic subscriptions configuration
    -- Each subscription contains: topic_name and message_type
    -- Note: Only fully supported message types with DDS + Foxglove handlers are included
    subscriptions = {
        -- Map data
        {
            topic_name = "/map",
            message_type = "OccupancyGrid",
        },
        
        -- Path planning result
        {
            topic_name = "/path",
            message_type = "Path",
        },
        
        -- Camera image
        {
            topic_name = "/camera/image",
            message_type = "Image",
        },
        
        -- Pose
        {
            topic_name = "/robot_pose",
            message_type = "Pose",
        },
        
        -- Point
        {
            topic_name = "/waypoint",
            message_type = "Point",
        },
        
        -- Polygon
        {
            topic_name = "/obstacle_polygon",
            message_type = "PolygonStamped",
        },
        
        
        
        
        -- Note: The following message types are currently NOT fully supported:
        -- 
        -- DDS support but missing Foxglove handler:
        -- - Odometry (PoseHandler only accepts Pose, not Odometry)
        -- - PoseStamped (PoseHandler only accepts Pose, not PoseStamped)  
        -- - PoseWithCovarianceStamped (PoseHandler only accepts Pose)
        --
        -- Missing DDS FromDDS converter implementation:
        -- - LaserScan (has MessageTrait but missing FromDDS implementation)
        -- - PointCloud2 (missing FromDDS converter)
        --
        -- Other issues:
        -- - Twist (needs custom handler)
        -- - JointTrajectory (commented out in type_support.hpp)
        -- - Marker/MarkerArray (visualization_msgs not defined in DDS)
    },
}