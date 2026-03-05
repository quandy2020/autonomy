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

 #include <atomic>
 #include <cmath>
 #include <csignal>
 #include <iomanip>
 #include <memory>
 #include <sstream>
 #include <thread>
 #include <vector>
 
 #include <opencv2/opencv.hpp>
 
 #include <Eigen/Dense>
 #include "autolink/common/log.hpp"
 #include "autonomy/common/macros.hpp"
 #include "autonomy/commsgs/builtin_interfaces.hpp"
 #include "autonomy/commsgs/geometry_msgs.hpp"
 #include "autonomy/commsgs/planning_msgs.hpp"
 #include "autonomy/control/controller/mppi_controller/controller.hpp"
 #include "autonomy/control/controller/mppi_controller/models/trajectories.hpp"
 #include "autonomy/control/controller/mppi_controller/optimizer.hpp"
 #include "autonomy/control/proto/mppi_controller.pb.h"
 #include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
 #include "autonomy/map/proto/map_2d_option.pb.h"
 
 using namespace autonomy;
 using namespace autonomy::control;
 using namespace autonomy::control::controller::mppi_controller;
 using namespace autonomy::map;
 using namespace autonomy::map::costmap_2d;
 
 static std::atomic<bool> g_stop_requested{false};
 
 static void SignalHandler(int /*signum*/) {
     g_stop_requested.store(true);
 }
 
 // Constants
 constexpr int WINDOW_WIDTH = 1200;
 constexpr int WINDOW_HEIGHT = 1200;
 constexpr double PIXELS_PER_METER = 20.0;
 constexpr double CONTROL_DT = 0.1;  // 10 Hz
 constexpr int MAX_STEPS = 1000;
 
 // Color definitions
 const cv::Scalar COLOR_PATH(0, 255, 0);            // Green
 const cv::Scalar COLOR_ROBOT(255, 0, 0);           // Red
 const cv::Scalar COLOR_CANDIDATES(100, 100, 255);  // Light blue
 const cv::Scalar COLOR_OPTIMAL(255, 0, 255);       // Magenta
 const cv::Scalar COLOR_VELOCITY(0, 255, 255);      // Cyan
 const cv::Scalar COLOR_OBSTACLE(0, 0, 255);        // Blue
 const cv::Scalar COLOR_COSTMAP(50, 50, 50);        // Dark gray
 const cv::Scalar COLOR_TEXT(255, 255, 255);        // White
 
 /**
  * @brief Test controller class to access optimizer for visualization
  */
 class TestMPPIController : public MPPIController {
 public:
     Optimizer& GetOptimizer() {
         return optimizer_;
     }
 };
 
 /**
  * @brief MPPI Controller application with OpenCV visualization and obstacle avoidance
  */
 class MPPIControllerApp
 {
 public:
     MPPIControllerApp() : width_(WINDOW_WIDTH), height_(WINDOW_HEIGHT), scale_(PIXELS_PER_METER) {
         canvas_ = cv::Mat::zeros(height_, width_, CV_8UC3);
         // In headless environments (e.g. inside Docker without DISPLAY), OpenCV highgui
         // backends like GTK are often unavailable, which would cause runtime exceptions
         // when creating windows. Detect this case and disable visualization gracefully.
         enable_visualization_ = (std::getenv("DISPLAY") != nullptr);
         if (enable_visualization_) {
             try {
                 cv::namedWindow("MPPI Controller - Obstacle Avoidance", cv::WINDOW_AUTOSIZE);
             } catch (const cv::Exception& e) {
                 enable_visualization_ = false;
                 AWARN << "OpenCV highgui not available, disable window display. what(): " << e.what();
             }
         }
         if (!enable_visualization_) {
             AINFO << "Headless mode: OpenCV window disabled (Ctrl+C to quit)";
         }
     }
 
     ~MPPIControllerApp() {
         if (controller_) {
             controller_->Deactivate();
             controller_->Cleanup();
         }
         if (enable_visualization_) {
             cv::destroyWindow("MPPI Controller - Obstacle Avoidance");
         }
     }
 
     void Clear() {
         canvas_ = cv::Mat::zeros(height_, width_, CV_8UC3);
         DrawGrid();
         DrawAxes();
     }
 
     void DrawCostmap(std::shared_ptr<Costmap2DWrapper> costmap) {
         if (!costmap) return;
 
         // Get costmap data
         auto costmap_2d = costmap->getCostmap();
         if (!costmap_2d) return;
 
         unsigned int size_x = costmap_2d->getSizeInCellsX();
         unsigned int size_y = costmap_2d->getSizeInCellsY();
         double resolution = costmap_2d->getResolution();
         double origin_x = costmap_2d->getOriginX();
         double origin_y = costmap_2d->getOriginY();
 
         // Draw costmap cells
         for (unsigned int i = 0; i < size_x; ++i) {
             for (unsigned int j = 0; j < size_y; ++j) {
                 unsigned char cost = costmap_2d->getCost(i, j);
                 if (cost > 0) {
                     double world_x = origin_x + (i + 0.5) * resolution;
                     double world_y = origin_y + (j + 0.5) * resolution;
                     cv::Point pt = ToImage(world_x, world_y);
                     
                     // Color based on cost value
                     cv::Scalar color;
                     if (cost >= 254) {  // Lethal obstacle
                         color = COLOR_OBSTACLE;
                     } else if (cost >= 200) {  // Inscribed obstacle
                         color = COLOR_COSTMAP;
                     } else {  // Other costs
                         int intensity = static_cast<int>(cost * 0.5);
                         color = cv::Scalar(intensity, intensity, intensity);
                     }
                     
                     cv::circle(canvas_, pt, static_cast<int>(resolution * scale_ / 2), color, -1);
                 }
             }
         }
     }
 
     void DrawPath(const commsgs::planning_msgs::Path& path) {
         if (path.poses.empty()) return;
 
         for (size_t i = 0; i < path.poses.size() - 1; ++i) {
             const auto& p1 = path.poses[i].pose.position;
             const auto& p2 = path.poses[i + 1].pose.position;
             cv::line(canvas_, ToImage(p1.x, p1.y), ToImage(p2.x, p2.y), COLOR_PATH, 3);
         }
     }
 
     void DrawCandidateTrajectories(const models::Trajectories& trajectories) {
         if (trajectories.x.rows() == 0 || trajectories.x.cols() == 0) return;
 
         for (int i = 0; i < trajectories.x.rows(); ++i) {
             std::vector<cv::Point> points;
             for (int j = 0; j < trajectories.x.cols(); ++j) {
                 points.push_back(ToImage(trajectories.x(i, j), trajectories.y(i, j)));
             }
             DrawPolyline(points, COLOR_CANDIDATES, 1);
         }
     }
 
     void DrawOptimalTrajectory(const Eigen::ArrayXXf& trajectory) {
         if (trajectory.rows() == 0 || trajectory.cols() < 2) return;
 
         std::vector<cv::Point> points;
         for (int i = 0; i < trajectory.rows(); ++i) {
             points.push_back(ToImage(trajectory(i, 0), trajectory(i, 1)));
         }
         DrawPolyline(points, COLOR_OPTIMAL, 3);
     }
 
     void DrawRobot(const commsgs::geometry_msgs::PoseStamped& pose) {
         const auto& pos = pose.pose.position;
         const auto& orient = pose.pose.orientation;
         double yaw = 2.0 * std::atan2(orient.z, orient.w);
 
         cv::Point center = ToImage(pos.x, pos.y);
         cv::circle(canvas_, center, 10, COLOR_ROBOT, -1);
         cv::circle(canvas_, center, 10, COLOR_TEXT, 2);
 
         cv::Point arrow_end(center.x + static_cast<int>(20 * std::cos(yaw)),
                             center.y + static_cast<int>(20 * std::sin(yaw)));
         cv::arrowedLine(canvas_, center, arrow_end, COLOR_TEXT, 3, 8, 0, 0.3);
     }
 
     void DrawVelocity(const commsgs::geometry_msgs::TwistStamped& cmd_vel,
                       const commsgs::geometry_msgs::PoseStamped& robot_pose) {
         cv::Point center = ToImage(robot_pose.pose.position.x, robot_pose.pose.position.y);
         double vx = cmd_vel.twist.linear.x;
         double vy = cmd_vel.twist.linear.y;
 
         if (std::abs(vx) > 0.01 || std::abs(vy) > 0.01) {
             cv::Point vel_end(center.x + static_cast<int>(vx * scale_ * 3),
                               center.y + static_cast<int>(vy * scale_ * 3));
             cv::arrowedLine(canvas_, center, vel_end, COLOR_VELOCITY, 2);
         }
     }
 
     void AddText(const std::string& text, int x, int y, const cv::Scalar& color = COLOR_TEXT) {
         cv::putText(canvas_, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
     }
 
     void Update() {
         if (enable_visualization_) {
             cv::imshow("MPPI Controller - Obstacle Avoidance", canvas_);
             cv::waitKey(1);
         }
     }
 
     /**
      * @brief Create test path with obstacles
      */
     commsgs::planning_msgs::Path CreatePath(const std::string& type) {
         commsgs::planning_msgs::Path path;
         path.header.frame_id = "map";
         path.header.stamp = commsgs::builtin_interfaces::Time::Now();
 
         auto AddPose = [&](double x, double y, double yaw) {
             commsgs::geometry_msgs::PoseStamped pose;
             pose.header = path.header;
             pose.pose.position.x = x;
             pose.pose.position.y = y;
             pose.pose.position.z = 0.0;
             pose.pose.orientation.w = std::cos(yaw / 2.0);
             pose.pose.orientation.z = std::sin(yaw / 2.0);
             path.poses.push_back(pose);
         };
 
         if (type == "obstacle_test") {
             // Path that requires obstacle avoidance
             AddPose(0.0, 0.0, 0.0);
             AddPose(2.0, 0.0, 0.0);
             AddPose(4.0, 2.0, M_PI / 4);
             AddPose(6.0, 4.0, M_PI / 2);
             AddPose(8.0, 4.0, 0.0);
         } else if (type == "circular") {
             constexpr double radius = 4.0;
             constexpr int num_points = 50;
             for (int i = 0; i <= num_points; ++i) {
                 double angle = 2.0 * M_PI * i / num_points;
                 AddPose(radius * std::cos(angle), radius * std::sin(angle), angle + M_PI / 2.0);
             }
         } else if (type == "straight") {
             constexpr double length = 8.0;
             constexpr int num_points = 20;
             for (int i = 0; i <= num_points; ++i) {
                 AddPose(length * i / num_points, 0.0, 0.0);
             }
         } else if (type == "s_curve") {
             constexpr int num_points = 30;
             for (int i = 0; i <= num_points; ++i) {
                 double t = static_cast<double>(i) / num_points;
                 double yaw = std::atan2(2.0 * 2.0 * M_PI * std::cos(2.0 * M_PI * t), 5.0);
                 AddPose(5.0 * t, 2.0 * std::sin(2.0 * M_PI * t), yaw);
             }
         }
 
         return path;
     }
 
     /**
      * @brief Create costmap with obstacles
      */
     std::shared_ptr<Costmap2DWrapper> CreateCostmap() {
         autonomy::map::proto::Costmap2DOptions options;
         options.set_enabled(true);
         options.set_frame_id("map");
         options.set_resolution(0.05);  // 5cm resolution
         options.set_robot_radius(0.2);
         options.set_footprint_padding(0.1);
         options.set_update_frequency(10.0);
         options.set_rolling_window(false);
 
         // Set costmap size (20m x 20m)
         options.set_width(400);   // 20m / 0.05m = 400 cells
         options.set_height(400);
 
         auto costmap = std::make_shared<Costmap2DWrapper>(options, "test_costmap", nullptr);
         costmap->Start();
 
         // Add obstacles manually to the costmap
         auto costmap_2d = costmap->getCostmap();
         if (costmap_2d) {
             // Add some circular obstacles
             std::vector<std::pair<double, double>> obstacles = {
                 {3.0, 1.0},   // Obstacle in the path
                 {5.0, 3.0},   // Another obstacle
                 {7.0, 4.5},   // Obstacle near goal
             };
 
             for (const auto& obs : obstacles) {
                 unsigned int mx, my;
                 if (costmap_2d->worldToMap(obs.first, obs.second, mx, my)) {
                     // Mark a circular region as obstacle
                     int radius_cells = static_cast<int>(0.3 / costmap_2d->getResolution());  // 30cm radius
                     for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                         for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                             if (dx * dx + dy * dy <= radius_cells * radius_cells) {
                                 unsigned int cx = mx + dx;
                                 unsigned int cy = my + dy;
                                 if (cx < costmap_2d->getSizeInCellsX() && cy < costmap_2d->getSizeInCellsY()) {
                                     costmap_2d->setCost(cx, cy, 254);  // Lethal cost
                                 }
                             }
                         }
                     }
                 }
             }
         }
 
         return costmap;
     }
 
     /**
      * @brief Update robot pose using kinematic model
      */
     void UpdatePose(commsgs::geometry_msgs::PoseStamped& pose, const commsgs::geometry_msgs::TwistStamped& cmd_vel,
                     double dt) {
         double vx = cmd_vel.twist.linear.x;
         double vy = cmd_vel.twist.linear.y;
         double omega = cmd_vel.twist.angular.z;
 
         const auto& orient = pose.pose.orientation;
         double yaw = 2.0 * std::atan2(orient.z, orient.w);
 
         pose.pose.position.x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
         pose.pose.position.y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
         yaw += omega * dt;
 
         pose.pose.orientation.w = std::cos(yaw / 2.0);
         pose.pose.orientation.z = std::sin(yaw / 2.0);
     }
 
     /**
      * @brief Initialize and configure controller
      */
     bool Initialize() {
         controller_ = std::make_unique<TestMPPIController>();
 
         // Create costmap
         costmap_ = CreateCostmap();
 
         // Configure controller
         autonomy::control::proto::ControllerOptions options;
         try {
             controller_->Configure(options, "mppi_app", nullptr, costmap_);
             controller_->Activate();
             return true;
         } catch (const std::exception& e) {
             AERROR << "Failed to configure controller: " << e.what();
             return false;
         }
     }
 
     /**
      * @brief Run simulation
      */
     void Run(const std::string& path_type) {
         if (!controller_) {
             AERROR << "Controller not initialized";
             return;
         }
 
         // Setup path
         auto path = CreatePath(path_type);
         controller_->SetPlan(path);
         AINFO << "Path type: " << path_type << ", Points: " << path.poses.size();
 
         // Initialize robot state
         commsgs::geometry_msgs::PoseStamped robot_pose = path.poses[0];
         commsgs::geometry_msgs::TwistStamped robot_velocity;
         robot_velocity.header.frame_id = "base_link";
         robot_velocity.header.stamp = commsgs::builtin_interfaces::Time::Now();
         robot_velocity.twist.linear.x = 0.0;
         robot_velocity.twist.linear.y = 0.0;
         robot_velocity.twist.angular.z = 0.0;
 
         AINFO << "Starting simulation (Press 'q' to quit, 'r' to reset, SPACE to pause)";
 
         // Main simulation loop
         bool paused = false;
         for (int step = 0; step < MAX_STEPS && !g_stop_requested.load(); ++step) {
             Clear();
             
             // Draw costmap and obstacles
             DrawCostmap(costmap_);
             DrawPath(path);
 
             // Compute control
             commsgs::geometry_msgs::TwistStamped cmd_vel;
             std::string message;
             robot_pose.header.stamp = commsgs::builtin_interfaces::Time::Now();
             controller_->ComputeVelocityCommands(robot_pose, robot_velocity, cmd_vel, nullptr, message);
 
             // Get and draw trajectories
             auto& optimizer = controller_->GetOptimizer();
             DrawCandidateTrajectories(optimizer.getGeneratedTrajectories());
             DrawOptimalTrajectory(optimizer.getOptimizedTrajectory());
 
             // Draw robot and velocity
             DrawRobot(robot_pose);
             DrawVelocity(cmd_vel, robot_pose);
 
             // Update robot state
             if (!paused) {
                 UpdatePose(robot_pose, cmd_vel, CONTROL_DT);
                 robot_velocity.twist = cmd_vel.twist;
             }
 
             // Display info
             std::stringstream info;
             info << "Step: " << step << " | v: " << std::fixed << std::setprecision(2) << cmd_vel.twist.linear.x
                  << " m/s | omega: " << cmd_vel.twist.angular.z << " rad/s";
             AddText(info.str(), 10, 30);
 
             std::stringstream pos;
             pos << "Position: (" << std::fixed << std::setprecision(2) << robot_pose.pose.position.x << ", "
                 << robot_pose.pose.position.y << ")";
             AddText(pos.str(), 10, 60);
 
             const auto& trajectories = optimizer.getGeneratedTrajectories();
             std::stringstream traj;
             traj << "Candidates: " << trajectories.x.rows() << " | Steps: " << trajectories.x.cols();
             AddText(traj.str(), 10, 90);
 
             if (paused) {
                 AddText("PAUSED - Press SPACE to resume", 10, 120, cv::Scalar(0, 255, 255));
             }
 
             // Legend
             int y_offset = height_ - 100;
             AddText("Green: Reference Path", 10, y_offset, COLOR_PATH);
             AddText("Light Blue: Candidate Trajectories", 10, y_offset + 25, COLOR_CANDIDATES);
             AddText("Magenta: Optimal Trajectory", 10, y_offset + 50, COLOR_OPTIMAL);
             AddText("Red: Robot | Blue: Obstacles", 10, y_offset + 75, COLOR_TEXT);
 
             Update();
 
             // Handle input
             if (enable_visualization_) {
                 int key = cv::waitKey(static_cast<int>(CONTROL_DT * 1000));
                 if (key == 'q' || key == 'Q') {
                     AINFO << "Quitting...";
                     break;
                 } else if (key == 'r' || key == 'R') {
                     AINFO << "Resetting...";
                     robot_pose = path.poses[0];
                     robot_velocity.twist.linear.x = 0.0;
                     robot_velocity.twist.linear.y = 0.0;
                     robot_velocity.twist.angular.z = 0.0;
                     controller_->Reset();
                 } else if (key == ' ') {
                     paused = !paused;
                     AINFO << (paused ? "Paused" : "Resumed");
                 }
             } else {
                 std::this_thread::sleep_for(std::chrono::duration<double>(CONTROL_DT));
             }
 
             // Check if goal reached
             if (controller_->IsGoalReached(0.2, 0.2)) {
                 AINFO << "Goal reached!";
                 AddText("GOAL REACHED!", width_ / 2 - 150, height_ / 2, cv::Scalar(0, 255, 0));
                 Update();
                 if (enable_visualization_) {
                     cv::waitKey(2000);
                 } else {
                     std::this_thread::sleep_for(std::chrono::seconds(2));
                 }
                 break;
             }
         }
 
         AINFO << "Simulation completed.";
     }
 
 
 private:
     cv::Point ToImage(double x, double y) {
         return cv::Point(static_cast<int>(width_ / 2 + x * scale_), static_cast<int>(height_ / 2 - y * scale_));
     }
 
     void DrawPolyline(const std::vector<cv::Point>& points, const cv::Scalar& color, int thickness) {
         if (points.size() < 2) return;
         for (size_t i = 0; i < points.size() - 1; ++i) {
             cv::line(canvas_, points[i], points[i + 1], color, thickness);
         }
     }
 
     void DrawGrid() {
         cv::Scalar grid_color(40, 40, 40);
         int spacing = static_cast<int>(scale_);
         cv::Point origin = ToImage(0, 0);
 
         for (int x = origin.x % spacing; x < width_; x += spacing) {
             cv::line(canvas_, cv::Point(x, 0), cv::Point(x, height_), grid_color, 1);
         }
         for (int y = origin.y % spacing; y < height_; y += spacing) {
             cv::line(canvas_, cv::Point(0, y), cv::Point(width_, y), grid_color, 1);
         }
     }
 
     void DrawAxes() {
         cv::Point origin = ToImage(0, 0);
         cv::Scalar axis_color(80, 80, 80);
         cv::line(canvas_, cv::Point(origin.x, 0), cv::Point(origin.x, height_), axis_color, 2);
         cv::line(canvas_, cv::Point(0, origin.y), cv::Point(width_, origin.y), axis_color, 2);
     }
 
     cv::Mat canvas_;
     int width_;
     int height_;
     double scale_;
     bool enable_visualization_ = false;
     std::unique_ptr<TestMPPIController> controller_;
     std::shared_ptr<Costmap2DWrapper> costmap_;
 };
 
 int main(int argc, char** argv) {
     std::signal(SIGINT, SignalHandler);
     std::signal(SIGTERM, SignalHandler);

     AINFO << "MPPI Controller Application with Obstacle Avoidance";
     AINFO << "===================================================";
 
     MPPIControllerApp app;
 
     if (!app.Initialize()) {
         AERROR << "Failed to initialize application";
         return 1;
     }
 
     std::string path_type = (argc > 1) ? argv[1] : "obstacle_test";
     AINFO << "Available path types: obstacle_test, circular, straight, s_curve";
     
     app.Run(path_type);
 
     return 0;
 }
 
 