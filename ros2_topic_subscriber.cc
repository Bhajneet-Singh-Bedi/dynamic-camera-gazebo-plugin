/*
 * Copyright (C) 2025 xAI
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #include <ignition/gazebo/System.hh>
 #include <ignition/gazebo/components/Pose.hh>
 #include <ignition/plugin/Register.hh>
 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/pose.hpp>
 #include <thread>
 
 namespace ignition
 {
 namespace gazebo
 {
 namespace systems
 {
 class Ros2TopicSubscriber
     : public System,
       public ISystemConfigure,
       public ISystemPreUpdate
 {
 public:
     Ros2TopicSubscriber() = default;
     ~Ros2TopicSubscriber() override
     {
         if (ros_thread_.joinable())
         {
             rclcpp::shutdown();
             ros_thread_.join();
         }
     }
 
     void Configure(
         const Entity& entity,
         const std::shared_ptr<const sdf::Element>& sdf,
         EntityComponentManager& ecm,
         EventManager& eventMgr) override
     {
         if (!rclcpp::ok())
         {
             ignmsg << "Initializing ROS 2 context" << std::endl;
             rclcpp::init(0, nullptr);
         }
 
         ros_node_ = std::make_shared<rclcpp::Node>("ros2_topic_subscriber");
 
         // Get topic name from SDF, default to "/pose"
         std::string topic = "/pose";
         if (sdf->HasElement("topic"))
         {
             topic = sdf->Get<std::string>("topic");
         }
 
         // Store the camera entity
         camera_entity_ = entity;
 
         // Subscribe to Pose messages
         subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Pose>(
             topic, 10,
             [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                 this->OnPoseMsg(msg);
             });
 
         ignmsg << "Successfully subscribed to ROS 2 topic: " << topic 
                << " for camera entity [" << camera_entity_ << "]" << std::endl;
 
         // Start ROS spinning in a separate thread
         ros_thread_ = std::thread([this]() {
             rclcpp::spin(ros_node_);
         });
     }
 
     void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
     {
         if (!new_pose_received_) return;
 
         // Convert ROS 2 Pose to Ignition Pose3d
         ignition::math::Pose3d new_pose(
             target_pose_.position.x,
             target_pose_.position.y,
             target_pose_.position.z,
             target_pose_.orientation.w,  // w first for Ignition quaternion
             target_pose_.orientation.x,
             target_pose_.orientation.y,
             target_pose_.orientation.z
         );
 
         // Update or create the Pose component
         auto pose_comp = ecm.Component<components::Pose>(camera_entity_);
         if (pose_comp)
         {
             *pose_comp = components::Pose(new_pose);
             // Mark as a one-time change to ensure it’s applied
             ecm.SetChanged(camera_entity_, components::Pose::typeId,
                            ComponentState::OneTimeChange);
         }
         else
         {
             ecm.CreateComponent(camera_entity_, components::Pose(new_pose));
         }
 
         ignmsg << "Updated camera pose to: "
                << "Position (x: " << new_pose.Pos().X()
                << ", y: " << new_pose.Pos().Y()
                << ", z: " << new_pose.Pos().Z() << "), "
                << "Orientation (qw: " << new_pose.Rot().W()
                << ", qx: " << new_pose.Rot().X()
                << ", qy: " << new_pose.Rot().Y()
                << ", qz: " << new_pose.Rot().Z() << ")" << std::endl;
 
         new_pose_received_ = false; // Reset flag after applying
     }
 
 private:
     void OnPoseMsg(const geometry_msgs::msg::Pose::SharedPtr msg)
     {
         // Store the received pose
         target_pose_ = *msg;
         new_pose_received_ = true;
 
         ignmsg << "Received ROS 2 Pose message: "
                << "Position (x: " << msg->position.x
                << ", y: " << msg->position.y
                << ", z: " << msg->position.z << "), "
                << "Orientation (x: " << msg->orientation.x
                << ", y: " << msg->orientation.y
                << ", z: " << msg->orientation.z
                << ", w: " << msg->orientation.w << ")" << std::endl;
     }
 
     std::shared_ptr<rclcpp::Node> ros_node_;
     rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
     std::thread ros_thread_;
     Entity camera_entity_ = kNullEntity; // Camera model entity
     geometry_msgs::msg::Pose target_pose_; // Latest received pose
     bool new_pose_received_ = false; // Flag for new pose
 };
 }
 }
 }
 
 IGNITION_ADD_PLUGIN(
     ignition::gazebo::systems::Ros2TopicSubscriber,
     ignition::gazebo::System,
     ignition::gazebo::systems::Ros2TopicSubscriber::ISystemConfigure,
     ignition::gazebo::systems::Ros2TopicSubscriber::ISystemPreUpdate)
 
 IGNITION_ADD_PLUGIN_ALIAS(
     ignition::gazebo::systems::Ros2TopicSubscriber,
     "ros2_topic_subscriber")