#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace path_follow
{
    using namespace std::chrono_literals;
    struct publish_path : public rclcpp::Node
    {
        publish_path(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions()) : Node("path_publish_node", Options)
        {
            // this->declare_parameter<std::vector<double>>("point_A",point);
            // this->declare_parameter<std::vector<double>>("point_B",point);
            // this->declare_parameter<std::vector<double>>("point_C",point);
            // this->declare_parameter<std::vector<double>>("point_D",point);
            // this->declare_parameter<std::vector<double>>("point_E",point);
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            path_pub = this->create_publisher<nav_msgs::msg::Path>("global_plan", 10);

            for (std::string W : waypoints)
            {
                geometry_msgs::msg::TransformStamped t;
                geometry_msgs::msg::PoseStamped p;
                try
                {
                    t = tf_buffer_->lookupTransform("world", W, tf2::TimePointZero, 500ms);
                }
                catch (tf2::TransformException &e)
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger("path_publish"), "Could not transform %s to 'world': %s",
                        W.c_str(), e.what());
                };
                p.header.frame_id = "world";
                p.header.stamp = rclcpp::Clock().now();
                p.pose.orientation = t.transform.rotation;
                p.pose.position.x = t.transform.translation.x;
                p.pose.position.y = t.transform.translation.y;
                p.pose.position.z = t.transform.translation.z;
                path.header.frame_id = "world";
                path.poses.push_back(std::move(p));
            };
            timer = this->create_wall_timer(1s, [this]() -> void
                                            { path_pub->publish(path); });
        };

    private:
        std::vector<double> point{0,0,0,0,0,0};
        nav_msgs::msg::Path path;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        std::vector<std::string> waypoints{"A", "B", "C", "D", "E"};
    };



}

RCLCPP_COMPONENTS_REGISTER_NODE(path_follow::publish_path)