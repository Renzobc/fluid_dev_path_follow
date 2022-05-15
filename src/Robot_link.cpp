#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <iterator>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <math.h>

#include <rclcpp_components/register_node_macro.hpp>

const double PI = 3.141592653589793238463;
const double TOLL = 0.25;
const double VEL = 0.5;
const double YAW_GAIN = 2.5;
const double PITCH_GAIN = 2.5;
const double DELTA_X_ERROR_GAIN = 0.1;
const double DELTA_Y_ERROR_GAIN = 0.75;
const double DELTA_Z_ERROR_GAIN = 0.75;

// GET PATH AND MOVE THE DYNAMIC TRANSFORM (ROBOT LINK) FROM TF TO TF OF THE PATH

namespace path_follow
{

    using namespace std::placeholders;
    using namespace std::chrono_literals;

    // Get the current orientation of the Robot_tf, transforms it to use tf2 libraries.
    std::tuple<double, double, double> GetRPY(const geometry_msgs::msg::TransformStamped &Robot_tf)
    {
        tf2::Quaternion quat(Robot_tf.transform.rotation.x, Robot_tf.transform.rotation.y, Robot_tf.transform.rotation.z, Robot_tf.transform.rotation.w);
        tf2::Matrix3x3 rot_matrix(quat);
        double roll{}, pitch{}, yaw{};
        rot_matrix.getRPY(roll, pitch, yaw);
        return std::make_tuple(roll, pitch, yaw);
    }

    class RobotLink : public rclcpp::Node
    {
    public:
        RobotLink(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions()) : Node("RobotLinkNode", Options)
        {
            // Initialize the transform broadcaster
            tf_broadcast = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            // Initialize the subcriber to tfs
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            // Initialize the subscriber to the Robot cmd.
            Robot_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("Robot_cmd_vel", 1, std::bind(&RobotLink::cmd_model_callback, this, _1));
        };

        // Implements the control logic subcribing to commands, velocities command of the DOF
        void cmd_model_callback(const geometry_msgs::msg::Twist &cmd)
        {
            geometry_msgs::msg::TransformStamped Robot_tf;
            try
            {
                Robot_tf = tf_buffer_->lookupTransform("world", "robot_link", tf2::TimePointZero, 1s);
                //////////////////////////////////////////////////////////////////////////////////////
                rclcpp::Time now = this->get_clock()->now();
                //////////////////////////////////////////////////////////////////////////////////////
                // TRANFORM THE VEL IN X
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "robot_link";
                pose.header.stamp = now;
                pose.pose.position.x = VEL;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                //////////////////////////////////////////////////////////////////////////////////////
                // Keeps track of how much tie has elapsed between callbacks
                //////////////////////////////////////////////////////////////////////////////////////
                rclcpp::Duration delta = now - last_call;
                last_call = now;
                //////////////////////////////////////////////////////////////////////////////////////
                // ASSUMPTION THAT THE 6 DOF CAN BE ACTUATED
                //////////////////////////////////////////////////////////////////////////////////////
                geometry_msgs::msg::TransformStamped Robot;

                std::tuple<double, double, double> Robot_RPY = GetRPY(Robot_tf);
                // double roll = std::get<0>(Robot_RPY);
                double pitch = std::get<1>(Robot_RPY) + delta.seconds() * cmd.angular.y;
                double yaw = std::get<2>(Robot_RPY) + delta.seconds() * cmd.angular.z;

                Robot.header.stamp = now;
                Robot.header.frame_id = "world";
                Robot.child_frame_id = "robot_link";
                Robot.transform.translation.x = Robot_tf.transform.translation.x + (cmd.linear.x) * delta.seconds();
                Robot.transform.translation.y = Robot_tf.transform.translation.y + (cmd.linear.y) * delta.seconds();
                Robot.transform.translation.z = Robot_tf.transform.translation.z + (cmd.linear.z) * delta.seconds();

                // Set the quaternion with new orientation based on commands
                tf2::Quaternion quat;
                quat.setRPY(0.0, pitch, yaw);

                Robot.transform.rotation.x = quat.x();
                Robot.transform.rotation.y = quat.y();
                Robot.transform.rotation.z = quat.z();
                Robot.transform.rotation.w = quat.w();
                // Send the transform
                tf_broadcast->sendTransform(Robot);
                //////////////////////////////////////////////////////////////////////////////////////
            }
            catch (tf2::TransformException &e)
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("cmd_model"), "Could not transform 'Robot' to 'world': %s", e.what());
            }
        };

    private:
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast;

        rclcpp::Time last_call = this->get_clock()->now();

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Robot_cmd_sub;
    };

    class RobotControl : public rclcpp::Node
    {

    public:
        RobotControl(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions()) : Node("RobotControlNode", Options)
        {
            // Initializa the subcriber to the tfs
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Subscribe to the path to follow
            global_path_sub = this->create_subscription<nav_msgs::msg::Path>("global_plan", 10, std::bind(&RobotControl::path_subs_callback, this, _1));

            // Subcribe to Robot tf and execute control logic
            Robot_control = this->create_wall_timer(5ms, std::bind(&RobotControl::control_Robot_callback, this));

            // Publish the control commands
            Robot_cmd = this->create_publisher<geometry_msgs::msg::Twist>("Robot_cmd_vel", 10);

            // pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("SegmentPose", 10);
        };

        // Calculation of yaw and roll of the nearest segment
        void path_subs_callback(const nav_msgs::msg::Path &path)
        {
            this->path = path;
        };

        // SUBCRIBER TO THE Robot POSE TO GET THE NEAREST SEGMENT & CALCULATES COMMAND BASE ON THAT POSE
        // PUBLISHES TO THE TOPIC "Robot_cmd_vel" geometry_msgs/Twist
        void control_Robot_callback()
        {
            // Get the Robot tf.
            geometry_msgs::msg::TransformStamped Robot;
            try
            {
                Robot = tf_buffer_->lookupTransform("world", "robot_link", tf2::TimePointZero, 250ms);
                // Get current angles of the Robot
                std::tuple<double, double, double> Robot_RPY = GetRPY(Robot);

                // Calculate the nearest segment on path, Get the yaw, pitch of such segment
                std::tuple<double, double, double, double, double, double> ControlCMD = Get_Control(Robot, Robot_RPY);

                // Construct the Twist message to send.
                geometry_msgs::msg::Twist cmd;
                /////////////////////////////////
                // Set the velocities for the model to execute in each DOF
                /////////////////////////////////
                cmd.linear.x = std::get<3>(ControlCMD); //+ VEL * cos(std::get<2>(Robot_RPY)) * cos(std::get<1>(Robot_RPY));
                cmd.linear.y = std::get<4>(ControlCMD); //+ VEL * sin(std::get<2>(Robot_RPY)) * cos(std::get<1>(Robot_RPY));
                cmd.linear.z = std::get<5>(ControlCMD); //- VEL * sin(std::get<1>(Robot_RPY));
                /////////////////////////////////
                // ROll is not commanded
                /////////////////////////////////
                cmd.angular.x = 0.0;
                /////////////////////////////////
                cmd.angular.y = std::get<1>(ControlCMD);
                cmd.angular.z = std::get<2>(ControlCMD);

                Robot_cmd->publish(cmd);
            }
            catch (tf2::TransformException &e)
            {
                RCLCPP_INFO(rclcpp::get_logger("control_cb"), "Could not transform 'Robot' to 'world': %s", e.what());
            }
            catch (std::exception &e)
            {
                RCLCPP_INFO(rclcpp::get_logger("control_cb"), "Some exeception occurred: %s", e.what());
            }
            catch (...)
            {
                RCLCPP_INFO(rclcpp::get_logger("control_cb"), "Some exeception occurred");
            };
        };

    private:
        std::tuple<double, double, double, double, double, double> Get_Control(const geometry_msgs::msg::TransformStamped &Robot, std::tuple<double, double, double> Robot_RPY)
        {
            if (path.poses.size() == 0)
                return std::make_tuple<double, double, double, double, double, double>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            if (idx_change)
            {
                idx = GetSegmentIdx(Robot, idx);
                idx_change = false;
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            geometry_msgs::msg::PoseStamped Aitr = path.poses.at(idx);
            geometry_msgs::msg::PoseStamped Bitr = path.poses.at(idx + 1);

            // Get yaw pitch roll from segment
            Eigen::Matrix<double, 3, 1> Apoint;
            Apoint << Aitr.pose.position.x, Aitr.pose.position.y, Aitr.pose.position.z;
            // std::cout << Apoint << std::endl;
            Eigen::Matrix<double, 3, 1> Bpoint;
            Bpoint << Bitr.pose.position.x, Bitr.pose.position.y, Bitr.pose.position.z;

            //  Calculate Yaw parallel to segment
            double yaw = atan2((Bpoint[1] - Apoint[1]), (Bpoint[0] - Apoint[0]));
            // Calculate Pitch
            double pitch = atan2((Apoint[2] - Bpoint[2]), sqrt(std::pow(Bpoint[0] - Apoint[0], 2) + std::pow(Bpoint[1] - Apoint[1], 2)));
            // Calculate rotation Matrix
            Eigen::Matrix<double, 3, 3> rot_yaw;
            rot_yaw << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;
            Eigen::Matrix<double, 3, 3> rot_pitch;
            rot_pitch << cos(pitch), 0, -sin(pitch), 0, 1, 0, sin(pitch), 1, cos(pitch);

            // Get Robot position & orientation
            Eigen::Matrix<double, 3, 1> Robot_pos;
            Robot_pos << Robot.transform.translation.x, Robot.transform.translation.y, Robot.transform.translation.z;

            // Calculate error coordinates in segment coordinate frame r, p, y 0.0, pitch, yaw
            Eigen::Matrix<double, 3, 1> error;
            Eigen::Matrix<double, 3, 1> s_error;
            Eigen::Matrix<double, 3, 3> SegmentTransfrom;
            SegmentTransfrom = rot_pitch * rot_yaw;
            s_error = SegmentTransfrom * (Robot_pos - Bpoint);

            RCLCPP_WARN(rclcpp::get_logger("Robot - SEGMENT"), "ERROR CALC;  x: %f ,y:%f, z:%f", s_error[0], s_error[1], s_error[2]);

            // Lookahead distance
            if (abs(s_error[0]) < TOLL && abs(s_error[2]) < TOLL)
                idx_change = true;

            s_error[0] *= -DELTA_X_ERROR_GAIN;
            s_error[1] *= -DELTA_Y_ERROR_GAIN;
            s_error[2] *= -DELTA_Z_ERROR_GAIN;
            error = SegmentTransfrom.inverse() * s_error;

            // COMPÙTE CONTROL FOR YAW AND PITCH VELOCITIES TO CONTROL THE Robot
            double yaw_desired_vel = YAW_GAIN * (angle_diff(yaw, std::get<2>(Robot_RPY)));
            double pitch_desired_vel = PITCH_GAIN * (angle_diff(pitch, std::get<1>(Robot_RPY)));

            return std::make_tuple<double, double, double, double, double, double>(0.0, std::move(pitch_desired_vel), std::move(yaw_desired_vel), std::move(error[0]), std::move(error[1]), std::move(error[2]));
        };

        double angle_diff(double yaw_des, double yaw_cuur)
        {
            yaw_des = yaw_des < 0 ? 360 + yaw_des : yaw_des;
            yaw_cuur = yaw_cuur < 0 ? 360 + yaw_cuur : yaw_cuur;
            return yaw_des - yaw_cuur;
        };

        int GetSegmentIdx(const geometry_msgs::msg::TransformStamped &Robot, int idx)
        {
            // CALCULATE DISTANCE TO EACH SEGMENT
            std::vector<double> dist;
            std::vector<geometry_msgs::msg::PoseStamped>::iterator itr = path.poses.begin();
            std::advance(itr, idx + 1);
            for (; itr != std::prev(this->path.poses.end()); itr++)
            {
                auto next = std::next(itr);
                // PATH SEGMENT
                Eigen::Matrix<double, 3, 1> line_vector;
                line_vector << next->pose.position.x - itr->pose.position.x, next->pose.position.y - itr->pose.position.y, next->pose.position.z - itr->pose.position.z;
                line_vector.normalize();

                // Robot TO POINT IN SEGMENT
                Eigen::Matrix<double, 3, 1> Robot_segment;
                Robot_segment << itr->pose.position.x - Robot.transform.translation.x, itr->pose.position.y - Robot.transform.translation.y, itr->pose.position.z - Robot.transform.translation.z;

                // DISTANCE CALCULATION Robot TO SEGMENT
                dist.push_back(line_vector.cross(Robot_segment).norm());
            }

            idx += std::distance(dist.begin(), std::min_element(dist.begin(), dist.end())) + 1;
            return idx % (path.poses.size() - 1);
        };

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::TimerBase::SharedPtr Robot_control;

        // Subcriber to the path and get the nearest point in path
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub;
        // Path gotten from subscriber
        nav_msgs::msg::Path path{};

        // Twist command publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Robot_cmd;

        bool idx_change{true};
        int idx{-1};
    };

    class InitialPoseBroadcast : public rclcpp::Node
    {
    public:
        InitialPoseBroadcast(const rclcpp::NodeOptions &Options = rclcpp::NodeOptions()) : Node("RobotInitialNode", Options)
        {
            this->declare_parameter<std::vector<double>>("initial_pose", initial_pose);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_publisher = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
            timer_ = this->create_wall_timer(250ms, std::bind(&InitialPoseBroadcast::broadcast_timer_callback, this));
        }

    private:
        void broadcast_timer_callback()
        {

            try
            {
                tf_buffer_->lookupTransform("world", "robot_link", tf2::TimePointZero, 250ms);
            }
            catch (tf2::TransformException &e)
            {

                RCLCPP_INFO(rclcpp::get_logger("control_cb"), "Could not transform 'Robot' to 'world', sending initial pose: %s", e.what());
                try
                {
                    this->get_parameter("initial_pose", initial_pose);
                    rclcpp::Time now = this->get_clock()->now();
                    geometry_msgs::msg::TransformStamped Robot;
                    Robot.header.stamp = now;

                    Robot.header.frame_id = "world";
                    Robot.child_frame_id = "robot_link";
                    Robot.transform.translation.x = initial_pose[0];
                    Robot.transform.translation.y = initial_pose[1];
                    Robot.transform.translation.z = initial_pose[2];
                    tf2::Quaternion q;
                    q.setRPY(initial_pose[3], initial_pose[4], initial_pose[5]);
                    Robot.transform.rotation.x = q.x();
                    Robot.transform.rotation.y = q.y();
                    Robot.transform.rotation.z = q.z();
                    Robot.transform.rotation.w = q.w();
                    tf_publisher->sendTransform(Robot);
                }
                catch (std::exception &e)
                {
                    RCLCPP_INFO(rclcpp::get_logger("parameter exception"), "Could not get initial pose, %s", e.what());
                }
                catch (...)
                {
                    RCLCPP_INFO(rclcpp::get_logger("parameter exception"), "Could not get initial pose");
                }
            }
        }
        std::vector<double> initial_pose{0,0,0,0,0,0};
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    };

}

RCLCPP_COMPONENTS_REGISTER_NODE(path_follow::RobotLink)
RCLCPP_COMPONENTS_REGISTER_NODE(path_follow::RobotControl)
RCLCPP_COMPONENTS_REGISTER_NODE(path_follow::InitialPoseBroadcast)
