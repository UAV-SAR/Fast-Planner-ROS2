#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include "waypoint_generator/sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2/utils.hpp>

using namespace std;
using bfmt = boost::format;

class WayPointGeneratorNode : public rclcpp::Node
{
    public:
        WayPointGeneratorNode() : Node("waypoint_generator")
        {
            this->declare_parameter("waypoint_type", string("manual"));
            this->get_parameter("waypoint_type", waypoint_type);

            sub1 = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&WayPointGeneratorNode::odom_callback, this, std::placeholders::_1));
            sub2 = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, std::bind(&WayPointGeneratorNode::goal_callback, this, std::placeholders::_1));
            sub3 = this->create_subscription<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10, std::bind(&WayPointGeneratorNode::traj_start_trigger_callback, this, std::placeholders::_1));

            pub1 = this->create_publisher<nav_msgs::msg::Path>("/waypoint_generator/waypoints", 50);
            pub2 = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoint_generator/waypoints_vis", 10);

            trigged_time = rclcpp::Time(0);
        }

        void load_seg(int segid, const rclcpp::Time &time_base)
        {
                std::string seg_str = boost::str(bfmt("seg%d/") % segid);
                double yaw;
                double time_from_start;
                RCLCPP_INFO(this->get_logger(), "Getting segment %d", segid);

                this->declare_parameter(seg_str + "yaw", 0.0);
                this->get_parameter(seg_str + "yaw", yaw);
                if (!(yaw > -3.1499999 && yaw < 3.14999999))
                {
                    RCLCPP_ERROR(this->get_logger(), "yaw=%.3f is out of range", yaw);
                    throw std::runtime_error("Yaw value out of range");
                }

                this->declare_parameter(seg_str + "time_from_start", 0.0);
                this->get_parameter(seg_str + "time_from_start", time_from_start);
                if (!(time_from_start >= 0.0))
                {
                    RCLCPP_ERROR(this->get_logger(), "time_from_start=%.3f is invalid", time_from_start);
                    throw std::runtime_error("Time from start value invalid");
                }

                std::vector<double> ptx;
                std::vector<double> pty;
                std::vector<double> ptz;

                this->get_parameter(seg_str + "x", ptx);
                this->get_parameter(seg_str + "y", pty);
                this->get_parameter(seg_str + "z", ptz);

                if (!(ptx.size() == pty.size() && ptx.size() == ptz.size()))
                {
                    RCLCPP_ERROR(this->get_logger(), "x, y, z size mismatch: %zu, %zu, %zu", ptx.size(), pty.size(), ptz.size());
                    throw std::runtime_error("Waypoint size mismatch");
                }

                nav_msgs::msg::Path path_msg;

                int32_t sec = static_cast<int32_t>(time_from_start);
                uint32_t nsec = static_cast<uint32_t>((time_from_start - sec) * 1e9);
                path_msg.header.stamp = time_base + rclcpp::Duration(sec, nsec);

                double baseyaw = tf2::getYaw(odom.pose.pose.orientation);
                
                for (size_t k = 0; k < ptx.size(); ++k) {
                    geometry_msgs::msg::PoseStamped pt;
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, baseyaw + yaw);
                    pt.pose.orientation = tf2::toMsg(q);
                    Eigen::Vector2d dp(ptx.at(k), pty.at(k));
                    Eigen::Vector2d rdp;
                    rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
                    rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
                    pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
                    pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
                    pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
                    path_msg.poses.push_back(pt);
                }

                waypointSegments.push_back(path_msg);
        }

        void load_waypoints(const rclcpp::Time &time_base)
        {
            int seg_cnt = 0;
            waypointSegments.clear();
            this->get_parameter("segment_cnt", seg_cnt);
            for (int i = 0; i < seg_cnt; ++i) {
                this->load_seg(i, time_base);
                if (i > 0) {
                    if (!(rclcpp::Time(waypointSegments[i - 1].header.stamp) < rclcpp::Time(waypointSegments[i].header.stamp)))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Segment %d time is not increasing: %.3f vs %.3f", i, rclcpp::Time(waypointSegments[i - 1].header.stamp).seconds(), rclcpp::Time(waypointSegments[i].header.stamp).seconds());
                        throw std::runtime_error("Segment time not increasing");
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "Overall load %zu segments", waypointSegments.size());
        }

        void publish_waypoints() 
        {
            waypoints.header.frame_id = std::string("world");
            waypoints.header.stamp = rclcpp::Clock().now();
            pub1->publish(waypoints);
            geometry_msgs::msg::PoseStamped init_pose;
            init_pose.header = odom.header;
            init_pose.pose = odom.pose.pose;
            waypoints.poses.insert(waypoints.poses.begin(), init_pose);
            waypoints.poses.clear();
        }
        
        void publish_waypoints_vis()
        {
            nav_msgs::msg::Path wp_vis = waypoints;
            geometry_msgs::msg::PoseArray poseArray;
            poseArray.header.frame_id = std::string("world");
            poseArray.header.stamp = rclcpp::Clock().now();

            {
                geometry_msgs::msg::Pose init_pose;
                init_pose = odom.pose.pose;
                poseArray.poses.push_back(init_pose);
            }

            for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
                geometry_msgs::msg::Pose p;
                p = it->pose;
                poseArray.poses.push_back(p);
            }
            pub2->publish(poseArray);
        }

        void odom_callback(const nav_msgs::msg::Odometry &msg) 
        {
            is_odom_ready = true;
            odom = msg;

            if (waypointSegments.size()) {
                rclcpp::Time expected_time = waypointSegments.front().header.stamp;
                if (rclcpp::Time(odom.header.stamp) >= expected_time) {
                    waypoints = waypointSegments.front();

                    std::stringstream ss;
                    ss << bfmt("Series send %.3f from start:\n") % trigged_time.seconds();
                    for (auto& pose_stamped : waypoints.poses) {
                        ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                                pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                                pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                                pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                                pose_stamped.pose.orientation.z << std::endl;
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());

                    publish_waypoints_vis();
                    publish_waypoints();

                    waypointSegments.pop_front();
                }
            }
        }

        void goal_callback(const geometry_msgs::msg::PoseStamped &msg) 
        {
            trigged_time = rclcpp::Clock().now();
            
            if (waypoint_type == string("circle")) {
                waypoints = circle();
                publish_waypoints_vis();
                publish_waypoints();
            } else if (waypoint_type == string("eight")) {
                waypoints = eight();
                publish_waypoints_vis();
                publish_waypoints();
            } else if (waypoint_type == string("point")) {
                waypoints = point();
                publish_waypoints_vis();
                publish_waypoints();
            } else if (waypoint_type == string("series")) {
                load_waypoints(trigged_time);
            } else if (waypoint_type == string("manual-lonely-waypoint")) {
                if (msg.pose.position.z > -0.1) {
                    geometry_msgs::msg::PoseStamped pt = msg;
                    waypoints.poses.clear();
                    waypoints.poses.push_back(pt);
                    publish_waypoints_vis();
                    publish_waypoints();
                } else {
                    RCLCPP_WARN(this->get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
                }
            } else {
                if (msg.pose.position.z > 0) {
                    geometry_msgs::msg::PoseStamped pt = msg;
                    if (waypoint_type == string("noyaw")) {
                        double yaw = tf2::getYaw(odom.pose.pose.orientation);
                        tf2::Quaternion q;
                        q.setRPY(0.0, 0.0, yaw);
                        pt.pose.orientation = tf2::toMsg(q);
                    }
                    waypoints.poses.push_back(pt);
                    publish_waypoints_vis();
                } else if (msg.pose.position.z > -1.0) {
                    if (waypoints.poses.size() >= 1) {
                        waypoints.poses.erase(std::prev(waypoints.poses.end()));
                    }
                    publish_waypoints_vis();
                } else {
                    if (waypoints.poses.size() >= 1) {
                        publish_waypoints_vis();
                        publish_waypoints();
                    }
                }
            }
        }

        void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped& msg) 
        {
            if (!is_odom_ready) {
                RCLCPP_ERROR(this->get_logger(), "[waypoint_generator] No odom!");
                return;
            }

            RCLCPP_WARN(this->get_logger(), "[waypoint_generator] Trigger!");
            trigged_time = rclcpp::Time(odom.header.stamp);
            if (!(trigged_time > rclcpp::Time(0)))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid trigger time: %.3f", trigged_time.seconds());
                return;
            }

            RCLCPP_ERROR_STREAM(this->get_logger(), "Pattern " << waypoint_type << " generated!");
            if (waypoint_type == string("free")) {
                waypoints = point();
                publish_waypoints_vis();
                publish_waypoints();
            } else if (waypoint_type == string("circle")) {
                waypoints = circle();
                publish_waypoints_vis();
                publish_waypoints();
            } else if (waypoint_type == string("eight")) {
                waypoints = eight();
                publish_waypoints_vis();
                publish_waypoints();
        } else if (waypoint_type == string("point")) {
                waypoints = point();
                publish_waypoints_vis();
                publish_waypoints();
            } else if (waypoint_type == string("series")) {
                load_waypoints(trigged_time);
            }
        }

    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3;
        string waypoint_type = string("manual");
        bool is_odom_ready;
        nav_msgs::msg::Odometry odom;
        nav_msgs::msg::Path waypoints;
        std::deque<nav_msgs::msg::Path> waypointSegments;
        rclcpp::Time trigged_time;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto n = std::make_shared<WayPointGeneratorNode>();
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
