#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        // Parameters
        target_altitude_ = declare_parameter<double>("target_altitude", 2.0);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
        frame_id_ = declare_parameter<std::string>("frame_id", "map");

        // Publishers / Subscribers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State &msg)
            { state_ = msg; });

        local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                local_pose_ = msg;
                have_local_pose_ = true;
            });

        // Service clients
        arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // This timer streams the current setpoint (needed for OFFBOARD)
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            std::bind(&ControlNode::publish_setpoint, this));

        // Simple ROS services you can call from the CLI
        using std_srvs::srv::Trigger;

        srv_arm_ = this->create_service<Trigger>(
            "/arm",
            [this](const std::shared_ptr<Trigger::Request> /*req*/,
                   std::shared_ptr<Trigger::Response> res)
            {
                res->success = arm(true);
                res->message = res->success ? "armed" : "arm failed";
            });

        srv_disarm_ = this->create_service<Trigger>(
            "/disarm",
            [this](const std::shared_ptr<Trigger::Request> /*req*/,
                   std::shared_ptr<Trigger::Response> res)
            {
                res->success = arm(false);
                res->message = res->success ? "disarmed" : "disarm failed";
            });

        srv_takeoff_ = this->create_service<Trigger>(
            "/takeoff",
            [this](const std::shared_ptr<Trigger::Request> /*req*/,
                   std::shared_ptr<Trigger::Response> res)
            {
                res->success = takeoff();
                res->message = res->success ? "takeoff started" : "takeoff failed";
            });

        srv_land_ = this->create_service<Trigger>(
            "/land",
            [this](const std::shared_ptr<Trigger::Request> /*req*/,
                   std::shared_ptr<Trigger::Response> res)
            {
                res->success = land();
                res->message = res->success ? "landing" : "land failed";
            });

        RCLCPP_INFO(get_logger(), "ControlNode ready. Params: target_altitude=%.2f, rate=%.1f Hz",
                    target_altitude_, publish_rate_hz_);
    }

private:
    // ---- Arm/Disarm
    bool arm(bool value)
    {
        if (!arm_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(get_logger(), "Arming service not available");
            return false;
        }
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = value;
        auto fut = arm_client_->async_send_request(req);
        auto ret = wait_for(fut);
        if (!ret)
            return false;
        bool ok = fut.get()->success;
        RCLCPP_INFO(get_logger(), value ? "Arm: %s" : "Disarm: %s", ok ? "OK" : "FAILED");
        return ok;
    }

    // ---- Set mode
    bool set_mode(const std::string &mode)
    {
        if (!mode_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(get_logger(), "SetMode service not available");
            return false;
        }
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = mode;
        auto fut = mode_client_->async_send_request(req);
        auto ret = wait_for(fut);
        if (!ret)
            return false;
        bool ok = fut.get()->mode_sent;
        RCLCPP_INFO(get_logger(), "SetMode '%s': %s", mode.c_str(), ok ? "OK" : "FAILED");
        return ok;
    }

    // ---- Takeoff using OFFBOARD position hold
    bool takeoff()
    {
        if (!have_local_pose_)
        {
            RCLCPP_WARN(get_logger(), "No local pose yet; waiting up to 5s...");
            auto t0 = now();
            rclcpp::Rate r(20.0);
            while (!have_local_pose_ && (now() - t0) < rclcpp::Duration(5, 0))
                r.sleep();
            if (!have_local_pose_)
            {
                RCLCPP_ERROR(get_logger(), "No local pose; cannot take off");
                return false;
            }
        }

        // Freeze XY at current position, target Z = target_altitude_
        setpoint_pose_ = local_pose_;
        setpoint_pose_.header.frame_id = frame_id_;
        setpoint_pose_.pose.position.z = target_altitude_;
        have_setpoint_ = true;

        // PX4 requires streaming setpoints BEFORE switching to OFFBOARD
        RCLCPP_INFO(get_logger(), "Priming OFFBOARD setpoints...");
        rclcpp::Rate prime_rate(20.0);
        for (int i = 0; i < 60; ++i)  // ~3 seconds
        {
            auto now = this->get_clock()->now();
            setpoint_pose_.header.stamp.sec  = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
            setpoint_pose_.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
            pose_pub_->publish(setpoint_pose_);
            prime_rate.sleep();
        }

        bool ok_mode = set_mode("OFFBOARD");
        bool ok_arm = arm(true);
        return ok_mode && ok_arm;
    }

    // ---- Land via PX4 AUTO.LAND
    bool land()
    {
        return set_mode("AUTO.LAND");
    }

    // ---- Timer: continuously publish the current setpoint
    void publish_setpoint()
    {
        if (!have_setpoint_) return; // nothing to publish yet
        auto now = this->get_clock()->now();
        setpoint_pose_.header.stamp.sec  = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
        setpoint_pose_.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
        pose_pub_->publish(setpoint_pose_);
    }

    // ---- helpers
    template <typename FutureT>
    bool wait_for(FutureT &fut, std::chrono::milliseconds timeout = 2s)
    {
        auto ret = rclcpp::spin_until_future_complete(shared_from_this(), fut, timeout);
        if (ret != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Service call timeout/failure");
            return false;
        }
        return true;
    }

    // Members
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_, srv_disarm_, srv_takeoff_, srv_land_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State state_;
    geometry_msgs::msg::PoseStamped local_pose_;
    geometry_msgs::msg::PoseStamped setpoint_pose_;

    bool have_local_pose_ = false;
    bool have_setpoint_ = false;
    double target_altitude_{2.0};
    double publish_rate_hz_{20.0};
    std::string frame_id_{"map"};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
