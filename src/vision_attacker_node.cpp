#include <rclcpp/rclcpp.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "Eigen/Dense"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "../include/vision_attacker/outpost.hpp"

class Tracker_node : public rclcpp::Node
{
public:
    explicit Tracker_node(std::string node_name) : rclcpp::Node(node_name)
    {
        lagTime = declare_parameter("lag_time", 0.08);
        carThreshold = declare_parameter("car_attack_threshold", 30);
        outpostThreshold = declare_parameter("outpost_attack_threshold", 5);
        airK = declare_parameter("air_k", 0.04);
        robotPtr = std::make_unique<vision_interfaces::msg::Robot>();
        markerPub = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
        aimPub = create_publisher<vision_interfaces::msg::AutoAim>(
            "/serial_driver/aim_target", rclcpp::SensorDataQoS());
        robotSub = create_subscription<vision_interfaces::msg::Robot>(
            "/serial_driver/robot", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::robot_callback, this, std::placeholders::_1));
        targetSub = this->create_subscription<auto_aim_interfaces::msg::Target>(
            "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::target_callback, this, std::placeholders::_1));
        
        aimPoint.header.frame_id = "odom";
        aimPoint.ns = "aiming_point";
        aimPoint.type = visualization_msgs::msg::Marker::SPHERE;
        aimPoint.action = visualization_msgs::msg::Marker::ADD;
        aimPoint.scale.x = aimPoint.scale.y = aimPoint.scale.z = 0.12;
        aimPoint.color.r = 1.0;
        aimPoint.color.g = 1.0;
        aimPoint.color.b = 1.0;
        aimPoint.color.a = 1.0;
        aimPoint.lifetime = rclcpp::Duration::from_seconds(0.1);
    };

private:
    void robot_callback(const vision_interfaces::msg::Robot robot)
    {
        try
        {
            *robotPtr = robot;
        }
        catch (std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "获取机器人信息时发生错误.");
        }
    };

    void armors_callback(const auto_aim_interfaces::msg::Armors armor)
    {
        try
        {
            *armorsPtr = armor;
            armorsPtr->armors[0].pose.position.x;
        }
        catch (std::exception &ex)
        {
            return;
        }
    };

    void target_callback(const auto_aim_interfaces::msg::Target target_msg)
    {
        lagTime = get_parameter("lag_time").as_double();
        carThreshold = get_parameter("car_attack_threshold").as_double();
        outpostThreshold = get_parameter("outpost_attack_threshold").as_double();
        airK = get_parameter("air_k").as_double();

        Eigen::Vector2d xy;
        xy << target_msg.position.x, target_msg.position.y;
        double yDis = target_msg.position.z;
        double xDis = xy.norm();
        double speed = robotPtr->muzzle_speed > 5 ? robotPtr->muzzle_speed : 25.00;
        double flyTime = 0;
        solveTrajectory(flyTime, xDis, yDis, lagTime, speed, 0.04);

        vision_interfaces::msg::AutoAim aim;
        aim.aim_yaw = robotPtr->self_yaw;
        aim.aim_pitch = robotPtr->self_pitch;
        aim.fire = 0;
        if (target_msg.id == "outpost")
        {
            outpostStates foeOutpost;
            foeOutpost.update(target_msg, robotPtr->self_yaw);
            std::vector<armorStates> armors = foeOutpost.getPreArmor(flyTime);
            for (const auto armor : armors)
            {
                if (abs(armor.delta_yaw) < outpostThreshold / 180.0 * M_PI)
                {
                    aim.aim_yaw = atan2(armor.y, armor.x);
                    aim.aim_pitch = solveTrajectory(flyTime, armor.x, armor.z, lagTime, speed, 0.04);
                    aim.fire = 1;
                    aimPoint.pose.position.x = armor.x;
                    aimPoint.pose.position.y = armor.y;
                    aimPoint.pose.position.z = armor.z;
                    break;
                }
                else
                {
                    continue;
                }
            }
        }
        else
        {
            carStates foeCar;
            foeCar.update(target_msg, robotPtr->self_yaw);
            std::vector<armorStates> armors = foeCar.getPreArmor(flyTime);
            for (const auto armor : armors)
            {
                if (abs(armor.delta_yaw) < carThreshold / 180.0 * M_PI)
                {
                    aim.aim_yaw = atan2(armor.y, armor.x);
                    aim.aim_pitch = solveTrajectory(flyTime, armor.x, armor.z, lagTime, speed, airK);
                    aim.fire = 1;
                    aimPoint.pose.position.x = armor.x;
                    aimPoint.pose.position.y = armor.y;
                    aimPoint.pose.position.z = armor.z;
                    break;
                }
                else
                {
                    continue;
                }
            }
        }
        aimPoint.header.stamp=now();
        markerPub->publish(aimPoint);
        aimPub->publish(aim);
    };

    double solveTrajectory(double &flyTime, const double x, const double z, const double lagTime, const double muzzleSpeed, const double air_k)
    {
        const double gravity = 0.978;
        double theta;
        double time;
        double aimZ;
        double realZ;
        double targetZ;

        aimZ = z;
        for (size_t i = 0; i < 20; i++)
        {
            theta = atan2(aimZ, x);
            time = (exp(air_k * x) - 1) / (air_k * muzzleSpeed * cos(theta));
            realZ = muzzleSpeed * sin(theta) * time - gravity * (time * time) / 2;
            aimZ = aimZ + (z - realZ);
            if (abs(realZ - z) < 0.001)
            {
                time += lagTime;
                flyTime = time;
                targetZ = aimZ;
                break;
            }
            else
            {
                continue;
            }
        }
        return atan2(targetZ, x);
    }
    double lagTime = 0.08;
    double carThreshold = 30;
    double outpostThreshold = 5;
    double airK = 0.04;
    visualization_msgs::msg::Marker aimPoint;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr targetSub;
    std::unique_ptr<vision_interfaces::msg::Robot> robotPtr;
    std::unique_ptr<auto_aim_interfaces::msg::Armors> armorsPtr;
    rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robotSub;
    rclcpp::Publisher<vision_interfaces::msg::AutoAim>::SharedPtr aimPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tracker_node>("tracker");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}