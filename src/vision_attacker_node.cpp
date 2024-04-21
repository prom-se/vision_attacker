#include <rclcpp/rclcpp.hpp>
#include <vision_interfaces/msg/car.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <vision_interfaces/msg/armor.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include "Eigen/Dense"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
struct armorStates
{
    double x;
    double y;
    double z;
    double delta_yaw;
};

struct carStates
{
public:
    void update(const auto_aim_interfaces::msg::Target target_msg, const double self_yaw)
    {
        x = target_msg.position.x;
        y = target_msg.position.y;
        z = target_msg.position.z;
        delta_z = target_msg.dz;
        v_x = target_msg.velocity.x;
        v_y = target_msg.velocity.y;
        v_z = target_msg.velocity.z;
        v_yaw = target_msg.v_yaw;
        this->self_yaw = self_yaw;
    };
    std::vector<armorStates> getPreArmor(const double time)
    {
        std::vector<armorStates> preArmors;
        for (size_t i = 0; i < 4; i++)
        {
            armorStates armor;
            double armor_yaw = yaw + time * v_yaw + M_PI_2 * i;
            while (armor_yaw < 0)
            {
                armor_yaw += 2 * M_PI;
            };
            while (armor_yaw > 2 * M_PI)
            {
                armor_yaw -= 2 * M_PI;
            };
            armor.x = x + time * v_x - cos(armor_yaw);
            armor.y = y + time * v_y - sin(armor_yaw);
            armor.z = z + time * v_z + i % 2 * delta_z;
            armor.delta_yaw = armor_yaw - self_yaw;
            preArmors.emplace_back(armor);
        };
        return preArmors;
    };

private:
    double x;
    double y;
    double z;
    double delta_z;
    double yaw;
    double v_x;
    double v_y;
    double v_z;
    double v_yaw;
    double self_yaw;
} foeCar;

class Tracker_node : public rclcpp::Node
{
public:
    explicit Tracker_node(std::string node_name) : rclcpp::Node(node_name)
    {
        robotPtr = std::make_unique<vision_interfaces::msg::Robot>();
        aimPub = create_publisher<vision_interfaces::msg::AutoAim>(
            "/serial_driver/aim_target", rclcpp::SensorDataQoS());
        // carSub = create_subscription<vision_interfaces::msg::Car>(
        //     "/Obverser/car", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::car_callback, this, std::placeholders::_1));
        armorSub = create_subscription<vision_interfaces::msg::Armor>(
            "/detector/armor", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::armor_callback, this, std::placeholders::_1));
        robotSub = create_subscription<vision_interfaces::msg::Robot>(
            "/serial_driver/robot", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::robot_callback, this, std::placeholders::_1));
        // Create Subscription
        target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
            "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::target_callback, this, std::placeholders::_1));
    };

private:
    void car_callback(const vision_interfaces::msg::Car car_msg)
    {
        Eigen::Vector3d xyz;
        xyz << car_msg.pose.position.x, car_msg.pose.position.y, car_msg.pose.position.z;
        Eigen::Vector2d xy;
        xy << car_msg.pose.position.x, car_msg.pose.position.y;
        double yDis = car_msg.pose.position.z;
        double xDis = xy.norm();

        double speed = robotPtr->muzzle_speed > 5 ? robotPtr->muzzle_speed : 25.00;
        double theta, real_y, aim_y, time;
        for (int i = 0; i < 20; i++)
        {
            theta = atan2(yDis, xDis);
            time = (exp(air_k * xDis) - 1) / (air_k * speed * cos(theta));
            real_y = speed * sin(theta) * time - Gravity * (time * time) / 2;
            aim_y = aim_y + (yDis - real_y);
            if (abs(real_y - yDis) < 0.001)
            {
                offset_time = time;
                xyz(2) = aim_y;
                break;
            }
        }
        vision_interfaces::msg::AutoAim autoAimMsg;
        autoAimMsg.aim_yaw = atan2(xyz(0), xyz(1)) * 180.0 / M_PI;
        autoAimMsg.aim_pitch = atan2(xyz(2), xDis) * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "Yaw:%05.2f/Pitch:%05.2f", autoAimMsg.aim_yaw, autoAimMsg.aim_pitch);
        aimPub->publish(autoAimMsg);
    };

    void armor_callback(const vision_interfaces::msg::Armor armor_msg)
    {
        Eigen::Vector3d xyz;
        xyz << armor_msg.pose.position.x, armor_msg.pose.position.y, armor_msg.pose.position.z;
        Eigen::Vector2d xy;
        xy << armor_msg.pose.position.x, armor_msg.pose.position.y;
        double yDis = armor_msg.pose.position.z;
        double xDis = xy.norm();

        double speed = robotPtr->muzzle_speed > 5 ? robotPtr->muzzle_speed : 25.00;
        double theta, real_y, aim_y, time;
        for (int i = 0; i < 20; i++)
        {
            theta = atan2(yDis, xDis);
            time = (exp(air_k * xDis) - 1) / (air_k * speed * cos(theta));
            real_y = speed * sin(theta) * time - Gravity * (time * time) / 2;
            aim_y = aim_y + (yDis - real_y);
            if (abs(real_y - yDis) < 0.001)
            {
                offset_time = time;
                xyz(2) = aim_y;
                break;
            }
        }
        vision_interfaces::msg::AutoAim autoAimMsg;
        autoAimMsg.aim_yaw = atan2(xyz(0), xyz(1)) * 180.0 / M_PI;
        autoAimMsg.aim_pitch = atan2(xyz(2), xDis) * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "Yaw:%05.2f/Pitch:%05.2f", autoAimMsg.aim_yaw, autoAimMsg.aim_pitch);
        aimPub->publish(autoAimMsg);
    };

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
        Eigen::Vector3d xyz;
        xyz << target_msg.position.x, target_msg.position.y, target_msg.position.z;
        Eigen::Vector2d xy;
        xy << target_msg.position.x, target_msg.position.y;
        double yDis = target_msg.position.z;
        double xDis = xy.norm();

        double speed = robotPtr->muzzle_speed > 5 ? robotPtr->muzzle_speed : 25.00;
        double theta, real_y, aim_y, time;
        double targetY;

        aim_y = yDis;

        double errorTime = 0.08;
        for (int i = 0; i < 20; i++)
        {
            theta = atan2(aim_y, xDis);
            time = (exp(air_k * xDis) - 1) / (air_k * speed * cos(theta));
            real_y = speed * sin(theta) * time - Gravity * (time * time) / 2;
            aim_y = aim_y + (yDis - real_y);
            if (abs(real_y - yDis) < 0.001)
            {
                offset_time = time + errorTime;
                targetY = aim_y;
                break;
            }
        }
        vision_interfaces::msg::AutoAim autoAimMsg;
        double preX;
        double preY;
        double preZ;
        double preYaw = target_msg.yaw + target_msg.v_yaw * offset_time;
        double exYaw = target_msg.v_yaw ? preYaw + M_PI_2 : preYaw - M_PI_2;
        double armorYaw = preYaw;
        while (armorYaw < 0)
        {
            armorYaw += 2 * M_PI;
        }
        armorYaw = armorYaw / M_PI * 180.0 - robotPtr->self_yaw;
        double armorExYaw = target_msg.v_yaw ? armorYaw + 90.0 : armorYaw - 90.0;
        RCLCPP_INFO(this->get_logger(), "armorYaw:%05.2f", armorYaw);
        RCLCPP_INFO(this->get_logger(), "armorExYaw:%05.2f", armorExYaw);
        uint8_t angelGate = 30;
        if (target_msg.tracking)
        {
            foeCar.update(target_msg,robotPtr->self_yaw);
            std::vector<armorStates> armors=foeCar.getPreArmor(errorTime);
            if (abs(target_msg.v_yaw) > 1.5)
            {
                // std::sort();
            }else{

            }
        }
        else if ((autoAimMsg.aim_pitch == 0 && autoAimMsg.aim_yaw == 0) || !target_msg.tracking)
        {
            autoAimMsg.aim_pitch = robotPtr->self_pitch;
            autoAimMsg.aim_yaw = robotPtr->self_yaw;
            autoAimMsg.fire = 0;
        }

        RCLCPP_INFO(this->get_logger(), "Yaw:%05.2f/Pitch:%05.2f/Fire:%01d/Time:%04.2f", autoAimMsg.aim_yaw, autoAimMsg.aim_pitch, autoAimMsg.fire, offset_time);
        aimPub->publish(autoAimMsg);
    };

    double Gravity = 9.8;
    double air_k = 0.043;
    double offset_time = 0.5;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    std::unique_ptr<vision_interfaces::msg::Robot> robotPtr;
    std::unique_ptr<auto_aim_interfaces::msg::Armors> armorsPtr;    
    rclcpp::Subscription<vision_interfaces::msg::Armor>::SharedPtr armorSub;
    rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robotSub;
    rclcpp::Publisher<vision_interfaces::msg::AutoAim>::SharedPtr aimPub;
    rclcpp::Subscription<vision_interfaces::msg::Car>::SharedPtr carSub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tracker_node>("tracker");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}