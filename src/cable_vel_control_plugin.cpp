#include <rclcpp/rclcpp.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/msg/point.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <sdf/Element.hh>
#include <thread>

namespace gazebo
{
    class CableVelControlPlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        physics::LinkPtr link;
        event::ConnectionPtr updateConnection;

        ignition::math::Vector3d targetPosition;
        ignition::math::Vector3d error;
        ignition::math::Vector3d integral;
        ignition::math::Vector3d prevError;
        ignition::math::Vector3d derivative;
        ignition::math::Vector3d controlInput;

        double kP;
        double kI;
        double kD;

        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr targetSub;

    public:
        CableVelControlPlugin() : ModelPlugin()
        {
            // Initialize the cable velocity
            this->targetPosition.Set(0, 0, 0);
        }

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the model pointer for convenience
            this->model = _model;

            // Initialize ROS 2 node
            if (!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }
            this->node = rclcpp::Node::make_shared("cable_vel_control_plugin");

            // Get the link
            this->link = this->model->GetLink("end_sphere");
            if (!this->link)
            {
                RCLCPP_FATAL(this->node->get_logger(), "Link end_sphere not found. Plugin will not work.");
                return;
            }
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&CableVelControlPlugin::OnUpdate, this));

            // Initialize the target position at the current position
            this->targetPosition = this->link->WorldPose().Pos();

            // Initialize the PID controller
            this->error = this->targetPosition - this->link->WorldPose().Pos();
            this->integral = ignition::math::Vector3d(0, 0, 0);
            this->prevError = ignition::math::Vector3d(0, 0, 0);

            // Create a ROS 2 subscriber
            this->targetSub = this->node->create_subscription<geometry_msgs::msg::Point>(
                "/cable_target_position", 10,
                std::bind(&CableVelControlPlugin::OnRosMsg, this, std::placeholders::_1));

            // Add a small delay to ensure parameter server is ready
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Fetch PID parameters - using ROS 2 parameter API
            // Declare and get parameters with default values
            this->node->declare_parameter("cable_PID.kP", 5.0);
            this->node->declare_parameter("cable_PID.kI", 0.01);
            this->node->declare_parameter("cable_PID.kD", 0.01);
            
            this->kP = this->node->get_parameter("cable_PID.kP").as_double();
            this->kI = this->node->get_parameter("cable_PID.kI").as_double();
            this->kD = this->node->get_parameter("cable_PID.kD").as_double();
            
            RCLCPP_INFO_STREAM(this->node->get_logger(), "PID gains: " << this->kP << ", " << this->kI << ", " << this->kD);
        }

        void OnUpdate()
        {
            // Spin the ROS 2 node to process callbacks
            rclcpp::spin_some(this->node);
            
            // Calculate the error
            this->error = this->targetPosition - this->link->WorldPose().Pos();

            // Calculate the integral
            this->integral += this->error;

            // Calculate the derivative
            this->derivative = this->error - this->prevError;

            // Calculate the control input
            this->controlInput = this->kP * this->error + this->kI * this->integral + this->kD * this->derivative;

            // Apply the control input
            this->link->SetLinearVel(this->controlInput);

            // Store the previous error
            this->prevError = this->error;

            // Publish the current position
            RCLCPP_INFO(this->node->get_logger(), "Current position: %f, %f, %f", 
                       this->link->WorldPose().Pos().X(), this->link->WorldPose().Pos().Y(), this->link->WorldPose().Pos().Z());
            RCLCPP_INFO(this->node->get_logger(), "Control input: %f, %f, %f", 
                       this->controlInput.X(), this->controlInput.Y(), this->controlInput.Z());
        }
        
        void OnRosMsg(const geometry_msgs::msg::Point::SharedPtr _msg)
        {
            this->targetPosition.Set(_msg->x, _msg->y, _msg->z);
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(CableVelControlPlugin)
};