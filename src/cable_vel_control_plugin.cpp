#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Point.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh> // Add this line
#include <sdf/Element.hh>
#include <thread> // Add this line for sleep functionality

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

        ros::NodeHandle nh;
        ros::Subscriber targetSub;

    public:
        CableVelControlPlugin() : ModelPlugin(), nh("~") // Initialize the NodeHandle with private namespace
        {
            // Initialize the cable velocity
            this->targetPosition.Set(0, 0, 0);
        }

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the model pointer for convenience
            this->model = _model;

            // Check if ROS is initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL("ROS is not initialized. Plugin will not work.");
                return;
            }

            // Get the link
            this->link = this->model->GetLink("end_sphere");
            if (!this->link)
            {
                ROS_FATAL("Link end_sphere not found. Plugin will not work.");
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

            // Create a ROS node
            this->targetSub = this->nh.subscribe("/cable_target_position", 1, &CableVelControlPlugin::OnRosMsg, this);

            // Add a small delay to ensure parameter server is ready
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Fetch PID parameters
            std::map<std::string, double> cablePID;
            // nh.setParam("/cable_PID", std::vector<double>{0.1, 0.01, 0.01});

            // check if the parameter exists
            if (!this->nh.hasParam("/cable_PID"))
            {
                ROS_WARN("Parameter /cable_PID does not exist. Using default gains.");
                this->kP = 5.0;
                this->kI = 0.01;
                this->kD = 0.01;
            }
            else
            {
                ROS_INFO("Parameter /cable_PID exists. Fetching gains.");

                // Fetch the gains
                if (this->nh.getParam("/cable_PID", cablePID))
                {
                    if (cablePID.find("kP") != cablePID.end() && cablePID.find("kI") != cablePID.end() && cablePID.find("kD") != cablePID.end())
                    {
                        this->kP = cablePID["kP"];
                        this->kI = cablePID["kI"];
                        this->kD = cablePID["kD"];
                        ROS_INFO_STREAM("PID gains: " << this->kP << ", " << this->kI << ", " << this->kD);
                    }
                    else
                    {
                        ROS_ERROR("PID gains vector does not have exactly 3 elements.");
                    }
                }
                else
                {
                    ROS_ERROR("Failed to get PID gains from parameter server.");
                }
            }
        }

        void OnUpdate()
        {
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
            ROS_INFO("Current position: %f, %f, %f", this->link->WorldPose().Pos().X(), this->link->WorldPose().Pos().Y(), this->link->WorldPose().Pos().Z());
            ROS_INFO("Control input: %f, %f, %f", this->controlInput.X(), this->controlInput.Y(), this->controlInput.Z());
        }
        void OnRosMsg(const geometry_msgs::Point::ConstPtr &_msg)
        {
            this->targetPosition.Set(_msg->x, _msg->y, _msg->z);
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(CableVelControlPlugin)
};