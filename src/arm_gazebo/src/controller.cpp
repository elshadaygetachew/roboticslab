// #include <functional>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>
// #include <ignition/math/Vector3.hh>
// #include <iostream>
// namespace gazebo
// {
// 	class ModelPush : public ModelPlugin
// 	{
// 	public:
// 		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
// 		{
// 			// Store the pointer to the model
// 			this->model = _parent;

// 			// // intiantiate the joint controller
// 			this->jointController = this->model->GetJointController();

// 			// // set your PID values
// 			this->pid = common::PID(30.1, 10.01, 10.03);

// 			auto joint_name = "arm1_arm2_joint";

// 			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

// 			this->jointController->SetPositionPID(name, pid);

// 			// Listen to the update event. This event is broadcast every
// 			// simulation iteration.
// 			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
// 				std::bind(&ModelPush::OnUpdate, this));
// 		}

// 		// Called by the world update start event
// 	public:
// 		void OnUpdate()
// 		{
// 			float angleDegree = -90;
// 			float rad = M_PI * angleDegree / 180;

// 			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
// 			// this->jointController->SetPositionPID(name, pid);
// 			// this->jointController->SetPositionTarget(name, rad);
// 			// this->jointController->Update();

// 			// Get joint position by index. 
// 			// 0 returns rotation accross X axis
// 			// 1 returns rotation accross Y axis
// 			// 2 returns rotation accross Z axis
// 			// If the Joint has only Z axis for rotation, 0 returns that value and 1 and 2 return nan
// 			double a1 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);
// 			// double a2 = this->model->GetJoint("chasis_arm1_joint").Position(0);
// 			// double a3 = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position(2);
// 			std::cout << "Current arm1_arm2_joint values: " << a1 * 180.0 / M_PI << std::endl;
// 		}

// 		// a pointer that points to a model object
// 	private:
// 		physics::ModelPtr model;

// 		// 	// A joint controller object
// 		// 	// Takes PID value and apply angular velocity
// 		// 	//  or sets position of the angles
// 	private:
// 		physics::JointControllerPtr jointController;

// 	private:
// 		event::ConnectionPtr updateConnection;

// 		// // 	// PID object
// 	private:
// 		common::PID pid;
// 	};

// 	// Register this plugin with the simulator
// 	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
// }




#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "arm_gazebo/JointMessage.h"

namespace gazebo
{
  class Controller1 : public ModelPlugin
  {
    public: Controller1() : ModelPlugin()
    {
        printf("Hello World!\n");
    }

    private: void updateJointAnglesCallback(const arm_gazebo::JointMessage& msg)
    {
        ROS_INFO("Received angle");

        std::string name1 = this->j1->GetScopedName();

        this->jointController->SetPositionPID(name1, this->pidJoint1);
        this->jointController->SetPositionTarget(name1, getRad(msg.jointOne));

        std::string name2 = this->j2->GetScopedName();

        this->jointController->SetPositionPID(name2, this->pidJoint2);
        this->jointController->SetPositionTarget(name2, getRad(msg.jointTwo));

        std::string name3 = this->j3->GetScopedName();

        this->jointController->SetPositionPID(name3, this->pidJoint3);
        this->jointController->SetPositionTarget(name3, getRad(msg.jointThree));

        std::string name4 = this->j4->GetScopedName();

        this->jointController->SetPositionPID(name4, this->pidJoint4);
        this->jointController->SetPositionTarget(name4, getRad(msg.jointFour));
    }

    private: static float getRad(float angleDegree) {
        float rad = M_PI * angleDegree/180 ;
        return rad;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;

        this->jointController = this->model->GetJointController();

        this->pidJoint1 = common::PID(8, 4, 5, 3);

        this->pidJoint2 = common::PID(10, 8.1, 7);

        this->pidJoint3 = common::PID(10, 8.1, 8);

        this->pidJoint4 = common::PID(10, 8.1, 8);

        auto joint_name1 = "chasis_arm1_joint";
        auto joint_name2 = "arm1_arm2_joint";
        auto joint_name3 = "arm2_arm3_joint";
        auto joint_name4 = "arm3_arm4_joint";

        this->j1 = this->model->GetJoint(joint_name1);
        this->j2 = this->model->GetJoint(joint_name2);
        this->j3 = this->model->GetJoint(joint_name3);
        this->j4 = this->model->GetJoint(joint_name4);

        this->jointPublisher = this->nodeHandle.advertise<arm_gazebo::JointMessage>("/jointmessage", 10);
        this->jointSubscriber = this->nodeHandle.subscribe("updateJointAngles", 1000, &Controller1::updateJointAnglesCallback, this);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&Controller1::OnUpdate, this)
        );
    }
    //called by the world update start event
    public : void OnUpdate()
    {
        arm_gazebo::JointMessage msg;

        msg.jointOne = this->j1->Position();
        msg.jointTwo = this->j2->Position();
        msg.jointThree = this->j3->Position();
        msg.jointFour = this->j4->Position();

        this->jointPublisher.publish(msg);

        this->jointController->Update();
    }

    //Pointer to the model
    private: physics::ModelPtr model;

    //Pointer to the model
    private: physics::JointControllerPtr jointController;

    //Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: common::PID pidJoint1;
    private: common::PID pidJoint2;
    private: common::PID pidJoint3;
    private: common::PID pidJoint4;

    private: physics::JointPtr j1;
    private: physics::JointPtr j2;
    private: physics::JointPtr j3;
    private: physics::JointPtr j4;

    private: ros::NodeHandle nodeHandle;
    private: ros::Publisher jointPublisher;
    private: ros::Subscriber jointSubscriber;
  };
  GZ_REGISTER_MODEL_PLUGIN(Controller1)
}