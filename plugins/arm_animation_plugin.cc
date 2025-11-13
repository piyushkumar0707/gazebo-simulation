#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <chrono>

namespace gazebo
{
  class ArmAnimationPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: common::Time lastUpdateTime;
    private: double animationTime;
    private: physics::JointPtr baseJoint;
    private: physics::JointPtr shoulderJoint;
    private: physics::JointPtr elbowJoint;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->animationTime = 0.0;
      this->lastUpdateTime = this->model->GetWorld()->SimTime();

      // Get joints
      this->baseJoint = this->model->GetJoint("joint_base");
      this->shoulderJoint = this->model->GetJoint("joint_shoulder");
      this->elbowJoint = this->model->GetJoint("joint_elbow");

      if (!this->baseJoint || !this->shoulderJoint || !this->elbowJoint)
      {
        gzerr << "One or more joints not found!\n";
        return;
      }

      // Listen to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ArmAnimationPlugin::OnUpdate, this));

      gzmsg << "Arm Animation Plugin loaded! The arm will perform an automated sequence.\n";
    }

    public: void OnUpdate()
    {
      common::Time currentTime = this->model->GetWorld()->SimTime();
      double dt = (currentTime - this->lastUpdateTime).Double();
      this->lastUpdateTime = currentTime;
      this->animationTime += dt;

      // Animated sequence (repeats every 20 seconds)
      double t = fmod(this->animationTime, 20.0);
      
      double baseAngle = 0.0;
      double shoulderAngle = 0.0;
      double elbowAngle = 0.0;

      // Define keyframes for pick and place motion
      if (t < 3.0) {
        // Stage 1: Move to home position
        baseAngle = 0.0;
        shoulderAngle = 0.0;
        elbowAngle = 0.0;
      }
      else if (t < 6.0) {
        // Stage 2: Rotate base and reach toward object
        double progress = (t - 3.0) / 3.0;
        baseAngle = progress * 1.0;  // Rotate 60 degrees
        shoulderAngle = progress * 0.8;
        elbowAngle = progress * -0.6;
      }
      else if (t < 9.0) {
        // Stage 3: Lower to pick up (hold position)
        baseAngle = 1.0;
        shoulderAngle = 0.8;
        elbowAngle = -0.6;
      }
      else if (t < 12.0) {
        // Stage 4: Lift and rotate to new location
        double progress = (t - 9.0) / 3.0;
        baseAngle = 1.0 - progress * 2.0;  // Rotate to other side
        shoulderAngle = 0.8 - progress * 0.3;
        elbowAngle = -0.6 + progress * 0.3;
      }
      else if (t < 15.0) {
        // Stage 5: Lower to place
        double progress = (t - 12.0) / 3.0;
        baseAngle = -1.0;
        shoulderAngle = 0.5 + progress * 0.3;
        elbowAngle = -0.3 - progress * 0.3;
      }
      else {
        // Stage 6: Return to home
        double progress = (t - 15.0) / 5.0;
        baseAngle = -1.0 * (1.0 - progress);
        shoulderAngle = 0.8 * (1.0 - progress);
        elbowAngle = -0.6 * (1.0 - progress);
      }

      // Apply positions with PID control
      this->baseJoint->SetPosition(0, baseAngle);
      this->shoulderJoint->SetPosition(0, shoulderAngle);
      this->elbowJoint->SetPosition(0, elbowAngle);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(ArmAnimationPlugin)
}
