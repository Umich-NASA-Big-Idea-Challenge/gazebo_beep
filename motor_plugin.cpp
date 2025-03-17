#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>

#include <cmath>

namespace ignition
{
namespace gazebo
{
class CustomMotorPlugin : public ignition::gazebo::System,
                          public ISystemConfigure,
                          public ISystemPreUpdate
{
private:
    ignition::transport::Node node;
    Entity jointEntity;
    const double max_voltage = 48.0;
    const double resistance = 170 * pow(10,-3);
    const double inductance = 57 * pow(10,-6);
    const double ke = 0.1002; // V/(rad/s)
    const double kt = 0.105;
    const double max_current = 22.3;
    const double rated_torque = 9.0; // 9 Nm
    const double max_torque = 18.0; //peak torque
    double dt = 0.001;
    double current = 0.0;
    double duty_cycle = 0.0;

public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &) override
    {
        Model model(_entity);
        if (!model.Valid(_ecm))
        {
            ignerr << "CustomMotorPlugin: Model entity is invalid.\n";
            return;
        }

        std::string joint_name;
        if (_sdf->HasElement("joint_name"))
        {
            joint_name = _sdf->Get<std::string>("joint_name");
        } else {
            ignerr << "CustomMotorPlugin: joint_name needs to be specified.\n";
            return;
        }

        jointEntity = model.JointByName(_ecm, joint_name);
        if (jointEntity == kNullEntity)
        {
            ignerr << "CustomMotorPlugin: Joint [" << joint_name << "] not found.\n";
            return;
        }

        std::string topic;
        if (_sdf->HasElement("topic")) {
            topic = _sdf->Get<std::string>("topic");
        } else {
            ignerr << "CustomMotorPlugin: topic needs to be specified \n";
            return;
        }

        ignmsg << "Trying to load Motor Plugin for: " << joint_name << "\n";

        node.Subscribe(topic, &CustomMotorPlugin::OnMsg, this);

        ignmsg << "BLDC Motor Plugin Loaded for Joint: " << joint_name << "\n";
    }

    void OnMsg(const msgs::Double &_msg)
    {
        duty_cycle = _msg.data();
        ignmsg << "Received duty cycle: " << duty_cycle << "\n";
        if(duty_cycle < -1 || duty_cycle > 1) {
            ignerr << "CustomMotorPlugin: duty is out of range\n";
        }
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        ignmsg << "PreUpdate called, duty_cycle: " << duty_cycle << "\n";
        
        dt = std::chrono::duration<double>(_info.dt).count();
        auto velComp = _ecm.Component<components::JointVelocity>(jointEntity);
        
        if ((duty_cycle > 0 && current < 0) || (duty_cycle < 0 && current > 0)) {
            current = 0;  // Reset current when direction changes
        }
        
        if (!velComp) {
            // Create the component if it doesn't exist
            ignmsg << "Creating JointVelocity component for joint\n";
            _ecm.CreateComponent(jointEntity, components::JointVelocity());
            return;  // Return for this iteration
        }


        double omega = velComp ? velComp->Data()[0] : 0.0;
        ignmsg << "Joint velocity: " << omega << "\n";

        double voltage = duty_cycle * max_voltage;
        double back_emf = ke * omega;
        double dI = (voltage - back_emf - (resistance * current)) / inductance;
        current += dI * dt;

        current = std::max(-max_current, std::min(max_current, current));

        double torque = kt * current;

        // Implement torque limiting
        // For short bursts (< 1 second), allow up to max_torque
        // For continuous operation, limit to rated_torque
        static double torque_integration = 0.0;
        static double last_high_torque_time = 0.0;
        double current_time = _info.simTime.count();
        
        // Simple torque limiting - can be made more sophisticated
        if (std::abs(torque) > rated_torque) {
            // If exceeding rated torque, limit how long it can stay there
            torque_integration += dt;
            last_high_torque_time = current_time;
            
            // If high torque has been maintained for too long, cap it
            if (torque_integration > 1.0) {  // 1 second limit for peak torque
                double torque_limit = rated_torque;
                torque = (torque > 0) ? torque_limit : -torque_limit;
            } else {
                // During burst period, still respect absolute max
                double torque_limit = max_torque;
                torque = std::max(-torque_limit, std::min(torque_limit, torque));
            }
        } else {
            // If we're below rated torque, gradually reset the integration
            // Allow recovery of burst capability after 3 seconds below rated torque
            if (current_time - last_high_torque_time > 3.0) {
                torque_integration = std::max(0.0, torque_integration - dt);
            }
        }

        // Apply torque to the joint
        _ecm.CreateComponent(jointEntity, components::JointForceCmd({torque}));
    }
};

} // namespace gazebo
} // namespace ignition

IGNITION_ADD_PLUGIN(
    ignition::gazebo::CustomMotorPlugin,
    ignition::gazebo::System,
    ignition::gazebo::CustomMotorPlugin::ISystemConfigure,
    ignition::gazebo::CustomMotorPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    ignition::gazebo::CustomMotorPlugin,
    "custom_motor_plugin")
