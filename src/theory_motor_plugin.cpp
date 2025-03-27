#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
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
    
    // Motor specifications from datasheet
    const double max_voltage = 48.0;           // V
    const double resistance = 170.0e-3;        // mΩ (phase to phase)
    const double inductance = 57.0e-6;         // μH (phase to phase)
    const double ke = 0.1002;                  // V/(rad/s)
    const double kt = 0.105;                   // Nm/A
    const double km = 0.25;                    // Nm/√W (torque constant)
    const double max_current = 22.3;           // A
    const double rated_current = 10.3;         // A
    const double rated_torque = 9.0;           // Nm
    const double max_torque = 18.0;            // Nm (peak torque)
    const double rated_speed = 390.0 * (2*M_PI/60.0); // rad/s (converted from rpm)
    const double motor_inertia = 807.0e-7;     // kg·m² (converted from gcm²)
    const double mech_time_constant = 0.94e-3; // s
    const double elec_time_constant = 0.34e-3; // s
    const int pole_pairs = 21;                 // Number of pole pairs
    
    // Gearbox parameters
    const double gear_ratio = 9.0;             // 9:1 reduction
    const double gearbox_efficiency = .9;    // 90% efficiency
    const double backdrive_friction = 0.51;    // Nm
    const double backlash_angle = 0.19 * (M_PI/180.0); // rad (converted from degrees)
    
    // State variables
    double dt = 0.001;
    double current = 0.0;
    double duty_cycle = 0.0;
    double previous_position = 0.0;
    double effective_position = 0.0;
    bool backlash_region = false;
    
    // Temperature modeling
    const double ambient_temp = 25.0;          // °C
    const double max_temp = 50.0;              // °C
    const double thermal_resistance = 1.0;     // °C/W (estimated)
    double motor_temp = ambient_temp;
    
    // Torque limiting variables
    double torque_integration = 0.0;
    double last_high_torque_time = 0.0;

    //used for making sure duty spins it in the right direction
    bool invert_direction = false;

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

        if (_sdf->HasElement("invert_direction"))
        {
            invert_direction = _sdf->Get<bool>("invert_direction");
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

        // Initialize position tracking variables
        auto posComp = _ecm.Component<components::JointPosition>(jointEntity);
        if (posComp) {
            previous_position = posComp->Data()[0];
            effective_position = previous_position;
        }

        ignmsg << "BLDC Motor Plugin Loaded for Joint: " << joint_name << "\n";
    }

    void OnMsg(const msgs::Double &_msg)
    {
        duty_cycle = _msg.data();

        if(invert_direction) {
            duty_cycle = -duty_cycle;
        }

        ignmsg << "Received duty cycle: " << duty_cycle << "\n";
        if(duty_cycle < -1 || duty_cycle > 1) {
            ignerr << "CustomMotorPlugin: duty is out of range\n";
            duty_cycle = std::max(-1.0, std::min(1.0, duty_cycle));
        }
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        dt = std::chrono::duration<double>(_info.dt).count();
        
        // Get joint velocity and position
        auto velComp = _ecm.Component<components::JointVelocity>(jointEntity);
        auto posComp = _ecm.Component<components::JointPosition>(jointEntity);
        
        if (!velComp || !posComp) {
            // Create components if they don't exist
            if (!velComp) _ecm.CreateComponent(jointEntity, components::JointVelocity());
            if (!posComp) _ecm.CreateComponent(jointEntity, components::JointPosition());
            return;  // Return for this iteration
        }

        double wheel_omega = velComp->Data()[0];
        double current_position = posComp->Data()[0];
        
        // Handle backlash
        double position_change = current_position - previous_position;
        previous_position = current_position;
        
        // Implement backlash model
        if (std::abs(position_change) > 0) {
            if (backlash_region) {
                // In backlash region
                effective_position += position_change;
                if (std::abs(effective_position) > backlash_angle/2) {
                    // Exit backlash region
                    backlash_region = false;
                    wheel_omega = position_change / dt; // Estimate velocity
                } else {
                    wheel_omega = 0.0; // No effective motion during backlash
                }
            } else if (position_change * wheel_omega < 0) {
                // Direction change - enter backlash region
                backlash_region = true;
                effective_position = 0;
                wheel_omega = 0.0;
            }
        }
        
        // Calculate motor side with gear ratio
        double motor_omega = wheel_omega * gear_ratio;
        
        // Apply backdrive friction when there's no command but joint is moving
        double backdrive_torque = 0.0;
        if (std::abs(duty_cycle) < 0.05 && std::abs(wheel_omega) > 0.01) {
            backdrive_torque = -backdrive_friction * std::tanh(wheel_omega * 10.0);
            // tanh provides smooth transition around zero
        }
        
        // Motor electrical model
        double voltage = duty_cycle * max_voltage;
        double back_emf = ke * motor_omega;
        double dI = (voltage - back_emf - (resistance * current)) / inductance;
        current += dI * dt;
        
        // Current limiting
        current = std::max(-max_current, std::min(max_current, current));
        
        // Motor torque calculation considering electrical and mechanical characteristics
        double motor_torque = kt * current;
        
        // Temperature modeling - simplified
        double power_loss = resistance * current * current; // I²R losses
        double delta_temp = (power_loss * thermal_resistance - (motor_temp - ambient_temp) * 0.1) * dt;
        motor_temp += delta_temp;
        
        // Temperature derating (if motor gets too hot)
        double temp_derating = 1.0;
        if (motor_temp > max_temp) {
            temp_derating = std::max(0.5, 1.0 - (motor_temp - max_temp) * 0.02);
            motor_torque *= temp_derating;
        }
        
        // Torque transfer through gearbox
        double wheel_torque = (motor_torque * gear_ratio * gearbox_efficiency) + backdrive_torque;
        
        // Implement torque limiting with time integration
        double current_time = _info.simTime.count();
        
        if (std::abs(wheel_torque) > rated_torque) {
            torque_integration += dt;
            last_high_torque_time = current_time;
            
            if (torque_integration > 1.0) {  // 1 second limit for peak torque
                double torque_limit = rated_torque;
                wheel_torque = (wheel_torque > 0) ? torque_limit : -torque_limit;
            } else {
                double torque_limit = max_torque;
                wheel_torque = std::max(-torque_limit, std::min(torque_limit, wheel_torque));
            }
        } else {
            if (current_time - last_high_torque_time > 3.0) {
                torque_integration = std::max(0.0, torque_integration - dt);
            }
        }
        
        // Apply torque to joint
        _ecm.CreateComponent(jointEntity, components::JointForceCmd({wheel_torque}));
        
        // For debugging
        ignmsg << "Motor stats: omega=" << motor_omega 
               << " current=" << current 
               << " torque=" << wheel_torque 
               << " temp=" << motor_temp << "°C\n";
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