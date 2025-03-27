#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/transport/Node.hh>
// Changed from Double to Float message
#include <ignition/msgs/float.pb.h>
#include <ignition/plugin/Register.hh>

#include <map>
#include <string>
#include <cmath>
#include <mutex>

namespace ignition
{
namespace gazebo
{

/// \brief A minimal data-driven motor controller plugin that uses empirical
/// data to determine the relationship between duty cycle and motor output.
class DataMotor : public ignition::gazebo::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate
{
  private:
    // Transport node for communication
    ignition::transport::Node node;
    
    // Joint entity this plugin controls
    Entity jointEntity = kNullEntity;
    
    // Current duty cycle command (protected by mutex for thread safety)
    double duty_cycle = 0.0;
    std::mutex duty_cycle_mutex;
    
    // Mapping from duty to speed (rad/s)
    std::map<double, double> dutyToSpeed;
    
    // Controller parameters
    double speed_kp = 3.5;
    double max_torque = 18.0;
    
    // Motor direction
    bool invert_direction = false;
    
    // Component created flags
    bool velocity_comp_created = false;
    bool position_comp_created = false;
    
    // Iteration counter
    int iteration_count = 0;
    
  public:
    /// \brief Configure the plugin
    void Configure(const Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  EntityComponentManager &_ecm,
                  EventManager &) override
    {
      // First make sure the model is valid
      Model model(_entity);
      if (!model.Valid(_ecm))
      {
        ignerr << "DataMotor: Model entity is invalid.\n";
        return;
      }

      // Get joint name (required)
      std::string joint_name;
      if (_sdf->HasElement("joint_name"))
      {
        joint_name = _sdf->Get<std::string>("joint_name");
      }
      else
      {
        ignerr << "DataMotor: joint_name is required.\n";
        return;
      }
      
      // Get topic name (required)
      std::string topic;
      if (_sdf->HasElement("topic"))
      {
        topic = _sdf->Get<std::string>("topic");
      }
      else
      {
        ignerr << "DataMotor: topic is required.\n";
        return;
      }

      // Optional parameter
      if (_sdf->HasElement("invert_direction"))
        invert_direction = _sdf->Get<bool>("invert_direction");
      
      // Find joint entity
      jointEntity = model.JointByName(_ecm, joint_name);
      if (jointEntity == kNullEntity)
      {
        ignerr << "DataMotor: Joint [" << joint_name << "] not found.\n";
        return;
      }
      
      // Load default empirical data points
      dutyToSpeed[0.0] = 0.0;
      dutyToSpeed[0.1] = 5.0;   // Duty 0.1 → 5 rad/s
      dutyToSpeed[0.2] = 11.0;  // Duty 0.2 → 11 rad/s
      dutyToSpeed[0.3] = 17.0;  // Duty 0.3 → 17 rad/s
      
      // Mirror for negative values
      dutyToSpeed[-0.1] = -5.0;
      dutyToSpeed[-0.2] = -11.0;
      dutyToSpeed[-0.3] = -17.0;
      
      // Changed to use Float message type instead of Double
      node.Subscribe(topic, &DataMotor::OnMsg, this);
      
      ignmsg << "DataMotor plugin loaded for joint: " << joint_name << "\n";
      ignmsg << "Using empirical mapping for duty cycle to speed\n";
    }

    /// \brief Callback for receiving duty cycle commands
    // Changed to use Float message type instead of Double
    void OnMsg(const msgs::Float &_msg)
    {
      std::lock_guard<std::mutex> lock(duty_cycle_mutex);
      
      duty_cycle = _msg.data();
      
      // Clamp duty cycle to valid range
      duty_cycle = std::max(-1.0, std::min(1.0, duty_cycle));
      
      if (invert_direction)
        duty_cycle = -duty_cycle;
        
      igndbg << "Received duty cycle: " << duty_cycle << "\n";
    }

    /// \brief Pre-update callback
    void PreUpdate(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override
    {
      // Skip if joint entity is invalid
      if (jointEntity == kNullEntity)
        return;
        
      // Ensure components exist - do this carefully to avoid race conditions
      if (!velocity_comp_created)
      {
        if (!_ecm.Component<components::JointVelocity>(jointEntity))
        {
          _ecm.CreateComponent(jointEntity, components::JointVelocity({0}));
        }
        velocity_comp_created = true;
        return; // Wait a cycle for component to be fully created
      }
      
      if (!position_comp_created)
      {
        if (!_ecm.Component<components::JointPosition>(jointEntity))
        {
          _ecm.CreateComponent(jointEntity, components::JointPosition({0}));
        }
        position_comp_created = true;
        return; // Wait a cycle for component to be fully created
      }
      
      // Skip first few iterations to ensure everything is initialized
      if (iteration_count < 5)
      {
        iteration_count++;
        return;
      }
      
      // Safely get current duty cycle
      double current_duty;
      {
        std::lock_guard<std::mutex> lock(duty_cycle_mutex);
        current_duty = duty_cycle;
      }
      
      // Safely get components
      auto velComp = _ecm.Component<components::JointVelocity>(jointEntity);
      if (!velComp) return;
      
      // Carefully access velocity data
      const auto &velData = velComp->Data();
      if (velData.empty()) return;
      
      double current_velocity = velData[0];
      
      // Get target speed based on duty cycle
      double target_speed = InterpolateDutyToSpeed(current_duty);
      
      // Simple proportional control
      double speed_error = target_speed - current_velocity;
      double torque = speed_kp * speed_error;
      
      // Limit torque
      torque = std::max(-max_torque, std::min(max_torque, torque));
      
      // Apply torque to joint
      _ecm.CreateComponent(jointEntity, components::JointForceCmd({torque}));
      
      // Log occasionally (every 500 iterations to avoid spam)
      static int log_counter = 0;
      if (++log_counter >= 500)
      {
        log_counter = 0;
        igndbg << "Motor stats: duty=" << current_duty
               << " velocity=" << current_velocity
               << " target=" << target_speed
               << " torque=" << torque << "\n";
      }
    }
    
  private:
    /// \brief Interpolate duty cycle to speed
    double InterpolateDutyToSpeed(double duty)
    {
      // Zero duty = zero speed
      if (std::abs(duty) < 1e-6)
        return 0.0;
        
      // Out of range handling
      if (duty <= dutyToSpeed.begin()->first)
        return dutyToSpeed.begin()->second;
      
      if (duty >= dutyToSpeed.rbegin()->first)
        return dutyToSpeed.rbegin()->second;
      
      // Find nearest points and interpolate
      auto high = dutyToSpeed.lower_bound(duty);
      if (high == dutyToSpeed.begin())
        return high->second;
        
      auto low = std::prev(high);
      
      // Linear interpolation
      double t = (duty - low->first) / (high->first - low->first);
      return low->second + t * (high->second - low->second);
    }
};

}  // namespace gazebo
}  // namespace ignition

// Register plugin
IGNITION_ADD_PLUGIN(
    ignition::gazebo::DataMotor,
    ignition::gazebo::System,
    ignition::gazebo::DataMotor::ISystemConfigure,
    ignition::gazebo::DataMotor::ISystemPreUpdate)

// Add alias
IGNITION_ADD_PLUGIN_ALIAS(
    ignition::gazebo::DataMotor,
    "data_motor")