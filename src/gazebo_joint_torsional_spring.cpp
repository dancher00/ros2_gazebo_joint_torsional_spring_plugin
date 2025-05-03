#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
<<<<<<< HEAD
  class TorsionalSpringPlugin : public ModelPlugin
  {
  public:
    TorsionalSpringPlugin() {}

  private:
    physics::ModelPtr model;
    sdf::ElementPtr sdf;
    physics::JointPtr joint;
    // Set point
    double setPoint;
    // Spring constant
    double kx;
    // Pointer to update event connection
    event::ConnectionPtr updateConnection;
    // ROS2 node
    std::shared_ptr<rclcpp::Node> rosNode;

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "You have zero joints! Something is wrong! Not loading plugin." << std::endl;
        return;
      }

      // Store model pointer
      this->model = _model;
      // Store the SDF pointer
      this->sdf = _sdf;

      // Initialize ROS2, if needed
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      // Get joint name first
      std::string joint_name = "";
      if (_sdf->HasElement("joint"))
        joint_name = _sdf->Get<std::string>("joint");
      else
        std::cerr << "Must specify joint to apply a torsional spring at!\n";

      // Create ROS2 node with unique name using joint name
      std::string model_name = _model->GetName();
      this->rosNode = std::make_shared<rclcpp::Node>(model_name + "_" + joint_name + "_torsional_spring_plugin");

      // Get joint
      if (_sdf->HasElement("joint"))
        this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
      else
        std::cerr << "Must specify joint to apply a torsional spring at!\n";

      // Get kx parameter
      this->kx = 0.0;
      if (_sdf->HasElement("kx"))
        this->kx = _sdf->Get<double>("kx");
      else
        RCLCPP_INFO(this->rosNode->get_logger(),
                   "Torsional spring coefficient not specified! Defaulting to: %f",
                   this->kx);

      // Get set point parameter
      this->setPoint = 0.0;
      if (_sdf->HasElement("set_point"))
        this->setPoint = _sdf->Get<double>("set_point");  // Fixed typo: was "set*point"
      else
        RCLCPP_INFO(this->rosNode->get_logger(),
                   "Set point not specified! Defaulting to: %f",
                   this->setPoint);

      RCLCPP_INFO(this->rosNode->get_logger(), "Loaded gazebo_joint_torsional_spring for ROS2.");

      // Listen to update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TorsionalSpringPlugin::OnUpdate, this));
    }

  protected:
    void OnUpdate()
    {
      // Spin ROS2 node once
      rclcpp::spin_some(this->rosNode);

      if (this->joint)
      {
        double current_angle = this->joint->Position(0);
        this->joint->SetForce(0, this->kx * (this->setPoint - current_angle));
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
=======
    class TorsionalSpringPlugin : public ModelPlugin
    {
    public:
        TorsionalSpringPlugin() {}

    private:
        physics::ModelPtr model;
        sdf::ElementPtr sdf;
        
        physics::JointPtr joint;
        // Set point
        double setPoint;
        // Spring constant
        double kx;
        // Pointer to update event connection
        event::ConnectionPtr updateConnection;
        
        // ROS2 node
        std::shared_ptr<rclcpp::Node> rosNode;

    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "You have zero joints! Something is wrong! Not loading plugin." << std::endl;
                return;
            }
            
            // Store model pointer
            this->model = _model;
            // Store the SDF pointer
            this->sdf = _sdf;
            
            // Initialize ROS2, if needed
            if (!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }
            
            // Create ROS2 node
            std::string model_name = _model->GetName();
            this->rosNode = std::make_shared<rclcpp::Node>(model_name + "_torsional_spring_plugin");
            
            // Get joint
            if (_sdf->HasElement("joint"))
                this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
            else
                std::cerr << "Must specify joint to apply a torsional spring at!\n";
            
            // Get kx parameter
            this->kx = 0.0;
            if (_sdf->HasElement("kx"))
                this->kx = _sdf->Get<double>("kx");
            else
                RCLCPP_INFO(this->rosNode->get_logger(), 
                           "Torsional spring coefficient not specified! Defaulting to: %f", 
                           this->kx);
            
            // Get set point parameter
            this->setPoint = 0.0;
            if (_sdf->HasElement("set_point"))
                this->setPoint = _sdf->Get<double>("set_point");
            else
                RCLCPP_INFO(this->rosNode->get_logger(),
                           "Set point not specified! Defaulting to: %f",
                           this->setPoint);
            
            RCLCPP_INFO(this->rosNode->get_logger(), "Loaded gazebo_joint_torsional_spring for ROS2.");
            
            // Listen to update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&TorsionalSpringPlugin::OnUpdate, this));
        }

    protected:
        void OnUpdate()
        {
            // Spin ROS2 node once
            rclcpp::spin_some(this->rosNode);
            
            if (this->joint)
            {
                double current_angle = this->joint->Position(0);
                this->joint->SetForce(0, this->kx * (this->setPoint - current_angle));
            }
        }
    };
    
    GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
>>>>>>> 975f086febd0c4c25b062c9ffeddc00e6af7f2dc
}