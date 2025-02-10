#include "rclcpp/rclcpp.hpp"
#include "impedance.hpp"

class ImpedanceNode : public rclcpp::Node
{
public:
    ImpedanceNode() : Node("impedance_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("topic_arm_state", "");
        this->declare_parameter<std::string>("topic_arm_command", "");
        this->declare_parameter<std::string>("topic_wrench_state", "");
        this->declare_parameter<std::vector<double>>("Ka", {});
        this->declare_parameter<std::vector<double>>("Kv", {});
        this->declare_parameter<std::vector<double>>("Kp", {});
        this->declare_parameter<std::vector<double>>("mass_arm", {});
        this->declare_parameter<std::vector<double>>("damping_arm", {});
        this->declare_parameter<std::vector<double>>("stiffness_coupling", {});
        this->declare_parameter<std::vector<double>>("desired_pose", {});

        try
        {
            topic_arm_state_ = this->get_parameter("topic_arm_state").as_string();
            topic_arm_command_ = this->get_parameter("topic_arm_command").as_string();
            topic_wrench_state_ = this->get_parameter("topic_wrench_state").as_string();
            Ka_ = this->get_parameter("Ka").as_double_array();
            Kv_ = this->get_parameter("Kv").as_double_array();
            Kp_ = this->get_parameter("Kp").as_double_array();
            M_ = this->get_parameter("mass_arm").as_double_array();
            D_ = this->get_parameter("damping_arm").as_double_array();
            K_ = this->get_parameter("stiffness_coupling").as_double_array();
            desired_pose_ = this->get_parameter("desired_pose").as_double_array();
        }
        catch (const rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve parameters: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Initialize and run impedance control
        impedance_.init(
            shared_from_this(),
            topic_arm_state_,
            topic_arm_command_,
            topic_wrench_state_,
            Ka_, Kv_, Kp_,
            M_, D_, K_,
            desired_pose_);
        
        impedance_.run();
    }

private:
    std::string topic_arm_state_;
    std::string topic_arm_command_;
    std::string topic_wrench_state_;
    std::vector<double> Ka_, Kv_, Kp_;
    std::vector<double> M_, D_, K_;
    std::vector<double> desired_pose_;

    Impedance impedance_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImpedanceNode>());
    rclcpp::shutdown();
    return 0;
}
