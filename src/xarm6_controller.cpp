#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// KDL remains mostly the same, but check includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>

class XArm6Controller : public rclcpp::Node {
public:
    XArm6Controller() : Node("xarm6_controller_node") {
        // Parameters
        this->declare_parameter("sim", true);
        bool sim = this->get_parameter("sim").as_bool();

        // Topic names (Updated for ROS 2 defaults)
        std::string command_topic = sim ? 
            "/xarm6_traj_controller/joint_trajectory" : 
            "/scaled_xarm6_traj_controller/joint_trajectory";

        // Publishers
        cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(command_topic, 10);
        xyzrpy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/xarm6/toolpose", 10);

        // Subscribers
        joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&XArm6Controller::joint_state_callback, this, std::placeholders::_1));
        
        ref_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/reftraj", 10, std::bind(&XArm6Controller::ref_callback, this, std::placeholders::_1));

        // TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        

        // Timer for the loop (replaces ros::Rate)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&XArm6Controller::control_loop, this));
    }

private:
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Map joint names to KDL indices (assumes order, but ideally find by name)
        
        std::vector<std::string> expected_order = {
        	"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    	};

    	int found_count = 0;
    	for (size_t i = 0; i < msg->name.size(); ++i) {
        	for (size_t j = 0; j < expected_order.size(); ++j) {
            	if (msg->name[i] == expected_order[j]) {
                	std::cout <<"Read this "<<msg->position[i]<< " for joint"<< j+1 << "\n";
                	found_count++;
            	}
        	}
    	}

    	if (found_count == 6) {
        	joints_initialized_ = true;
    	}
        
        
        
    }

    void ref_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_ref_ = *msg;
        ref_received_ = true;
    }

    void control_loop() {
        if (!joints_initialized_){std::cout <<"have not received the joint coordinates yet!\n"; return;}

        KDL::Frame cart_pos;
        
    }

    

   
    // Members
    
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr xyzrpy_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist current_ref_;
    bool joints_initialized_ = false;
    bool ref_received_ = false;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XArm6Controller>());
    rclcpp::shutdown();
    return 0;
}
