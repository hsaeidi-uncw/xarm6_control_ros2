#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <xarm6_control_ros2/msg/plan.hpp> 


#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#define CYCLE_TIME_IN_SECONDS 0.1
#define NUMBER_OF_DOFS 6

class TaskSpaceTrajNode : public rclcpp::Node {
public:
    TaskSpaceTrajNode() : Node("task_space_traj") {
        // Publishers
        reflexxes_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/reftraj", 1);

        // Subscribers
        plan_sub_ = this->create_subscription<xarm6_control_ros2::msg::Plan>(
            "/plan", 1, std::bind(&TaskSpaceTrajNode::get_plan, this, std::placeholders::_1));
        
        pos_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/xarm6/tool_pose", 1, std::bind(&TaskSpaceTrajNode::get_pos, this, std::placeholders::_1));

        // Initialize Reflexxes API with smart pointers
        rml_ = std::make_unique<ReflexxesAPI>(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
        ip_ = std::make_unique<RMLPositionInputParameters>(NUMBER_OF_DOFS);
        op_ = std::make_unique<RMLPositionOutputParameters>(NUMBER_OF_DOFS);

        // Control Timer (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TaskSpaceTrajNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Task Space Traj Node started with Reflexxes.");
    }

private:
    void get_pos(const geometry_msgs::msg::Twist::SharedPtr msg) {
        rob_pos_ = *msg;
        rob_pos_received_ = true;
    }

    void get_plan(const xarm6_control_ros2::msg::Plan::SharedPtr msg) {
        plan_ = *msg;
        plan_available_ = true;
        number_of_points_ = plan_.points.size();
        initialize_plan_parameters();
        RCLCPP_INFO(this->get_logger(), "New plan received. Starting motion.");
    }

    void initialize_plan_parameters() {
        if (plan_.points.empty()) return;

        // Set Initial Position from the first point of the plan
        ip_->CurrentPositionVector->VecData[0] = plan_.points[0].linear.x;
        ip_->CurrentPositionVector->VecData[1] = plan_.points[0].linear.y;
        ip_->CurrentPositionVector->VecData[2] = plan_.points[0].linear.z;
        ip_->CurrentPositionVector->VecData[3] = plan_.points[0].angular.x;
        ip_->CurrentPositionVector->VecData[4] = plan_.points[0].angular.y;
        ip_->CurrentPositionVector->VecData[5] = plan_.points[0].angular.z;

        // Initialize Dynamics to zero and set constraints
        for (int i = 0; i < NUMBER_OF_DOFS; ++i) {
            ip_->CurrentVelocityVector->VecData[i] = 0.0;
            ip_->CurrentAccelerationVector->VecData[i] = 0.0;
            ip_->MaxVelocityVector->VecData[i] = 0.1;
            ip_->MaxAccelerationVector->VecData[i] = 0.1;
            ip_->MaxJerkVector->VecData[i] = 0.1;
            ip_->SelectionVector->VecData[i] = true;
        }

        // Set target to index 1
        if (plan_.points.size() > 1) {
            set_target(1);
        }
        result_value_ = 0; 
    }

    void set_target(int index) {
        ip_->TargetPositionVector->VecData[0] = plan_.points[index].linear.x;
        ip_->TargetPositionVector->VecData[1] = plan_.points[index].linear.y;
        ip_->TargetPositionVector->VecData[2] = plan_.points[index].linear.z;
        ip_->TargetPositionVector->VecData[3] = plan_.points[index].angular.x;
        ip_->TargetPositionVector->VecData[4] = plan_.points[index].angular.y;
        ip_->TargetPositionVector->VecData[5] = plan_.points[index].angular.z;

        for (int i = 0; i < NUMBER_OF_DOFS; ++i) {
            ip_->TargetVelocityVector->VecData[i] = 0.0;
        }
    }

    void control_loop() {
        if (!plan_available_) return;

        if (result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED) {
            result_value_ = rml_->RMLPosition(*ip_, op_.get(), flags_);

            if (result_value_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "Reflexxes Error code: %d", result_value_);
                return;
            }

            if (result_value_ == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
                int next_wp = ctr_ % number_of_points_;
                set_target(next_wp);

                if (rob_pos_received_) {
                    ip_->CurrentPositionVector->VecData[0] = rob_pos_.linear.x;
                    ip_->CurrentPositionVector->VecData[1] = rob_pos_.linear.y;
                    ip_->CurrentPositionVector->VecData[2] = rob_pos_.linear.z;
                    ip_->CurrentPositionVector->VecData[3] = rob_pos_.angular.x;
                    ip_->CurrentPositionVector->VecData[4] = rob_pos_.angular.y;
                    ip_->CurrentPositionVector->VecData[5] = rob_pos_.angular.z;
                }
                ctr_++;
                result_value_ = rml_->RMLPosition(*ip_, op_.get(), flags_);
            }

            *ip_->CurrentPositionVector = *op_->NewPositionVector;
            *ip_->CurrentVelocityVector = *op_->NewVelocityVector;
            *ip_->CurrentAccelerationVector = *op_->NewAccelerationVector;

            geometry_msgs::msg::Twist ref;
            ref.linear.x = ip_->CurrentPositionVector->VecData[0];
            ref.linear.y = ip_->CurrentPositionVector->VecData[1];
            ref.linear.z = ip_->CurrentPositionVector->VecData[2];
            ref.angular.x = ip_->CurrentPositionVector->VecData[3];
            ref.angular.y = ip_->CurrentPositionVector->VecData[4];
            ref.angular.z = ip_->CurrentPositionVector->VecData[5];
            reflexxes_pub_->publish(ref);
        }
    }

    // Member Variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr reflexxes_pub_;
    rclcpp::Subscription<xarm6_control_ros2::msg::Plan>::SharedPtr plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<ReflexxesAPI> rml_;
    std::unique_ptr<RMLPositionInputParameters> ip_;
    std::unique_ptr<RMLPositionOutputParameters> op_;
    RMLPositionFlags flags_;
    int result_value_ = 0;

    xarm6_control_ros2::msg::Plan plan_;
    geometry_msgs::msg::Twist rob_pos_;
    bool plan_available_ = false;
    bool rob_pos_received_ = false;
    int number_of_points_ = 0;
    int ctr_ = 2;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskSpaceTrajNode>());
    rclcpp::shutdown();
    return 0;
}
