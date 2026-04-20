#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/bool.hpp>
// Import the custom message 
#include "xarm6_control_ros2/msg/plan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MainPlanner : public rclcpp::Node {

public:
  MainPlanner() : Node("main_planner") {
    // Define the publishers and subscribers
    plan_pub_ = this->create_publisher<xarm6_control_ros2::msg::Plan>("/plan", 10);
    
         
    // TODO: Make sure all the conversions to robot frame are done correctly in your ball detection code        
    ball_pos_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/ball_pos", 10, std::bind(&MainPlanner::get_ball_pos, this, _1));
    
    // This is needed for getting feedback from the robot position and switching between the states        
    rob_pos_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/xarm6/toolpose", 10, std::bind(&MainPlanner::get_rob_pos, this, _1));        

    

    // Create a timer to publish the plan every 0.1 seconds (10Hz)
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MainPlanner::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Main Planner ROS 2 Node Started. Waiting for ball pos...");
  }

private:



  void get_ball_pos(const geometry_msgs::msg::Twist::SharedPtr msg) {
      ball_x_ = msg->linear.x;
      ball_y_ = msg->linear.y;
      if (!ball_pos_received_) {
          RCLCPP_INFO(this->get_logger(), "Received ball position: x=%f, y=%f", ball_x_, ball_y_);
          ball_pos_received_ = true;
      }
  }
  
   void get_rob_pos(const geometry_msgs::msg::Twist::SharedPtr msg) {
      xyzrpy = *msg;
  }
  

  void open_gripper(){
   // DO NOT CHANGE THIS FUNCTION
    int returnCode = system("ros2 action send_goal /xarm_gripper/gripper_action control_msgs/action/GripperCommand \"{command: {position: 0.0, max_effort: 0.0}}\"");

    if (returnCode == 0) {
        std::cout << "Command executed successfully." << std::endl;
        gripper_done_ = true;
    } else {
        std::cout << "Command failed." << std::endl;
        gripper_done_ = false;
    }
  }
  
   void close_gripper(){
   // DO NOT CHANGE THIS FUNCTION
    int returnCode = system("ros2 action send_goal /xarm_gripper/gripper_action control_msgs/action/GripperCommand \"{command: {position: 0.8, max_effort: 0.0}}\"");

    if (returnCode == 0) {
        std::cout << "Command executed successfully." << std::endl;
        gripper_done_ = true;
    } else {
        std::cout << "Command failed." << std::endl;
        gripper_done_ = false;
    }
  }
  bool plan_executed(xarm6_control_ros2::msg::Plan plan_){
      // TODO: complete this function to check if the robot has reached the final point in the sub-plan
      return true;
  }
  void create_overall_plan() {
        // Approach Plan (TODO: YOU NEED TO UPDATE THESE TO COMPLETE THE TASK CORRECTLY)
        geometry_msgs::msg::Twist p;
        p.linear.x = 0.25; p.linear.y = 0.0; p.linear.z = 0.30;
        p.angular.x = -3.1415; p.angular.y = 0.0009; p.angular.z = -0.001;
        approach_plan_.points.push_back(p);
		// some dummy points just to create a short plan for now (TODO: UPDATE THIS)
        p.linear.x = 0.20; p.linear.y = 0.0; p.linear.z = 0.30;
        approach_plan_.points.push_back(p);
        // some dummy points just to create a short plan for now (TODO: UPDATE THIS)
        p.linear.x = 0.25; p.linear.y = 0.0; p.linear.z = 0.30;
        approach_plan_.points.push_back(p);
        // some dummy points just to create a short plan for now (TODO: UPDATE THIS)
         p.linear.x = 0.20; p.linear.y = 0.0; p.linear.z = 0.30;
        approach_plan_.points.push_back(p);

        // Drop Plan
        // TODO: YOU MUST UPDATE THIS WITH REASONABLE POINTS (THIS IS JUST HERE TO MAKE SURE THERE IS A NON-ZERO PLACEHOLDER)
        drop_plan_ = approach_plan_;
        
        // Retract Plan
        // TODO: YOU MUST UPDATE THIS WITH REASONABLE POINTS (THIS IS JUST HERE TO MAKE SURE THERE IS A NON-ZERO PLACEHOLDER)
        retract_plan_ = approach_plan_;

        overall_plan_created_ = true;
        state_ = 'a';
        RCLCPP_INFO(this->get_logger(), "Overall plan created. Starting APPROACH state.");
    }
  

  void control_loop()
  {
    if (!overall_plan_created_) {
            if (ball_pos_received_) create_overall_plan();
            return;
        }
		// use these characters of each state: 'a' for approach, 'c' for closing the gripper, 'd' for drop, 'o' for openning the gripper, 'r' for retracting, and 'f' for task completion (or finished)
        switch (state_) {
            case 'a': // Approach
                if (!plan_submitted_) {
                    plan_pub_->publish(approach_plan_);
                    plan_submitted_ = true;
                    
                }
                if (plan_executed(approach_plan_)) {
                    state_ = 'c'; plan_submitted_ = false;
                    RCLCPP_INFO(this->get_logger(), "Approach complete. Closing Gripper...");                
                    close_gripper();
                }
                break;

            case 'c': // Wait for Close
                if (gripper_done_) {
                    //TODO: COMPLETE THE REST USING THE EXAMPLE METHOD: state_ = '??';
                    RCLCPP_INFO(this->get_logger(), "Gripper closed. Starting DROP state.");
                }
                break;

            // TODO: remaining steps must go here
        }
  }

  // Member variables
   char state_ = 'i';
   // set the flags correctly
   bool ball_pos_received_ = false; 
   bool overall_plan_created_ = false;
   bool plan_submitted_ = false;
   bool gripper_done_ = false;
   float ball_x_, ball_y_;
   float ball_z_ = 0.02; // use this one for your project
   
   geometry_msgs::msg::Twist xyzrpy;
   
   xarm6_control_ros2::msg::Plan approach_plan_, drop_plan_, retract_plan_; // use these subplans effectively!
  
  rclcpp::Publisher<xarm6_control_ros2::msg::Plan>::SharedPtr plan_pub_;
  
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ball_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rob_pos_sub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Spin the node to keep the timer active
  rclcpp::spin(std::make_shared<MainPlanner>());
  rclcpp::shutdown();
  return 0;
}
