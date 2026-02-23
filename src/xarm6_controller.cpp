#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// KDL 
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




		chain_ = create_xarm6_chain();
		// get the number of joints from the chain
		no_of_joints = chain_.getNrOfJoints();
		RCLCPP_INFO(this->get_logger(), "Chain joints: %d", no_of_joints); // MUST BE 6

		// Initialize solvers 
		fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
		ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
		ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(chain_, *fk_solver_, *ik_vel_solver_, 100, 1e-4);
        
        
		// define a joint array in KDL format for the joint positions
    	q_current_ = KDL::JntArray(no_of_joints);
		// define a joint array in KDL format for the next joint positions		
        q_next_ = KDL::JntArray(no_of_joints);

		std::cout << "number of elements of q and q_n:"<<q_current_.data.size()<< " ---"<<q_next_.data.size()<<"\n";

        // Timer for the loop 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&XArm6Controller::control_loop, this));
    }

private:
	KDL::Chain create_xarm6_chain() {
		float gripper_length = 0.155;
        KDL::Chain chain;
        // DH parameters 
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0, 0, 0, 0)));
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, -M_PI_2, 0.267, 0)));
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.28948, 0, 0, -1.385)));
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0775, -M_PI_2, 0, 1.385)));
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, M_PI_2, 0.3425, 0)));
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.076, -M_PI_2, 0, 0)));
        chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, 0, 0.097 + gripper_length, 0)));
        return chain;
    }
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Map joint names to KDL indices (the gripper messes up the order!)
        
        std::vector<std::string> expected_order = {
        	"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    	};

    	int found_count = 0;
    	for (size_t i = 0; i < msg->name.size(); ++i) {
        	for (size_t j = 0; j < expected_order.size(); ++j) {
            	if (msg->name[i] == expected_order[j]) {
            		q_current_(j) = msg->position[i];
                	//std::cout <<"Read this "<<msg->position[i]<< " for joint"<< j+1 << "\n";
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
        //std::cout << "received: "<< current_ref_.linear.x<< "  "<< current_ref_.linear.y<< "  "<< current_ref_.linear.z<< "  "<< current_ref_.angular.x<< "  "<<  current_ref_.angular.y<< "  "<<  current_ref_.angular.z<< " \n";
    }
    
    geometry_msgs::msg::Twist update_xyzrpy(KDL::Frame _cartpos){
		// define roll, pitch, yaw variables
		double roll, pitch, yaw;
		//extract the roll, pitch, yaw from the KDL frame after the fk calculations
		_cartpos.M.GetRPY(roll,pitch, yaw);
		geometry_msgs::msg::Twist _xyzrpy;
		// update and return the values
		_xyzrpy.linear.x = _cartpos.p[0];
		_xyzrpy.linear.y = _cartpos.p[1];
		_xyzrpy.linear.z = _cartpos.p[2];
		_xyzrpy.angular.x = roll;
		_xyzrpy.angular.y = pitch;
		_xyzrpy.angular.z = yaw;
		return _xyzrpy;
	}
	
	void send_trajectory() {
        trajectory_msgs::msg::JointTrajectory msg;
       
        msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
		
        trajectory_msgs::msg::JointTrajectoryPoint point;
        // ensure the ranges are correct and the joints do not flip!
        for (int i = 0; i < 6; ++i) {
        	while(q_next_(i) > M_PI)
				q_next_(i) -= 2*M_PI;
		 	while(q_next_(i) < -M_PI)
				q_next_(i) += 2*M_PI;
        	point.positions.push_back(q_next_(i));
        }
        // must match this to the loop rate.
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        msg.points.push_back(point);
        cmd_pub_->publish(msg);
    }


    void control_loop() {
        if (!joints_initialized_){std::cout <<"have not received the joint coordinates yet!\n"; return;}

        KDL::Frame cart_pos;
        // flag for the fk results
		if (fk_solver_->JntToCart(q_current_, cart_pos) >= 0) {
			xyzrpy = update_xyzrpy(cart_pos);
			xyzrpy_pub_->publish(xyzrpy);
			publish_tf(cart_pos);
			
		}// end of FK
		
		
	    // If reference received, calculate IK and publish
        if (ref_received_) {
             KDL::Frame target_frame;
             target_frame.p = KDL::Vector(current_ref_.linear.x, current_ref_.linear.y, current_ref_.linear.z);
             target_frame.M = KDL::Rotation::RPY(current_ref_.angular.x, current_ref_.angular.y, current_ref_.angular.z);
             // solve IK and give the next joint coordinates
             if (ik_pos_solver_->CartToJnt(q_current_, target_frame, q_next_) >= 0) {
                  send_trajectory();
             }
        }


    }

	
	void publish_tf(KDL::Frame & frame) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "fk_tooltip";
        t.transform.translation.x = frame.p.x();
        t.transform.translation.y = frame.p.y();
        t.transform.translation.z = frame.p.z();
        
        double r, p, y;
        frame.M.GetRPY(r, p, y);
        tf2::Quaternion q;
        q.setRPY(r, p, y);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    

   
    
    // Members
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;
    KDL::JntArray q_current_, q_next_;
    unsigned int no_of_joints;
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr xyzrpy_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist current_ref_;
    bool joints_initialized_ = false;
    bool ref_received_ = false;
    geometry_msgs::msg::Twist xyzrpy;
    
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XArm6Controller>());
    rclcpp::shutdown();
    return 0;
}
