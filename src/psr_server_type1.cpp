#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <psr_smach/PSRAction.h>
#include <iostream>
#include <string>
#include <stdbool.h>

//using namespace std;

// Define struct 'Goal', 'State', and 'ModelParameter'
struct Goal{
	double duration; // unit = s
	//std::string steer_direction; // "left" or "right"
	float Wc; //angular speed of CoM, unit = rad/s
	float Vc; //linear speed of CoM, unit = m/s
};

struct State{
	// RObot states	
	float linear_speed; // unit = m/s
	float angular_speed; // unit = rad/s	
	// Wheel states	
	float angular_acceleration_left; // unit = rad/(s^2)
	float angular_acceleration_right; // unit = rad/(s^2)	
	float angular_speed_left; // unit = rad/s
	float angular_speed_right; // unit = rad/s
	float wheel_angle_left; // unit = rad/s
	float wheel_angle_right; // unit = rad/s
};

struct ModelParameter{
	// Parameters for unicycle model	
	float Rw; // Wheel radius, unit = m
	float width; // Wheel distance, unit = m
	float theta; // Body tilting angle, unit = rad
};



// Define class 'PSRAction'
class PSRAction{

protected:
    
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<psr_smach::PSRAction> as_;
	std::string action_name_;
	psr_smach::PSRFeedback feedback_;
	psr_smach::PSRResult result_;
	//psr_smach::PSRGoal goal_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	geometry_msgs::Twist psr_msg;
	
	boost::shared_ptr<const psr_smach::PSRGoal_<std::allocator<void> > > goal_;	
	//struct Goal goal_;
	struct State state_;
	struct ModelParameter model_para_;
	float Rs; // Steering radius, unit = m
	
	double initial_time; // unit = s
	double current_time; // unit = s
	double elapsed_time; // unit = s


public:
    
	PSRAction(std::string name) : 
		as_(nh_, name, false),
		action_name_(name)
	{
		//register the goal and feeback callbacks
		as_.registerGoalCallback(boost::bind(&PSRAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&PSRAction::preemptCB, this));

		//subscribe to the data topic of interest
		sub_ = nh_.subscribe("/PSR/sensors", 1, &PSRAction::algorithmCB, this); // In topic /PSR/sensors, a self-defined msg type will be applied, namely psr_msgs.  For test, try std_msgs::FLoat32
		pub_ = nh_.advertise<geometry_msgs::Twist>("/PSR/motors", 1);
		ROS_INFO("Start Server!!!");
		as_.start();
		ROS_INFO("Server has been started!!!");
/*
		// Algorithm
		while(ros::ok())
		{
			
		}
*/
	}

	~PSRAction(void)
	{
	}

	void goalCB()
	{
		//ROS_INFO("Goal received!!!");		
		// accept the new goal
		goal_ = as_.acceptNewGoal();
		ROS_INFO("Goal received!!! Duration = %f, Wc = %f, Vc = %f", goal_->duration, goal_->angular_speed, goal_->linear_speed);	
		//goal_->duration = as_.acceptNewGoal()->duration;
		// strcpy(goal_->steer_direction, as_.acceptNewGoal()->steer_direction);
		//goal_->angular_speed = as_.acceptNewGoal()->angular_speed;
		//goal_->linear_speed = as_.acceptNewGoal()->linear_speed;
		Rs = goal_->linear_speed / goal_->angular_speed; // unit = m

		// reset model parameters
		model_para_.Rw = 0.06; // unit = m
		model_para_.width = 0.12; // unit = m
		model_para_.theta = 0; // unit = m

		// reset state variables
		state_.linear_speed = goal_->linear_speed; // unit = m/s
		state_.angular_speed = goal_->angular_speed; // unit = rad/s
		state_.angular_acceleration_right = 0.0; // unit = rad/(s^2)
		state_.angular_speed_left = 0.0; // unit = rad/s
		state_.angular_speed_right = 0.0; // unit = rad/s
		state_.wheel_angle_left = 0.0; // unit = rad
		state_.wheel_angle_right = 0.0; // unit = rad
		
		// reset initial time
		initial_time = ros::WallTime::now().toSec();
		current_time = initial_time;
		elapsed_time = 0.0;
	}

	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}

	void algorithmCB(const std_msgs::Float32::ConstPtr& sensor)
	{
		//ROS_INFO("Theta = %f", sensor);		
		// make sure that the action hasn't been canceled
		if (!as_.isActive())
		return;
		
		ROS_INFO("Theta = %f", sensor->data);	
		// Algorithm

		// Compute left and right angular speeds
		state_.angular_speed_left = (1-(model_para_.width/(2.0*Rs)))*(goal_->linear_speed/model_para_.Rw);
		state_.angular_speed_right = (1+(model_para_.width/(2.0*Rs)))*(goal_->linear_speed/model_para_.Rw);
		
		// Publish them
		psr_msg.linear.x = state_.wheel_angle_left;
		psr_msg.linear.y = state_.wheel_angle_right;
		psr_msg.angular.x = state_.angular_speed_left;
		psr_msg.angular.y = state_.angular_speed_right;
		pub_.publish(psr_msg);
		
		feedback_.current_time = elapsed_time;
		feedback_.angular_speed_left = state_.angular_speed_left;
		feedback_.angular_speed_right = state_.angular_speed_right;
		as_.publishFeedback(feedback_);

		// Check action process		
		current_time = ros::WallTime::now().toSec();
		elapsed_time = current_time - initial_time;
		ROS_INFO("Elapsed time = %f", elapsed_time);	
	
		if(elapsed_time >= goal_->duration){
			result_.completed = true;
			result_.angular_speed = state_.angular_speed;
			result_.linear_speed = state_.linear_speed;
			// Set speed to zeros
			psr_msg.angular.x = 0.0;
			psr_msg.angular.y = 0.0;
			pub_.publish(psr_msg);

			if(result_.completed == false){
        			ROS_INFO("%s: Aborted", action_name_.c_str());
        			//set the action state to aborted
				as_.setAborted(result_);
			}
			else{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
        			// set the action state to succeeded
        			as_.setSucceeded(result_);
			}
		} 
	}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psr_type1");

  PSRAction psr_go(ros::this_node::getName());
  ros::spin();

  return 0;
}
