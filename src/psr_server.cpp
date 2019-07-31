#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <psr_smach/PSRAction.h>
#include <iostream>
#include <string>
//using namespace std;

// Define struct 'Goal', 'State', and 'Parameter'
struct Goal{
	float duration; // unit = s
	std::string steer_direction; // "left" or "right"
	float steer_radius; // unit = m
	float linear_speed; // unit = m/s
};

struct State{
	float angular_acceleration_left; // unit = rad/(s^2)
	float angular_acceleration_right; // unit = rad/(s^2)
	float wheel_angle_left; // unit = rad/s
	float wheel_angle_right; // unit = rad/s	
	float angular_speed_left; // unit = rad/s
	float angular_speed_right; // unit = rad/s
	float wheel_angle_left; // unit = rad/s
	float wheel_angle_right; // unit = rad/s
};

struct Parameter{
	float 
};



// Define class 'PSRAction'
class PSRAction{

protected:
    
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<psr_smach::PSRAction> as_;
	std::string action_name_;
	psr_smach::PSRFeedback feedback_;
	psr_smach::PSRResult result_;
	ros::Subscriber sub_;
	ros::Pulisher pub_;
		
	struct Goal goal_;


public:
    
	PSRAction(std::string name) : 
		as_(nh_, name, false),
		action_name_(name)
	{
		//register the goal and feeback callbacks
		as_.registerGoalCallback(boost::bind(&PSRAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&PSRAction::preemptCB, this));

		//subscribe to the data topic of interest
		sub_ = nh_.subscribe("/random_number", 1, &PSRAction::analysisCB, this);
		pub_ = nh_.advertise<geometry_msgs::Twist>("/PSR/duty", 1);
		as_.start();
	}

	~PSRAction(void)
	{
	}

	void goalCB()
	{
		// accept the new goal
		goal_.duration = as_.acceptNewGoal()->duration;
		strcpy(goal_.steer_direction, as_.acceptNewGoal()->steer_direction);
		goal_.steer_radius = as_.acceptNewGoal()->steer_radius;
		goal_.linear_speed = as_.acceptNewGoal()->linear_speed;

		// reset helper variables
		data_count_ = 0;
		sum_ = 0;
		sum_sq_ = 0;
	}

	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}

	void analysisCB(const std_msgs::Float32::ConstPtr& msg)
	{
	// make sure that the action hasn't been canceled
	if (!as_.isActive())
		return;
    
	data_count_++;
	feedback_.sample = data_count_;
	feedback_.data = msg->data;
	//compute the std_dev and mean of the data 
	sum_ += msg->data;
	feedback_.mean = sum_ / data_count_;
	sum_sq_ += pow(msg->data, 2);
	feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
	as_.publishFeedback(feedback_);

	if(data_count_ > goal_) 
	{
	result_.mean = feedback_.mean;
	result_.std_dev = feedback_.std_dev;

	if(result_.mean < 5.0)
	{
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
	as_.setAborted(result_);
	}
	else 
	{
	ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
			}
		} 
	}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "averaging");

  AveragingAction averaging(ros::this_node::getName());
  ros::spin();

  return 0;
}
