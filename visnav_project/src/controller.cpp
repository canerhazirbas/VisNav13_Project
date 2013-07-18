#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "DirectionCalculation.h"
#include <visnav_project/State.h>
#include <visnav_project/PidParameterConfig.h>
#include <visnav_project/LineDetectionMsg.h>
#include <visnav_project/AvoidObstaclesMsg.h>
#include "std_srvs/Empty.h"

class PidController {
private:
	float error_old;
	float ierror;
	ros::Time t_old;

public:
	float c_proportional;
	float c_integral;
	float c_derivative;

	PidController() {
		c_proportional = c_integral = c_derivative = 0;
		error_old = 0;
		ierror = 0;
		t_old = ros::Time::now();
		reset();
	}

	float getCommand(const ros::Time& t, float error) {
		// derivative
		float derror = 0;
		ros::Duration diff = t - t_old;

		if (diff.toSec()) {
			derror = (error - error_old) / diff.toSec();
			error_old = error;
			return getCommand(t, error, derror);
		}
		return 0;
	}

	float getCommand(const ros::Time& t, float error, float derror) {
		ros::Duration diff = t - t_old;
		ierror = ierror + diff.toSec() * error;

		float u; // Control input
		u = c_proportional * error + c_derivative * derror
				+ c_integral * ierror;

		t_old = t;
		return u;
	}

	// resets the internal state
	void reset() {
	}

};

class ArdroneController {
private:
	ros::NodeHandle& nh;
	ros::Subscriber sub_pose, sub_enabled;
	ros::Publisher pub_vel;
	ros::Publisher pub_cmd_marker;
	ros::Publisher pub_land, pub_takeoff;
	ros::Subscriber sub_lineDetection;
	ros::Subscriber sub_avoidObstacle;
	ros::ServiceClient srv_cl_cam;
	std_srvs::Empty srv_empty;
	ros::Time first_time;

	dynamic_reconfigure::Server<visnav_project::PidParameterConfig> reconfigure_server;
	visnav_project::PidParameterConfig current_cfg;

	geometry_msgs::Twist twist;
	visnav_project::State state;
	visnav_project::LineDetectionMsg ld;
	visnav_project::AvoidObstaclesMsg ao;

	PidController pid_x, pid_y, pid_yaw;

	bool enabled, obstacle, get_direction, init_toggle, toggle_backdown,obstacle_detected;
	float goal_x, goal_y, goal_yaw;
	int iterator;

public:
	ArdroneController(ros::NodeHandle& nh) :
			nh(nh), reconfigure_server(), enabled(false) {
		pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
		pub_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);

		pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		pub_cmd_marker = nh.advertise<visualization_msgs::Marker>(
				"visualization_marker", 10);

		reconfigure_server.setCallback(
				boost::bind(&ArdroneController::onConfig, this, _1, _2));

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;
//		setGoalPose(0, 0, 0);

		sub_enabled = nh.subscribe<std_msgs::Bool>("/ardrone/enable_controller",
				1,
				boost::bind(&ArdroneController::onEnableController, this, _1));
		sub_pose = nh.subscribe<visnav_project::State>("/ardrone/filtered_pose",
				1, boost::bind(&ArdroneController::onFilteredPose, this, _1));
		sub_lineDetection = nh.subscribe("/line_detection", 100,
				&ArdroneController::lineDetectionCB, this);
		sub_avoidObstacle = nh.subscribe("/avoid_obstacle", 100,
				&ArdroneController::avoidObstacleCB, this);
		srv_cl_cam = nh.serviceClient<std_srvs::Empty>("/ardrone/togglecam");

		obstacle = FALSE;
		get_direction = FALSE;
		init_toggle = TRUE;
		iterator = 0;
		first_time.sec = 0;
		toggle_backdown = FALSE;
		obstacle_detected = FALSE;
	}

	void setPidParameters(visnav_project::PidParameterConfig &config) {
		pid_x.c_proportional = pid_y.c_proportional = config.c_prop_trans;
		pid_x.c_integral = pid_y.c_integral = config.c_int_trans;
		pid_x.c_derivative = pid_y.c_derivative = config.c_deriv_trans;

		pid_yaw.c_proportional = config.c_prop_yaw;
		pid_yaw.c_integral = config.c_int_yaw;
		pid_yaw.c_derivative = config.c_deriv_yaw;
	}

//	void setGoalPose(float x, float y, float yaw) {
//		goal_x = x;
//		goal_y = y;
//		goal_yaw = yaw;
//	}

	void setEnabled(bool v) {
		enabled = v;

		if (!enabled) {
			pid_y.reset();
			pid_yaw.reset();
		}
	}

	void onTimerTick(const ros::TimerEvent& e) {

		//take off
//		pub_takeoff.publish(std_msgs::Empty());
		//initial toggle camera
		if (init_toggle) {
			ROS_INFO("Initial Toggle the Camera!");
			if (!srv_cl_cam.call(srv_empty)) {
				ROS_INFO("Failed to toggle Camera");
			}
			init_toggle = FALSE;
		}

		if (!obstacle) {
			lineDetectionController(e.current_real);
		} else	{
			avoidObstacleController(e.current_real);
		}

		if (enabled)
			pub_vel.publish(twist);

		sendCmdMarker(e.current_real);
	}

	void lineDetectionController(const ros::Time& t) {
		float u_x = 0.03;

		float u_y = pid_y.getCommand(t, ld.error_pitch);

		twist.linear.x = u_x;
		twist.linear.y = u_y;

		float u_yaw = pid_yaw.getCommand(t, ld.error_yaw);

		// normalize angular control command
		twist.angular.z = atan2(sin(u_yaw), cos(u_yaw));
	}

	void avoidObstacleController(const ros::Time& t) {
		float u_x = 0.05, u_z = 0.4, u_y = 0;

		if (first_time.sec == 0) {
			first_time = t;
		} else if (t.sec - first_time.sec < 2) {
			ROS_INFO("Go up!");
			twist.linear.z = u_z;
		} else if (t.sec - first_time.sec < 4) {
			ROS_INFO("Go down!");
			twist.linear.z = -u_z;
		} else {
			ROS_INFO("Stop!");
			twist.linear.z = 0;
			obstacle = 0;
		}

		twist.linear.x = u_x;
		twist.linear.y = u_y;

	}

	void lineDetectionCB(
			const visnav_project::LineDetectionMsg::ConstPtr& ld_msg) {
		float threshold_error_yaw = 0.1;
		int iteratorMax = 15;

		ld = *ld_msg;
		//if the yaw error is continuously 10 times smaller than the threshold, we assume that the drone is now following the line.
		if (ld.error_yaw) {
			if (iterator < iteratorMax) {
				if (ld.error_yaw < threshold_error_yaw) {
					iterator++;
				} else {
					iterator = 0;
				}
			} else if (!obstacle_detected){
				ROS_INFO("Toggle the Camera!");
				if (!srv_cl_cam.call(srv_empty)) {
					ROS_INFO("Failed to toggle Camera");
				}
				iterator = 0;
			}
		}

	}

	void avoidObstacleCB(
			const visnav_project::AvoidObstaclesMsg::ConstPtr& ao_msg) {
		ao = *ao_msg;
		float threshold = 0.6;
		if (ao.distance != -1) {
			ROS_INFO("distance: %f", ao.distance);
			if (ao.distance < threshold && toggle_backdown == FALSE) {
				ROS_INFO("Too near to the obstacle! Toggle the Camera!");
				if (!srv_cl_cam.call(srv_empty)) {
					ROS_INFO("Failed to toggle Camera");
				}
				obstacle = 1;
				toggle_backdown = TRUE;
				obstacle_detected = TRUE;
			}
		}
	}

	void onConfig(visnav_project::PidParameterConfig& cfg, uint32_t level) {
		current_cfg = cfg;
		setEnabled(cfg.enable);
		setPidParameters(cfg);
	}

	void onEnableController(const std_msgs::Bool::ConstPtr& msg) {
		setEnabled(msg->data);
		current_cfg.enable = msg->data;
		reconfigure_server.updateConfig(current_cfg);
	}

	void onFilteredPose(const visnav_project::State::ConstPtr& pose_msg) {
		state = *pose_msg;
	}

	// control in xy and yaw
	void calculateContolCommand(const ros::Time& t) {
		float e_x, e_y, e_yaw;
		e_x = goal_x - state.x;
		e_y = goal_y - state.y;
		e_yaw = goal_yaw - state.yaw;

		// use this yaw to rotate commands from global to local frame
		float yaw = -(state.yaw + M_PI_2);

//		float u_x = pid_x.getCommand(t, e_x);
//		float u_y = pid_y.getCommand(t, e_y);

		float u_x = pid_x.getCommand(t, e_x, -state.vx);
		float u_y = pid_y.getCommand(t, e_y, -state.vy);

		twist.linear.x = cos(yaw) * u_x - sin(yaw) * u_y;
		twist.linear.y = sin(yaw) * u_x + cos(yaw) * u_y;

		float u_yaw = pid_yaw.getCommand(t, e_yaw);

		// normalize angular control command
		twist.angular.z = atan2(sin(u_yaw), cos(u_yaw));

	}

	void sendCmdMarker(const ros::Time& t) {
		visualization_msgs::Marker marker;

		marker.header.frame_id = "/world";

		marker.action = visualization_msgs::Marker::ADD;
		marker.ns = "ardrone_controller";
		marker.header.stamp = t;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.05; // width of line in m
		marker.scale.y = 0.1; // width of line in m
		geometry_msgs::Point p, q;
		p.x = state.x;
		p.y = state.y;
		p.z = state.z;

		float length = 2.0;

		marker.color.a = 1.0;
		marker.color.g = 1.0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.points.clear();
		marker.id = 0;

		float yaw = -(state.yaw + M_PI_2);
		q.x =
				p.x
						+ length
								* (cos(yaw) * twist.linear.x
										+ sin(yaw) * twist.linear.y);
		q.y = p.y
				+ length
						* (-sin(yaw) * twist.linear.x
								+ cos(yaw) * twist.linear.y);
		q.z = p.z + length * twist.linear.z;

		marker.points.push_back(p);
		marker.points.push_back(q);
		pub_cmd_marker.publish(marker);

		// show rotation: attach arrow to end of pose-marker-arrow and rotate by 90deg
		// length is proportional to the rotation speed
		marker.points.clear();
		p.x = state.x + cos(yaw) * 0.5; //0.5 = length of pose-marker-arrow
		p.y = state.y - sin(yaw) * 0.5;
		q.x = p.x + 1 * (cos(yaw - M_PI / 2) * twist.angular.z); // 10: arbitrary constant (angular.z is in rad, marker in meter)
		q.y = p.y + 1 * (-sin(yaw - M_PI / 2) * twist.angular.z);
		q.z = p.z + length * twist.linear.z;
		marker.points.push_back(p);
		marker.points.push_back(q);
		marker.id = 1;
		marker.color.g = 0;
		marker.color.b = 1.0;

		pub_cmd_marker.publish(marker);
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "ardrone_controller");
	ros::NodeHandle nh;

	ArdroneController controller(nh);

	ros::Timer timer = nh.createTimer(ros::Duration(0.02),
			boost::bind(&ArdroneController::onTimerTick, &controller, _1));

	ros::spin();

	return 0;
}

