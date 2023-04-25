#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <iiwa_msgs/SetSmartServoJointSpeedLimits.h>
#include <cmath>

#define PI 3.14159265
// getTimeToDestination() can also return negative values and the info from the cabinet take some milliseconds to update once the motion is started.
// That means that if you call getTimeToDestination() right after you set a target pose, you might get the wrong info (e.g. a negative number).
// This function tried to call getTimeToDestination() until something meaningful is obtained or until a maximum amount of time passed.
void sleepForMotion(iiwa_ros::service::TimeToDestinationService& iiwa, const double maxSleepTime) {
  double ttd = iiwa.getTimeToDestination();
  ros::Time start_wait = ros::Time::now();
  while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime)) {
    ros::Duration(0.5).sleep();
    ttd = iiwa.getTimeToDestination();
  }
  if (ttd > 0.0) {
    ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
    ros::Duration(ttd).sleep();
  }
}

int main (int argc, char **argv) {

	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");

  iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::state::JointPosition iiwa_joint_state;
  iiwa_ros::command::CartesianPose iiwa_pose_command;
  iiwa_ros::command::JointPosition iiwa_joint_command;
  iiwa_ros::service::TimeToDestinationService iiwa_time_destination;

  iiwa_pose_state.init("iiwa");
  iiwa_pose_command.init("iiwa");
  iiwa_joint_state.init("iiwa");
  iiwa_joint_command.init("iiwa");
  iiwa_time_destination.init("iiwa");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

  // Dynamic parameters. Last arg is the default value. Assignable from the launch file
  bool use_cartesian_command;
	nh.param("use_cartesian_command", use_cartesian_command, false);

  bool use_raster_pattern;
  nh.param("use_raster_pattern", use_raster_pattern, true);

  double square_size;
  nh.param("square_size", square_size, 0.25); //side of square length in m

  double init_x_pos;
  nh.param("init_x_pos", init_x_pos, -0.650);

  double init_y_pos;
  nh.param("init_y_pos", init_y_pos, -0.450);

  double init_z_pos;
  nh.param("init_z_pos", init_z_pos, 0.250);

  double rel_joint_vel;
  nh.param("rel_joint_vel", rel_joint_vel, 0.1);

  bool debug;
  nh.param("debug", debug, false);

  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 10.0); // 10 Hz = 0.1 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  //variable declaration
  iiwa_msgs::CartesianPose command_cartesian_position;
  iiwa_msgs::CartesianPose initial_cartesian_position;
  iiwa_msgs::JointPosition command_joint_position;
  bool new_pose = false, motion_done = false;

  int x_direction = 1;
  int y_direction = 1;
  bool invert = false;
  bool init_position_complete = false;
  int loop_count = 0;
  int num_poses;

  //Update number of poses based on use_raster_pattern bool
  if(use_raster_pattern){
    num_poses = 8;
  }else{
    num_poses = 4;
  }

  //raster pose joint position data
  double a1_raster_pose[8] = {1.15831, 1.78888, 1.13479, 1.61179, 0.50017, 1.48084, 0.10912, 1.84329};
  double a2_raster_pose[8] = {1.45563, 1.51025, 1.08052, 1.03553, 1.19027, 0.81667, 0.85254, 0.63564};
  double a3_raster_pose[8] = {0.86079, 0.29102, 0.28445, 0.28297, 1.18138, 0.48594, 1.18802, 0.27096};
  double a4_raster_pose[8] = {-0.46111, -0.20181, -0.81160, -0.95024, -1.48719, -1.55177, -2.04835, -1.81695};
  double a5_raster_pose[8] = {-0.52797, -0.78495, -0.02152, -0.52812, -0.80416, -0.63857, -0.75608, -0.64296};
  double a6_raster_pose[8] = {1.08759, 1.39308, 1.39142, 1.29087, 1.16503, 1.09915, 0.85577, 1.04670};
  double a7_raster_pose[8] = {-0.03264, 0.53630, 0.00010, -0.03285, 0.36581, 0.68564, 0.57598, 1.03574};

  //square pose joint position data
  double a1_square_pose[4] = {1.66945, 1.22804, 0.91597, 1.62945};
  double a2_square_pose[4] = {1.25381, 1.19460, 0.74224, 0.75460};
  double a3_square_pose[4] = {0.45038, 0.49532, 0.58048, 0.30611};
  double a4_square_pose[4] = {-0.51807,-0.58973, -1.52120, -1.35665};
  double a5_square_pose[4] = {-0.62232, -0.24815, -0.16138, -0.40937};
  double a6_square_pose[4] = {1.61520, 1.56363, 0.94964, 1.17917};
  double a7_square_pose[4] = {0.39339, 0.01055, 0.01063, 0.45241};

  //Get robot to initial position after the iiwa pose state is connected
  command_cartesian_position.poseStamped.pose.position.x = init_x_pos;
  command_cartesian_position.poseStamped.pose.position.y = init_y_pos;
  command_cartesian_position.poseStamped.pose.position.z = init_z_pos;

  //Initialize the robot to do slow moves (0.1 speed is the default)
  ros::service::waitForService("iiwa/configuration/setSmartServoLimits");
  ros::ServiceClient client = nh.serviceClient<iiwa_msgs::SetSmartServoJointSpeedLimits>("/iiwa/configuration/setSmartServoLimits");
  iiwa_msgs::SetSmartServoJointSpeedLimits updateSpeeds;
  updateSpeeds.request.joint_relative_velocity = rel_joint_vel;

  //Update terminal based on results
  if (client.call(updateSpeeds)) {
      if(!updateSpeeds.response.success)
          ROS_ERROR_STREAM("Config failed, Java error: " << updateSpeeds.response.error);
      else
          ROS_INFO_STREAM("Path Parameter Service successfully called.");
  }
  else {
      ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
  }

	while (ros::ok()) {
    if (iiwa_pose_state.isConnected()) {

			if (use_cartesian_command) {
        while (!iiwa_pose_state.isConnected()) {}
        command_cartesian_position = iiwa_pose_state.getPose();
        ROS_WARN_STREAM("Position: " << command_cartesian_position.poseStamped.pose.position);
        ROS_WARN_STREAM("Orientation: " << command_cartesian_position.poseStamped.pose.orientation);
        //Move the robot in a 2D box, keeping the same orientation and z position
        if(fmod(loop_count, 2) == 0){
           command_cartesian_position.poseStamped.pose.position.x += x_direction * square_size;
           x_direction *= -1;
        } else {
          command_cartesian_position.poseStamped.pose.position.y += y_direction * square_size;
          y_direction *= -1;
        }
        double cos_val = square_size*cos(2*PI*(loop_count / (double)(20/ros_rate)));
        double sin_val = square_size*sin(2*PI*(loop_count / (double)(20/ros_rate)));
        ROS_WARN_STREAM("Cos is: " << cos_val << ". Sin is: " << sin_val << ".");
        iiwa_pose_command.setPose(command_cartesian_position.poseStamped);

			} else {
        if(use_raster_pattern){ //Raster pattern joint position streaming
          while (!iiwa_pose_state.isConnected()){}
          command_joint_position = iiwa_joint_state.getPosition();
          command_joint_position.position.a1 = a1_raster_pose[loop_count];
          command_joint_position.position.a2 = a2_raster_pose[loop_count];
          command_joint_position.position.a3 = a3_raster_pose[loop_count];
          command_joint_position.position.a4 = a4_raster_pose[loop_count];
          command_joint_position.position.a5 = a5_raster_pose[loop_count];
          command_joint_position.position.a6 = a6_raster_pose[loop_count];
          command_joint_position.position.a7 = a7_raster_pose[loop_count];
          iiwa_joint_command.setPosition(command_joint_position);
        } else { //Square pattern joint position streaming
          while (!iiwa_pose_state.isConnected()){}
          command_joint_position = iiwa_joint_state.getPosition();
          command_joint_position.position.a1 = a1_square_pose[loop_count];
          command_joint_position.position.a2 = a2_square_pose[loop_count];
          command_joint_position.position.a3 = a3_square_pose[loop_count];
          command_joint_position.position.a4 = a4_square_pose[loop_count];
          command_joint_position.position.a5 = a5_square_pose[loop_count];
          command_joint_position.position.a6 = a6_square_pose[loop_count];
          command_joint_position.position.a7 = a7_square_pose[loop_count];
          iiwa_joint_command.setPosition(command_joint_position);
        }
			}
      if (debug)
        ROS_WARN_STREAM("Sending command to Robot for pose #: " << loop_count);
      //Sleep until the motion/movement is complete
			sleepForMotion(iiwa_time_destination, 2.0);
      //Logic for inverting
      //If doing a raster joint move, do the inversion indexing of the array to smooth the motion
      if (!use_cartesian_command && use_raster_pattern){
        if (!invert){
          if (loop_count < (num_poses-1))
            loop_count++;
          else
            invert = true;
        } else {
            if (loop_count > 0)
              loop_count--;
            else
              invert = false;
        }
      } else {
         loop_count++;
         if (!use_raster_pattern) //if using the square pattern, just go from positions 0-3 in order, repeatedly
          loop_count = loop_count % 4;
      }

			loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
		
    }
		else {
			ROS_WARN_STREAM("Robot is not connected...");
			ros::Duration(5.0).sleep(); // 5 seconds
		}
	}
};
