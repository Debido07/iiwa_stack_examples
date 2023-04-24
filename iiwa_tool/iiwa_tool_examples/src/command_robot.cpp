#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <cmath>

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

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  bool use_cartesian_command;
	nh.param("use_cartesian_command", use_cartesian_command, false);

	// Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
	nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

  iiwa_msgs::CartesianPose command_cartesian_position;
  iiwa_msgs::JointPosition command_joint_position;
  bool new_pose = false, motion_done = false;

	int direction = 1;
  bool invert = false;
  int loop_count = 0;
  int num_poses = 8;
  double a1_pose[8] = {1.15831, 1.78888, 1.13479, 1.61179, 0.50017, 1.48084, 0.10912, 1.84329};
  double a2_pose[8] = {1.45563, 1.51025, 1.08052, 1.03553, 1.19027, 0.81667, 0.85254, 0.63564};
  double a3_pose[8] = {0.86079, 0.29102, 0.28445, 0.28297, 1.18138, 0.48594, 1.18802, 0.27096};
  double a4_pose[8] = {-0.46111, -0.20181, -0.81160, -0.95024, -1.48719, -1.55177, -2.04835, -1.81695};
  double a5_pose[8] = {-0.52797, -0.78495, -0.02152, -0.52812, -0.80416, -0.63857, -0.75608, -0.64296};
  double a6_pose[8] = {1.08759, 1.39308, 1.39142, 1.29087, 1.16503, 1.09915, 0.85577, 1.04670};
  double a7_pose[8] = {-0.03264, 0.53630, 0.00010, -0.03285, 0.36581, 0.68564, 0.57598, 1.03574};

	while (ros::ok()) {
    if (iiwa_pose_state.isConnected()) {

			if (use_cartesian_command) {
        while (!iiwa_pose_state.isConnected()) {}
        command_cartesian_position = iiwa_pose_state.getPose();
				// Here we set the new commanded cartesian position, we just move the tool TCP 10 centemeters down and back up, every 10 seconds.
				command_cartesian_position.poseStamped.pose.position.z -= direction * 0.10;
				iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
			} else {
        while (!iiwa_joint_state.isConnected()) {}
        command_joint_position = iiwa_joint_state.getPosition();
        command_joint_position.position.a1 = a1_pose[loop_count];
        command_joint_position.position.a2 = a2_pose[loop_count];
        command_joint_position.position.a3 = a3_pose[loop_count];
        command_joint_position.position.a4 = a4_pose[loop_count];
        command_joint_position.position.a5 = a5_pose[loop_count];
        command_joint_position.position.a6 = a6_pose[loop_count];
        command_joint_position.position.a7 = a7_pose[loop_count];        
				iiwa_joint_command.setPosition(command_joint_position);
        ROS_WARN_STREAM("Sending command to Robot for pose #" << loop_count);
			}

			sleepForMotion(iiwa_time_destination, 2.0);
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
			loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
		
    }
		else {
			ROS_WARN_STREAM("Robot is not connected...");
			ros::Duration(5.0).sleep(); // 5 seconds
		}
	}
};
