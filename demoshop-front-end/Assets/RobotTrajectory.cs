/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/ 

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;
using RosSharp.RosBridgeClient.Messages.Trajectory;

namespace RosSharp.RosBridgeClient.Messages.MoveIt
{
	public class RobotTrajectory : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "moveit_msgs/RobotTrajectory";

		public JointTrajectory joint_trajectory;
		public MultiDOFJointTrajectory multi_dof_joint_trajectory;

		public RobotTrajectory()
		{
			this.joint_trajectory = new JointTrajectory();
			this. multi_dof_joint_trajectory = new MultiDOFJointTrajectory();
		}

		public RobotTrajectory(JointTrajectory joint_trajectory, MultiDOFJointTrajectory multi_dof_joint_trajectory)
		{
			this.joint_trajectory = joint_trajectory;
			this. multi_dof_joint_trajectory = multi_dof_joint_trajectory;
		}
	}
}

