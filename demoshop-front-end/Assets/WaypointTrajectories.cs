/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/ 

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;
using RosSharp.RosBridgeClient.Messages.MoveIt;

namespace RosSharp.RosBridgeClient.Messages
{
	public class WaypointTrajectories : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/WaypointTrajectories";

		public RobotTrajectory[] trajectories;
	}
}

