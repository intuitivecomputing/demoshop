/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/ 

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Messages
{
	public class Waypoint : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/Waypoint";

		public float[] joint_values;
		public float grip_action;

		public Waypoint()
		{
			joint_values = new float[7];
			grip_action = new float();
		}
	}
}

