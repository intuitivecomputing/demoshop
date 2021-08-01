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
	public class SnapWaypoint : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/SnapWaypoint";

		public string[] names;
		public double[] joints;
		public int wpt_type;

		public SnapWaypoint()
		{
			names = new string[6];
			joints = new double[6];
			wpt_type = new int();
		}
	}
}