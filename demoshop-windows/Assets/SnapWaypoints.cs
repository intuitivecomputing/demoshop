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
	public class SnapWaypoints : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/SnapWaypoints";

		public Waypoint[] waypoints;

		public SnapWaypoints(int num_wpts)
		{
			waypoints = new Waypoint[num_wpts];
		}
	}
}