// ExecutePathGoal.cs

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
	public class ExecutePathGoal : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/ExecutePathGoal";

		public byte request;

		public ExecutePathGoal()
		{
			request = new byte();
		}
	}
}

