// ExecutePathFeedback.cs

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
	public class ExecutePathFeedback : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/ExecutePathFeedback";

		public string status;

		public ExecutePathFeedback()
		{
			status = System.String.Empty;
		}
	}
}

