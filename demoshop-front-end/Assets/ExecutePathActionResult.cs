// ExecutePathActionResult.cs

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
	public class ExecutePathActionResult : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/ExecutePathActionResult";

		public Header header;
		public GoalStatus status;
		public ExecutePathResult result;

		public ExecutePathActionResult()
		{
			header = new Header();
			status = new GoalStatus();
			result = new ExecutePathResult();
		}
	}
}

