// ExecutePathActionFeedback.cs

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
	public class ExecutePathActionFeedback : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/ExecutePathActionFeedback";

		public Header header;
		public GoalStatus status;
		public ExecutePathFeedback feedback;

		public ExecutePathActionFeedback()
		{
			header = new Header();
			status = new GoalStatus();
			feedback = new ExecutePathFeedback();
		}
	}
}

