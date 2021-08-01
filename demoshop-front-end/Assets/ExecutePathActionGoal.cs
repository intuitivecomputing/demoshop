// ExecutePathActionGoal.cs

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
	public class ExecutePathActionGoal : Message
	{
		[JsonIgnore]
		public const string RosMessageName = "demoshop_ubuntu/ExecutePathActionGoal";

		public Header header;
		public GoalID goal_id;
		public ExecutePathGoal goal;

		public ExecutePathActionGoal()
		{
			header = new Header();
			goal_id = new GoalID();
			goal = new ExecutePathGoal();
		}
	}
}

