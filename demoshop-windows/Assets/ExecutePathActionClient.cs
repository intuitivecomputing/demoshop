// ExecutePathActionClient.cs

using RosSharp.RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClient
{
	public class ExecutePathActionClient : ActionClient<ExecutePathActionGoal, ExecutePathActionFeedback, ExecutePathActionResult>
	{
		private byte Request;

		public override ExecutePathActionGoal GetGoal()
		{
			return new ExecutePathActionGoal() { goal = new ExecutePathGoal { request = Request } };
		}

		public void SetRequest(byte request)
		{
			Request = request;
		}

		public string PrintFeedback()
		{
			if (ActionFeedback == null)
			{
				return "Waiting for command";
			}

			return ActionFeedback.feedback.status;
		}

		public string PrintResult()
		{
			if (ActionResult == null)
			{
				return "=";
			}

			return ActionResult.result.conclusion;
		}

		public string PrintStatus()
		{
			return ActionState.ToString();
		}
	}
}
