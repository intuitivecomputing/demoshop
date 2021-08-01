/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

using RosSharp.RosBridgeClient.Messages.Standard;

namespace RosSharp.RosBridgeClient.Messages.Trajectory
{
    public class JointTrajectory : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "trajectory_msgs/JointTrajectory";

        public Header header;
        public string[] joint_names;
        public JointTrajectoryPoint[] points;

        public JointTrajectory()
        {
            this.header = new Header();
            this.joint_names = new string[0];
            this.points = new JointTrajectoryPoint[0];
        }

        public JointTrajectory(Header header, string[] joint_names, JointTrajectoryPoint[] points)
        {
            this.header = header;
            this.joint_names = joint_names;
            this.points = points;
        }
    }
}
