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
    public class JointTrajectoryPoint : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "trajectory_msgs/JointTrajectoryPoint";

        //  Each trajectory point specifies either positions[, velocities[, accelerations]]
        //  or positions[, effort] for the trajectory to be executed.
        //  All specified values are in the same order as the joint names in JointTrajectory.msg
        public double[] positions;
        public double[] velocities;
        public double[] accelerations;
        public double[] effort;
        public Duration time_from_start;

        public JointTrajectoryPoint()
        {
            this.positions = new double[0];
            this.velocities = new double[0];
            this.accelerations = new double[0];
            this.effort = new double[0];
            this.time_from_start = new Duration();
        }

        public JointTrajectoryPoint(double[] positions, double[] velocities, double[] accelerations, double[] effort, Duration time_from_start)
        {
            this.positions = positions;
            this.velocities = velocities;
            this.accelerations = accelerations;
            this.effort = effort;
            this.time_from_start = time_from_start;
        }
    }
}
