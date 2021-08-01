// robot_interface_sim.cpp
// Author: Gopika Ajaykumar

#include <ros/ros.h>
#include <tf/tf.h>


// Move-It Actions and Messages
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_state/robot_state.h>

// Other Actions
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <demoshop_ubuntu/ExecutePathAction.h>
#include <control_msgs/GripperCommandAction.h>

// Transform Messages
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>

// Other Messages
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <demoshop_ubuntu/SnapTrajectories.h>
#include <demoshop_ubuntu/WaypointTrajectories.h>
#include <demoshop_ubuntu/SnapWaypoint.h>
#include <demoshop_ubuntu/SnapWaypoints.h>

// Constants
const float PI = 3.14159265359;

// Trajectory publisher
ros::Publisher trajs_pub;                           // Publishes list of planned trajectories for visualization on Unity GUI
ros::Publisher tracked_object_pose_pub;             // Publishes poses of objects being tracked in environment
ros::Publisher snap_wpt_pub;
demoshop_ubuntu::WaypointTrajectories trajs_msg;    // List of planned trajectories
demoshop_ubuntu::SnapWaypoint snap_wpt_msg;

std::vector<std::vector<double>> waypoints;         // Stores the waypoints from the demonstration provided by the user, where each waypoint is a list of its respective joint positions
std::vector<float> gripper_positions;               // Stores the gripper positions associated with each waypoint; corresponds to waypoints vector

// MoveIt!
boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group;   // Shared pointer to MoveGroupInterface 
static const std::string PLANNING_GROUP = "manipulator";
moveit_msgs::RobotTrajectory trajectory_msg;
const float MAX_TIME = 120.0;   // Maximum planning time
const int MAX_ATTEMPTS = 10;    // Maximum attempts allowed for replanning
const float VEL_SCALE = 0.1;    
const float ACC_SCALE = 0.1;
std::string planner_plugin_name;
std::string ns;

// Action server
typedef actionlib::SimpleActionServer<demoshop_ubuntu::ExecutePathAction> Server;
enum  Requests {CLEAR=0, INSPECT=1, PREVIEW=2, OPEN=3, CLOSE=4, EXECUTE=5, PICK=6};  // Six possible types of requests from action client
int indexToInspect; // User-specified index of waypoint for inspection
int indexToGrip;    // User-specified index of waypoint for changing grip action
std::string objectToPick;

std::map<std::string,moveit_msgs::CollisionObject> tracked_object_dict; // Map holding information about objects being tracked 
std::map<std::string,geometry_msgs::Pose> tracked_object_xform_dict;    // Map holding information about the transforms of objects being tracked 

geometry_msgs::TransformStamped obj_transform;

class Listener  // Transform listener
{
    public:
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener* tfListener;
        Listener()
        {
            tfListener = new tf2_ros::TransformListener(tfBuffer);
        }
        ~Listener()
        {
            delete tfListener;
        }
};

/*** Helper Functions ***/
// Converts geometry_msgs/Transform message to geometry_msgs/Pose message
geometry_msgs::Pose convertTransformToPose(geometry_msgs::Transform transform)
{
    //ROS_INFO("convertTransformToPose");
    geometry_msgs::Pose pose;
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation.x = transform.rotation.x;
    pose.orientation.y = transform.rotation.y;
    pose.orientation.z = transform.rotation.z;
    pose.orientation.w = transform.rotation.w;
    return pose;
}

// Converts tf2/Transform message to geometry_msgs/Pose message
geometry_msgs::Pose tftoMsg(tf2::Transform transform)
{
    //ROS_INFO("tfToMsg");;
    geometry_msgs::Pose pose;
    pose.position.x = transform.getOrigin().getX();
    pose.position.y = transform.getOrigin().getY();
    pose.position.z = transform.getOrigin().getZ();
    pose.orientation.x = transform.getRotation().getX();
    pose.orientation.y = transform.getRotation().getY();
    pose.orientation.z = transform.getRotation().getZ();
    pose.orientation.w = transform.getRotation().getW();
    return pose;
}

// Returns the absolute value of a double
double my_abs (double a)
{
    if (a>0) return a;
    else return -a;
}

// Returns a geometry_msgs/Pose message with the given position and orientation
geometry_msgs::Pose addPose(double x, double y, double z, double roll , double pitch , double yaw) 
{
    //ROS_INFO("addPose");
    tf2::Quaternion q;
    q.setRPY(roll,pitch, yaw);
    tf2::Transform transform(  q, tf2::Vector3(x,y,z));
    return tftoMsg(transform);
}


// Adds collision objects for static environments in the scene. NOTE: Modify to include the fixed objects specific to your environment.
void addCollisionObjects()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Static environment object dimensions and positions/orientations are hard-coded for study environment, change as needed
    moveit_msgs::CollisionObject base;
    base.id = "base";
    base.header.frame_id = "world";
    base.primitives.resize(1);
    base.primitives[0].type = base.primitives[0].BOX;
    base.primitives[0].dimensions.resize(3);
    base.primitives[0].dimensions[0] = 0.61;
    base.primitives[0].dimensions[1] = 0.7;
    base.primitives[0].dimensions[2] = 0.85;
    base.primitive_poses.resize(1);
    base.primitive_poses[0] = addPose(0.0, 0.255, -0.425, 0.0, 0.0, 0.0);
    base.operation = base.ADD;
    planning_scene_interface.applyCollisionObject(base);


    moveit_msgs::CollisionObject tv;
    tv.id = "TV";
    tv.header.frame_id = "world";
    tv.primitives.resize(1);
    tv.primitives[0].type = tv.primitives[0].BOX;
    tv.primitives[0].dimensions.resize(3);
    tv.primitives[0].dimensions[0] = 0.12;
    tv.primitives[0].dimensions[1] = 1.129;
    tv.primitives[0].dimensions[2] = .67;
    tv.primitive_poses.resize(1);
    tv.primitive_poses[0] = addPose(0.29, 1.06, 0.445, 0.0, 0.0, 0.0);
    tv.operation = tv.ADD;
    planning_scene_interface.applyCollisionObject(tv);


    moveit_msgs::CollisionObject wall;
    wall.id = "wall";
    wall.header.frame_id = "wall";
    wall.primitives.resize(1);
    wall.primitives[0].type = wall.primitives[0].BOX;
    tv.primitives[0].dimensions.resize(3);
    tv.primitives[0].dimensions[0] = 0.001;
    tv.primitives[0].dimensions[1] = 5;
    tv.primitives[0].dimensions[2] = 2.5;
    tv.primitive_poses.resize(1);
    tv.primitive_poses[0] = addPose(0.29, 1.06, 0.445, 0.0, 0.0, 0.0);
    tv.operation = tv.ADD;
    planning_scene_interface.applyCollisionObject(tv);

    moveit_msgs::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world";
    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions.resize(3);
    table.primitives[0].dimensions[0] = 0.76;
    table.primitives[0].dimensions[1] = 1.68;
    table.primitives[0].dimensions[2] = 0.71;
    table.primitive_poses.resize(1);
    table.primitive_poses[0] = addPose(-0.74, -.18, -0.49, 0.0, 0.0, 0.0);
    table.operation = table.ADD;
    planning_scene_interface.applyCollisionObject(table);

    moveit_msgs::CollisionObject computer;
    computer.id = "computer";
    computer.header.frame_id = "world";
    computer.primitives.resize(1);
    computer.primitives[0].type = computer.primitives[0].BOX;
    computer.primitives[0].dimensions.resize(3);
    computer.primitives[0].dimensions[0] = 0.54;
    computer.primitives[0].dimensions[1] = 0.5;
    computer.primitives[0].dimensions[2] = 0.5;
    computer.primitive_poses.resize(1);
    computer.primitive_poses[0] = addPose(-.95, -.90, .11, 0.0, 0.0, 0.0);
    computer.operation = computer.ADD;
    planning_scene_interface.applyCollisionObject(computer);
}

geometry_msgs::TransformStamped correctBoxFrame(const geometry_msgs::TransformStamped& box_msg, const std::string child_frame_id)
{
    tf::StampedTransform box_tf;
    tf::transformStampedMsgToTF(box_msg, box_tf);
    double x,y,z;
    box_tf.getBasis().getEulerYPR(z,y,x);
    double pi = 3.14159265;

    tf::Transform offset_tf;
    offset_tf.setIdentity();
    bool error = true;

    tf::Vector3 base_x(1,0,0);
    tf::Vector3 base_y(0,1,0);
    tf::Vector3 base_z(0,0,1);
    tf::Vector3 current_base_x;
    tf::Vector3 current_base_y;
    tf::Vector3 current_base_z;

    current_base_x = box_tf * base_x;
    current_base_y = box_tf * base_y;
    current_base_z = box_tf * base_z;

    if (my_abs(1.0 - base_z.dot(current_base_z)) <1e-1 ) {
        offset_tf.setIdentity();
        error = false;
    }
    if (my_abs(-1.0 - base_z.dot(current_base_z)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(pi, 0, 0));
        error = false;
    }
    if (my_abs(1.0 - base_z.dot(current_base_x)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(0, pi/2, 0));
        error = false;
    }
    if (my_abs(-1.0 - base_z.dot(current_base_x)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(0, -pi/2, 0));
        error = false;
    }
     if (my_abs(1.0 - base_z.dot(current_base_y)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(-pi/2, 0, 0));
        error = false;
    }
    if (my_abs(-1.0 - base_z.dot(current_base_y)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(pi/2, 0, 0));
        error = false;
    }

    if (error) {
        ROS_WARN("Invalid Rotation Angle: R: %.4f  P: %.4f  Y: %.4f", x, y, z);
    }

    geometry_msgs::TransformStamped corrected_box_msg;
    tf::StampedTransform correct_box_tf;
    correct_box_tf.setData(box_tf * offset_tf);
    correct_box_tf.stamp_ = ros::Time::now();
    tf::transformStampedTFToMsg (correct_box_tf, corrected_box_msg);
    corrected_box_msg.header.frame_id = "world";
    corrected_box_msg.child_frame_id = child_frame_id + "_corrected";

    return corrected_box_msg;
}

/*** Callbacks ***/
// Callback that adds waypoint to vector every time Demoshop publishes one, sets gripper position to default
void addWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
    waypoints.push_back(move_group->getCurrentJointValues());
    gripper_positions.push_back(0.0);
}

// Callback that deletes waypoint at specified index and its associated gripper action
void deleteCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (waypoints.size())
    {
        waypoints.erase(waypoints.begin() + msg->data);
        gripper_positions.erase(gripper_positions.begin() + msg->data);
    }
}

// Callback that adds the user-specified waypoint at the specified time point, together with a default gripper action
void addCallback(const std_msgs::Int32::ConstPtr& msg)
{
    waypoints.insert(waypoints.begin() + msg->data, move_group->getCurrentJointValues());
    gripper_positions.insert(gripper_positions.begin() + msg->data, 0.0);
}

// Callback that retrieves the index of the waypoint that the user has requested to inspect
void inspCallback(const std_msgs::Int32::ConstPtr& msg)
{
    indexToInspect = msg->data;
}

// Callback that retrieves the index of the waypoint that the user has requested to change the gripper action for
void gripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    indexToGrip = msg->data;
}

// Callback that retrieves the amount by which the gripper should close for a pre-specified waypoint
void closeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    gripper_positions[indexToGrip] = msg->data;
}

// Callback that retrieves and publishes transforms of objects in the world
void transformCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    static Listener tl;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object;
    geometry_msgs::TransformStamped corrected_box_msg;

    // NOTE: We used ar_master_22 as a reference tag placed on the workspace. Please modify according to your setup.
    if ((msg->transforms[0].child_frame_id.find("ar_master") != std::string::npos) && (msg->transforms[0].child_frame_id.find("ar_master_22") == std::string::npos))   // All tracked objects should have frame ar_master_x based on ar_track_alvar bundle tracking naming convention
    //if (msg->transforms[0].child_frame_id.find("filtered") != std::string::npos)  // Uncomment if using filter
    {
        try
        {
            if (tl.tfBuffer.canTransform(msg->transforms[0].child_frame_id, "world", ros::Time(0)))
            {
                obj_transform = tl.tfBuffer.lookupTransform("world", msg->transforms[0].child_frame_id, ros::Time(0));
                tracked_object_pose_pub.publish(obj_transform); // Send transform to Unity
                corrected_box_msg = correctBoxFrame(obj_transform, msg->transforms[0].child_frame_id);
            }
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }


        // NOTE: Modify the code below to reflect the objects in your environment.
        std::map<std::string, moveit_msgs::CollisionObject>::iterator it = tracked_object_dict.find(msg->transforms[0].child_frame_id);
        if(it == tracked_object_dict.end()) // The collision object needs to be added to the planning scene
        {
            collision_object.id = msg->transforms[0].child_frame_id;
            collision_object.header.frame_id = "cam";   // Transform is WRT camera frame
            collision_object.primitives.resize(1);
            collision_object.primitives[0].type = collision_object.primitives[0].BOX;
            collision_object.primitives[0].dimensions.resize(3);    // Hard-coded dimensions for study task objects, change as required
            collision_object.primitives[0].dimensions[0] = 0.2;
            collision_object.primitives[0].dimensions[1] = 0.244;
            collision_object.primitives[0].dimensions[2] = 0.236;
            collision_object.primitive_poses.resize(1);
            collision_object.primitive_poses[0] = convertTransformToPose(msg->transforms[0].transform);
            collision_object.operation = collision_object.ADD;
            tracked_object_dict.insert({collision_object.id, collision_object});
            geometry_msgs::Pose obj_xform = convertTransformToPose(corrected_box_msg.transform);
            tracked_object_xform_dict.insert({collision_object.id, obj_xform});
        }
        else    // The collision object is already in our planning scene
        {   
            tracked_object_dict[msg->transforms[0].child_frame_id].primitive_poses.resize(1);
            tracked_object_dict[msg->transforms[0].child_frame_id].primitive_poses[0] = convertTransformToPose(msg->transforms[0].transform);
            // Uncomment below if task object should be a collision object
            tracked_object_dict[msg->transforms[0].child_frame_id].operation = collision_object.MOVE;
            geometry_msgs::Pose obj_xform = convertTransformToPose(corrected_box_msg.transform);
            tracked_object_xform_dict[msg->transforms[0].child_frame_id] = obj_xform;
        }
    }
}

void pickCallback(const std_msgs::String::ConstPtr& msg)
{
    objectToPick = msg->data;
}

void snapCallback(const demoshop_ubuntu::SnapWaypoints::ConstPtr& msg)
{
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
        waypoints.push_back(msg->waypoints[i].joint_values);
        gripper_positions.push_back(msg->waypoints[i].grip_action);
    }
}

/*** Action server ***/
void execute(const demoshop_ubuntu::ExecutePathGoalConstPtr& goal, Server* as)
{
    demoshop_ubuntu::ExecutePathFeedback feedback;    // Feedback to send back to action client
    demoshop_ubuntu::ExecutePathResult result;        // Result of demo execution
    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> traj_ac("execute_trajectory", true); // NOTE: Replace "execute_trajectory" with the name of the action server corresponding to your robot
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac("gripper/gripper_cmd", true); // NOTE: Replace "gripper/gripper_cmd" with the name of the action server corresponding to your robot's gripper
    moveit::planning_interface::MoveGroupInterface gripper_move_group("gripper");   // MoveGroup Interface for gripper

    if (!traj_ac.waitForServer(ros::Duration(2.0)))
    {
        as->setAborted();
        ROS_INFO("Could not connect to action server for executing robot trajectory");
        return;
    }

    if (!gripper_ac.waitForServer(ros::Duration(2.0)))
    {
        as->setAborted();
        ROS_INFO("Could not connect to gripper action server for executing gripper trajectory");
        return;
    }

    if (goal->request == INSPECT)   // User wants the robot to move to a specific waypoint
    {
        const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
        std::vector<moveit_msgs::RobotTrajectory>::iterator it = trajs_msg.trajectories.begin();
        moveit::core::RobotStatePtr current_gripper_state;
        std::vector<double> gripper_joint_group_positions;
        bool success;
        move_group->setPlanningTime(MAX_TIME);
        move_group->setMaxVelocityScalingFactor(VEL_SCALE);
        move_group->setMaxAccelerationScalingFactor(ACC_SCALE);
        move_group->setGoalJointTolerance(0.001);
        move_group->setJointValueTarget(waypoints[indexToInspect]);
        move_group->setStartStateToCurrentState();
        feedback.status = "Moving to specified inspection point";
        as->publishFeedback(feedback);
        success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
            feedback.status = "Unable to move to specified inspection point";
            as->publishFeedback(feedback);
            as->setAborted();
            ROS_INFO("Aborting goal since could not move to inspection point");
            return;
        }
        move_group->clearPoseTargets();
        if (gripper_positions[indexToInspect] != 0.0)    // 0.0 means the gripper action for this waypoint has not been set yet
        {
            current_gripper_state = gripper_move_group.getCurrentState();
            current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

            if (my_abs(gripper_joint_group_positions[0] - gripper_positions[indexToInspect]) > 5e-2) // If gripper needs to be moved
            {
                ROS_INFO("Gripper goal: %f", gripper_positions[indexToInspect]);
                control_msgs::GripperCommandGoal goal;
                goal.command.position = gripper_positions[indexToInspect];
                feedback.status = "Moving gripper";
                as->publishFeedback(feedback);
                gripper_ac.sendGoal(goal);
                actionlib::SimpleClientGoalState gripper_state = traj_ac.getState();
                ROS_INFO("Gripper action finished: %s", gripper_state.toString().c_str());
                ros::Duration(0.5).sleep(); // sleep for half a second
                if (gripper_state.toString() == "SUCCEEDED" || gripper_state.toString() == "LOST")
                {
                    feedback.status = "Moved gripper";
                    as->publishFeedback(feedback);
                }
                else
                {
                    feedback.status = "Failed to move gripper at this inspection point.";
                    as->publishFeedback(feedback);
                    as->setAborted();
                    return;
                 }
            }
        }
        feedback.status = "At inspection point";
        as->publishFeedback(feedback);
        as->setSucceeded();
    }
    else if (goal->request == CLOSE)    // User wants to close the gripper
    {
        const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
        moveit::core::RobotStatePtr current_gripper_state;
        std::vector<double> gripper_joint_group_positions;
        current_gripper_state = gripper_move_group.getCurrentState();
        current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

        // Hard-coded close value, update as needed (or use F/T sensor)
        if (my_abs(gripper_joint_group_positions[0] - 0.8f) > 5e-3)  // If gripper needs to be moved
        {
            control_msgs::GripperCommandGoal goal;
            goal.command.position = 0.8;
            feedback.status = "Closing gripper";
            as->publishFeedback(feedback);
            gripper_ac.sendGoal(goal);
            ros::Duration(1.5).sleep();
            current_gripper_state = gripper_move_group.getCurrentState();
            current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);
            feedback.status = "Closed gripper";
            as->publishFeedback(feedback);
        }

        as->setSucceeded();
    }
    else if (goal->request == OPEN) // User wants to open the gripper
    {
        const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
        moveit::core::RobotStatePtr current_gripper_state;
        std::vector<double> gripper_joint_group_positions;
        current_gripper_state = gripper_move_group.getCurrentState();
        current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

        // Hard-coded open value, update as needed (or use F/T sensor)
        if (my_abs(gripper_joint_group_positions[0] - 0.1f) > 5e-2) 
        {
            control_msgs::GripperCommandGoal goal;
            goal.command.position = 0.1f;
            feedback.status = "Opening gripper";
            as->publishFeedback(feedback);
            gripper_ac.sendGoal(goal);
            ros::Duration(1.5).sleep();
            current_gripper_state = gripper_move_group.getCurrentState();
            current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);
            if (my_abs(gripper_joint_group_positions[0] - 0.1f) > 5e-2)
            {
                feedback.status = "Failed to open gripper";
                as->publishFeedback(feedback);
                as->setAborted();
                return;
            }
            feedback.status = "Opened gripper";
            as->publishFeedback(feedback);
        }

        as->setSucceeded();
    }
    else if (goal->request == PREVIEW)  // Show a preview of the robot's planned trajectory prior to execution
    {
        //ROS_INFO("Preview");
        // We just received a new demo, so we should clear our current stored trajectory list
        ROS_INFO("Previewing demonstration with %d waypoints\n", waypoints.size());
        trajs_msg.trajectories.clear();
  
        bool success;
        move_group->setPlanningTime(MAX_TIME);
        move_group->setMaxVelocityScalingFactor(VEL_SCALE);
        move_group->setMaxAccelerationScalingFactor(ACC_SCALE);
        move_group->setStartStateToCurrentState();
        const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        for (int i = 0; i < waypoints.size(); i++)
        {
            move_group->setGoalJointTolerance(0.001);
            move_group->setJointValueTarget(waypoints[i]);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            feedback.status = "Planning a path to waypoint " + std::to_string(i);
            as->publishFeedback(feedback);
            bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            int attempts = 0;
            while ((success == false) && (attempts < MAX_ATTEMPTS))
            {
                feedback.status = "Could not find a path to waypoint " + std::to_string(i) + ". Replanning";
                as->publishFeedback(feedback);
                ROS_INFO("Plan failed. Replanning");
                move_group->setJointValueTarget(waypoints[i]);
                success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                attempts += 1;
            }
            if (!success)
            {
                feedback.status = "Unable to plan a path for this demonstration. Please provide a new demonstration.";
                as->publishFeedback(feedback);
                as->setAborted();
                ROS_INFO("Aborting goal since could not plan to waypoint");
                return;
            }
            feedback.status = "Planned path to waypoint " + std::to_string(i);
            as->publishFeedback(feedback);
            trajs_msg.trajectories.push_back(my_plan.trajectory_);

            moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
            current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
            move_group->setStartState(*current_state);
        }

        trajs_pub.publish(trajs_msg);
        feedback.status = "Showing preview of planned demonstration trajectory";
        as->publishFeedback(feedback);

        std::cout << "Published trajectories" << std::endl;
        as->setSucceeded();
    }
    else if (goal->request == EXECUTE)  // Preview is over, and now the user wants to execute the trajectory
    {
        const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
        move_group->setMaxVelocityScalingFactor(VEL_SCALE);
        move_group->setMaxAccelerationScalingFactor(ACC_SCALE);
        std::vector<moveit_msgs::RobotTrajectory>::iterator it = trajs_msg.trajectories.begin();
        moveit::core::RobotStatePtr current_gripper_state;
        std::vector<double> gripper_joint_group_positions;

        move_group->setStartStateToCurrentState();

        feedback.status = "Beginning to execute demonstration";
        as->publishFeedback(feedback);
        for (int i = 0; i < trajs_msg.trajectories.size(); i++)
        {
            if (it->joint_trajectory.points.size() == 0) // Invalid
            {
                feedback.status = "Unable to execute this demonstration. Please provide a new demonstration.";
                as->publishFeedback(feedback);
                as->setAborted();
                ROS_INFO("One of the trajectories to a waypoint was invalid.");
                return;
            }
            else
            {
                // Move to initial position
                moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
                std::vector<double> joint_group_positions;
                current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

                std::vector<double> goal_joint_positions;
                goal_joint_positions.resize(6);
                goal_joint_positions[0] = it->joint_trajectory.points[0].positions[0];
                goal_joint_positions[1] = it->joint_trajectory.points[0].positions[1];
                goal_joint_positions[2] = it->joint_trajectory.points[0].positions[2];
                goal_joint_positions[3] = it->joint_trajectory.points[0].positions[3];
                goal_joint_positions[4] = it->joint_trajectory.points[0].positions[4];
                goal_joint_positions[5] = it->joint_trajectory.points[0].positions[5];

                bool close_enough = true;
                for (int j = 0; j < 6; j++)
                {
                    if (my_abs((joint_group_positions[j] + (2*PI)) - goal_joint_positions[j]) < 5e-2)
                    {
                        goal_joint_positions[j] -= (2*PI);
                        it->joint_trajectory.points[0].positions[j] -= (2*PI);
                    }
                    else if (my_abs((joint_group_positions[j] - (2*PI)) - goal_joint_positions[j]) < 5e-2)
                    {
                        goal_joint_positions[j] += (2*PI);
                        it->joint_trajectory.points[0].positions[j] += (2*PI);
                    }

                    float difference = joint_group_positions[j] - goal_joint_positions[j];
                    if (my_abs(difference) > 5e-2) 
                    {
                        close_enough = false; 
                    }
                }

                if (!close_enough)
                {
                    ROS_INFO("Moving to initial point of the trajectory to the waypoint");
                    move_group->setGoalJointTolerance(0.001);
                    move_group->setJointValueTarget(goal_joint_positions);
                    bool success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if (!success) 
                    {
                        ROS_INFO ("Could not move to the first point in the trajectory to this waypoint");
                        ROS_INFO("goal joint positions: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f", 
                            goal_joint_positions[0],
                            goal_joint_positions[1],
                            goal_joint_positions[2],
                            goal_joint_positions[3],
                            goal_joint_positions[4],
                            goal_joint_positions[5]);
                        feedback.status = "Unable to execute this demonstration. Please provide a new demonstration.";
                        as->publishFeedback(feedback);
                        as->setAborted();
                        return;
                    }
                }

                ROS_INFO("Moving to the next waypoint");
                moveit_msgs::ExecuteTrajectoryGoal goal;
                goal.trajectory = *it;
                traj_ac.sendGoal(goal);
                if (traj_ac.waitForResult(goal.trajectory.joint_trajectory.points[goal.trajectory.joint_trajectory.points.size()-1].time_from_start + ros::Duration(5)))
                {
                    actionlib::SimpleClientGoalState state = traj_ac.getState();
                    ROS_INFO("Action finished: %s", state.toString().c_str());
                    if (state.toString() == "SUCCEEDED")
                    {
                        feedback.status = "Moved to waypoint " + std::to_string(i);
                        as->publishFeedback(feedback);
                    }
                    else if (i != 0)
                    {
                        feedback.status = "Could not move to waypoint " + std::to_string(i) + ". Please reexecute or provide a new demonstration.";
                        as->publishFeedback(feedback);
                        as->setAborted();
                        return;
                    }
                    ros::Duration(0.5).sleep(); // sleep for half a second
                } 
                else 
                {
                    feedback.status = "Could not move to waypoint " + std::to_string(i) + ". Please reexecute or provide a new demonstration.";
                    as->publishFeedback(feedback);
                    ROS_INFO("Action server could not execute trajectory");
                    as->setAborted();
                    return;
                }

                if (gripper_positions[i] != 0.0)    // 0.0 means the gripper action for this waypoint has not been set yet
                {
                    if (gripper_positions[i] == 0.5)
                    {
                        gripper_positions[i] = 0.8;
                    }
                    current_gripper_state = gripper_move_group.getCurrentState();
                    current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

                    if (my_abs(gripper_joint_group_positions[0] - gripper_positions[i]) > 5e-2) 
                    {
                        ROS_INFO("Gripper goal: %f", gripper_positions[i]);
                        control_msgs::GripperCommandGoal goal;
                        goal.command.position = gripper_positions[i];
                        feedback.status = "Moving gripper";
                        gripper_ac.sendGoal(goal);
                        actionlib::SimpleClientGoalState gripper_state = traj_ac.getState();
                        ROS_INFO("Gripper action finished: %s", gripper_state.toString().c_str());
                        ros::Duration(0.5).sleep();   // Sleep for half a second
                        if (gripper_state.toString() == "SUCCEEDED")
                        {
                            feedback.status = "Moved gripper";
                            as->publishFeedback(feedback);
                        }
                        else
                        {
                            feedback.status = "Failed to move gripper. Please reexecute or provide a new demonstration.";
                            as->publishFeedback(feedback);
                            as->setAborted();
                            return;
                        }
                    }
                }
            }
            ++it;
        } 

        // If we got to this point, then the demonstration trajectory execution was a success
        result.conclusion = "SUCCESS";
        feedback.status = "Successfully completed demonstration execution";
        as->publishFeedback(feedback);
        as->setSucceeded(result);
    }
    else if (goal->request == CLEAR)    // User wants to clear current demonstration
    {
        trajs_msg.trajectories.clear();
        gripper_positions.clear();
        waypoints.clear();
        feedback.status = "Cleared waypoints";
        as->publishFeedback(feedback);
        as->setSucceeded();
    }
    else if (goal->request == PICK) // NOTE: This request corresponds to the "snap" functionality. 
    {
        bool success;
        move_group->setPlanningTime(MAX_TIME);
        move_group->setMaxVelocityScalingFactor(VEL_SCALE);
        move_group->setMaxAccelerationScalingFactor(0.01);
        move_group->setStartStateToCurrentState();
        const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        geometry_msgs::Pose above_object;
        tf2::Quaternion orientation;
        tf2::Quaternion obj_quat;
        tf2::Matrix3x3 m;
        geometry_msgs::Pose obj_xform = tracked_object_xform_dict[objectToPick];
        tf2::fromMsg(obj_xform.orientation, obj_quat);
        double roll, pitch, yaw;
        m = tf2::Matrix3x3(obj_quat);
        m.getRPY(roll, pitch, yaw);
        orientation.setRPY(M_PI, 0, (yaw + (M_PI/2)));
        above_object.orientation = tf2::toMsg(orientation);

        feedback.status = "Adding waypoints to object. Please do not move the object or the robot.";
        as->publishFeedback(feedback);

        /*** Pre-grasp approach ***/
        above_object.position.x = obj_xform.position.x;
        above_object.position.y = obj_xform.position.y;
        above_object.position.z = obj_xform.position.z + .3;    // .3 meters above object
        move_group->setPoseTarget(above_object);
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        int attempts = 0;
        while ((success == false) && (attempts < MAX_ATTEMPTS)) // Plan failed, replanning
        {
            feedback.status = "Could not plan an approach to the object. Replanning";
            as->publishFeedback(feedback);
            ROS_INFO("Plan failed. Replanning");
            move_group->setPoseTarget(above_object);
            success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            attempts += 1;
        }
        if (!success)   // No plan found
        {
            feedback.status = "Sorry, I was unable to plan a path to this object";
            as->publishFeedback(feedback);
            as->setAborted();
            ROS_INFO("Aborting goal since could not plan an approach to the object");
            return;
        }

        // A plan was found
        snap_wpt_msg.joints.clear();

        // Parse trajectory
        int i = 0;
        int j = 0;
        for (trajectory_msgs::JointTrajectoryPoint point: my_plan.trajectory_.joint_trajectory.points)
        {
            if ((i != 0) && ((i % 5) == 0) || i == (my_plan.trajectory_.joint_trajectory.points.size() - 1))    // For pre-grasp approach, we will only retrieve every five waypoints or so
            {
                snap_wpt_msg.wpt_type = -1;
                snap_wpt_msg.joints = point.positions;
                snap_wpt_msg.names = my_plan.trajectory_.joint_trajectory.joint_names;
                snap_wpt_pub.publish(snap_wpt_msg);
                ros::Duration(0.5).sleep(); // Wait for Unity to show waypoint
                j += 1;
            }
            i += 1;
        }
        std::cout << j << std::endl;

        // Update state for MoveIt! to prepare for grasp
        moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
        current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
        move_group->setStartState(*current_state);

        /*** Grasp ***/
        above_object.position.x = obj_xform.position.x;
        above_object.position.y = obj_xform.position.y;
        above_object.position.z = obj_xform.position.z + .22;   // Hover above the object by half the gripper size
        move_group->setPoseTarget(above_object);
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        attempts = 0;
        while ((success == false) && (attempts < MAX_ATTEMPTS)) // Plan failed. Replanning
        {
            feedback.status = "Could not plan a grasp for this object. Replanning";
            as->publishFeedback(feedback);
            ROS_INFO("Plan failed. Replanning");
            move_group->setPoseTarget(above_object);
            success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            attempts += 1;
        }
        if (!success)   // No plan found
        {
            feedback.status = "Sorry, I was unable to plan a grasp for this object.";
            as->publishFeedback(feedback);
            as->setAborted();
            ROS_INFO("Aborting goal since could not plan a grasp for the object");
            return;
        }

        // A plan was found
        snap_wpt_msg.joints.clear();

        // Parse trajectory
        i = 0;
        j = 0;
        for (trajectory_msgs::JointTrajectoryPoint point: my_plan.trajectory_.joint_trajectory.points)  // For the grasp trajectory, we will use every other waypoint
        {
            if (i == (my_plan.trajectory_.joint_trajectory.points.size() - 1))
            {
                if (i == (my_plan.trajectory_.joint_trajectory.points.size() - 1))  // wpt_type of 1 requires a close grip action associated with it
                {
                    snap_wpt_msg.wpt_type = 1;
                }
                else
                {
                    snap_wpt_msg.wpt_type = -1;
                }
                snap_wpt_msg.joints = point.positions;
                snap_wpt_msg.names = my_plan.trajectory_.joint_trajectory.joint_names;
                snap_wpt_pub.publish(snap_wpt_msg);
                ros::Duration(0.5).sleep(); // Wait for Unity to show waypoint
                j+= 1;

            }
            i += 1;
        }
        ROS_INFO("# waypoints in grasp: ");
        std::cout << j << std::endl;

        // Update state for MoveIt! to prepare for post-grasp
        current_state = move_group->getCurrentState();
        current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
        move_group->setStartState(*current_state);

        /*** Post-grasp retreat ***/
        above_object.position.x = obj_xform.position.x;
        above_object.position.y = obj_xform.position.y;
        above_object.position.z = obj_xform.position.z + .4;
        move_group->setPoseTarget(above_object);
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        attempts = 0;
        while ((success == false) && (attempts < MAX_ATTEMPTS))
        {
            feedback.status = "Could not plan a post-grasp retreat for this object. Replanning";
            as->publishFeedback(feedback);
            ROS_INFO("Plan failed. Replanning");
            move_group->setPoseTarget(above_object);
            success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            attempts += 1;
        }
        if (!success)
        {
            feedback.status = "Sorry, I was unable to plan a post-grasp retreat for this object.";
            as->publishFeedback(feedback);
            as->setAborted();
            ROS_INFO("Aborting goal since could not plan a post-grasp retreat for the object");
            return;
        }

        snap_wpt_msg.joints.clear();
        // Parse trajectory
        i = 0;
        j = 0;
        for (trajectory_msgs::JointTrajectoryPoint point: my_plan.trajectory_.joint_trajectory.points)
        {
            if (((i != 0) && ((i % 2) == 0)) || i == (my_plan.trajectory_.joint_trajectory.points.size() - 1))
            {
                if (i == (my_plan.trajectory_.joint_trajectory.points.size() - 1))
                {
                    snap_wpt_msg.wpt_type = 2;
                }
                else
                {
                    snap_wpt_msg.wpt_type = -1;
                }
                snap_wpt_msg.joints = point.positions;
                snap_wpt_msg.names = my_plan.trajectory_.joint_trajectory.joint_names;
                snap_wpt_pub.publish(snap_wpt_msg);
                ros::Duration(0.5).sleep();
                j += 1;
            }
            i += 1;
        }
        ROS_INFO("# waypoints in post-grasp: ");
        std::cout << j << std::endl;
        current_state = move_group->getCurrentState();
        current_state->setJointGroupPositions(joint_model_group, my_plan.trajectory_.joint_trajectory.points.back().positions);
        move_group->setStartState(*current_state);
        
        feedback.status = "Successfully added waypoints to object";
        as->publishFeedback(feedback);
        as->setSucceeded();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // Assign shared pointer with an address to move_group 
    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));

    if (!node_handle.getParam("move_group/planning_plugin", planner_plugin_name))
    {
        ROS_FATAL_STREAM("Could not find planner plugin name");
    }

    ns = node_handle.getNamespace();

    Server server(node_handle, "execute_path", boost::bind(&execute, _1, &server), false);
    server.start();

    trajs_pub = node_handle.advertise<demoshop_ubuntu::WaypointTrajectories>("trajectories", 1000);
    tracked_object_pose_pub = node_handle.advertise<geometry_msgs::TransformStamped>("tracked_object_poses", 1000);
    snap_wpt_pub = node_handle.advertise<demoshop_ubuntu::SnapWaypoint>("addSnapWaypoints", 1000);

    addCollisionObjects();

    ros::Subscriber sub = node_handle.subscribe("addWaypoint", 1000, addWaypointCallback);      // Subscriber for obtaining automatically generated demonstration waypoints
    ros::Subscriber del_sub = node_handle.subscribe("indicesToDel", 1000, deleteCallback);      // Subscriber for obtaining indices of user-specified waypoints that need to be deleted from the demonstration
    ros::Subscriber add_sub = node_handle.subscribe("indicesToAdd", 1000, addCallback);         // Subscriber for obtaining indices of user-specified waypoints that need to be added into the demonstration
    ros::Subscriber insp_sub = node_handle.subscribe("indicesToInsp", 1000, inspCallback);      // Subscriber for obtaining indices of user-specified waypoints from the demonstration that need to be inspected
    ros::Subscriber grip_sub = node_handle.subscribe("indicesToGrip", 1000, gripCallback);      // Subscriber for obtaining indices of user-specified waypoints from the demonstration that need to change gripper actions
    ros::Subscriber transform_sub = node_handle.subscribe("tf", 1000, transformCallback);       // Subscriber for obtaining transforms of tracked objects
    ros::Subscriber close_sub = node_handle.subscribe("amountToGrip", 1000, closeCallback);     // Subscriber for obtaining gripper actions
    ros::Subscriber pick_sub = node_handle.subscribe("objectToPick", 1000, pickCallback);       // NOTE: This subscriber is associated with the "snap" feature.
    ros::Subscriber snap_sub = node_handle.subscribe("snapWaypoints", 1000, snapCallback);      // NOTE: This subscriber is associated with the "snap" feature.

    ros::waitForShutdown();
    return 0;
}

