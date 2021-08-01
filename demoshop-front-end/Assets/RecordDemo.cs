// RecordDemo.cs

using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages;
using UnityEngine.UI;
using UnityEngine;


using sensor_msgs = RosSharp.RosBridgeClient.Messages.Sensor;
using geometry_msgs = RosSharp.RosBridgeClient.Messages.Geometry;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;
using moveit_msgs = RosSharp.RosBridgeClient.Messages.MoveIt;
using trajectory_msgs = RosSharp.RosBridgeClient.Messages.Trajectory;
using tf2_msgs = RosSharp.RosBridgeClient.Messages.Tf2;
using msgs = RosSharp.RosBridgeClient.Messages;
using rosapi = RosSharp.RosBridgeClient.Messages.Rosapi;

namespace RosSharp.RosBridgeClient
{
    public class RecordDemo : MonoBehaviour
    {
        // Public variables
        // GUI
        [HideInInspector] 
        public bool isMoving = true;    // Indicates whether user is in select mode or not
        [HideInInspector] 
        public bool isDragging = false;    // Indicates whether user is dragging a window or not
        [HideInInspector] 
        public bool clickedButton = false;  // Indicates whether user last clicked on a button or elsewhere
        [HideInInspector]
        public bool displayMenu;    // Indicates whether waypoints menu should currently be displayed
        public GUIStyle buttonStyle;
        public GUIStyle yellowButtonStyle;
        public GUIStyle redButtonStyle;
        public GUIStyle inspectStyle;
        public GUIStyle stopStyle;
        public GUIStyle menuFont;
        public GUIStyle wptMenuButtonStyle;
        public bool gripChange = false;

        // Robot joint states
        public List<string> JointNames;
        public List<JointStateWriter> JointStateWriters;
        public List<JointStateReader> JointStateReaders;

        // Private variables
        // Game objects 
        private GameObject gripper;
        private GameObject inputIndex;
        private GameObject confirmation;
        private GameObject assist;
        private GameObject confirmObject;
        private GameObject correctObject;
        private GameObject confirmSnap;
        private GameObject closeWarning;
        // ROS Sharp
        private RosSocket rosSocket;
        private string wpt_publication_id;  
        private string add_publication_id;
        private string del_publication_id;   
        private string grip_publication_id;
        private string close_publication_id;
        private string pick_publication_id;
        private string insp_publication_id;
        private string snap_publication_id;
        private string trajs_subscription_id;
        private string pick_subscription_id;
        private string tf_subscription_id;
        private string snap_subscription_id;
        private string snap_wpt_subscription_id;
        private std_msgs.Bool wpt_msg = new std_msgs.Bool();  			// Waypoints are ROS PoseStamped messages
        private std_msgs.Int32 add_msg = new std_msgs.Int32();  		// Message contains an index to add to in ROS waypoint vector
        private std_msgs.Int32 del_msg = new std_msgs.Int32();  		// Message contains an index to delete from in ROS waypoint vector
        private std_msgs.Int32 insp_msg = new std_msgs.Int32(); 		// Message contains the index of the waypoint to which to move the robot
        private std_msgs.Int32 grip_msg = new std_msgs.Int32(); 		// Message contains the index of the waypoint that needs to change gripper actions
        private std_msgs.Float32 close_msg = new std_msgs.Float32();    // Message contains gripper position value
        private std_msgs.String pick_msg = new std_msgs.String();
        //private tf2_msgs.TFMessage transform_msg = new tf2_msgs.TFMessage();
        private geometry_msgs.TransformStamped transform_msg = new geometry_msgs.TransformStamped();
        private msgs.SnapWaypoints snap_wpts_msg; // = new demoshop_ubuntu.SnapWaypoints();
        ExecutePathActionClient client; 

        // Waypoints
        private List<GameObject> generatedWaypoints = new List<GameObject>();    // Contains waypoints currently in the scene
        [SerializeField]
        private GameObject waypoint; // Specify waypoint prefab
        // Windows
        [SerializeField]
        private GameObject inputIndexBox; // Specify index input window prefab
        /*[SerializeField]
        private GameObject warningBox; // Specify warning window prefab*/
        [SerializeField]
        private GameObject confirmBox;
        [SerializeField]
        private GameObject assistBox;
        [SerializeField]
        private GameObject confirmObjectBox;
        [SerializeField]
        private GameObject correctObjectBox;
        [SerializeField]
        private GameObject confirmSnapBox;
        [SerializeField]
        private GameObject closeWarningBox;

        // Waypoint parameters and variables
        [SerializeField]
        private float rate = 0.02f;  			// Specify rate at which waypoints are added to the scene and published
        [SerializeField]
        private float min_distance = 0.1f;   	// Specifiy minimum distance between added waypoints
        private Vector3 last_position;  		// Keeps track of last position where a waypoint was added to prevent multiple waypoints near the same position
        private Gradient gradient;
        private GradientColorKey[] colorKey;
        private GradientAlphaKey[] alphaKey;
        [SerializeField]
        private Color selected; // Color to highlight selected objects
        [SerializeField]
        private Color specified; 
        [SerializeField]
        private Color snapPreviewed; 
        private int numSelected;    // Number of waypoints that are currently selected
        // Trajectory
        private TrailRenderer tr;
        // GUI
        private bool hasStarted = false;    // Specifies whether a demo is currently running
        private bool trailSeen = false; // Specifies whether a trail is currently visualized
        private bool showExecute = true;
        private const byte CLEAR = 0;   // Request to send to ExecutePath action server when desire to clear demo
        private const byte INSPECT = 1; // Request to send to ExecutePath action server when desire to inspect a waypoint
        private const byte PREVIEW = 2;     // Request to send to ExecutePath action server when ready to execute demo on robot
        private const byte OPEN = 3;
        private const byte CLOSE = 4;
        private const byte EXECUTE = 5;
        private const byte PICK = 6;
        
        // Menu parameters and variables
        private Vector2 menu_position;
        private int requested_index = -1;

        // Robot joints
        private string[] jointNames = new string[7]; // MODIFY THE SIZE OF THE ARRAY ACCORDING TO THE NUMBER OF JOINTS YOUR ROBOT HAS
        private float[] jointStates = new float[7];  // MODIFY THE SIZE OF THE ARRAY ACCORDING TO THE NUMBER OF JOINTS YOUR ROBOT HAS

        // GUI flags
        private bool inPreview = false;
        private bool inWptPreview = false;
        private bool showConfirmation = false;
        private const float INIT_RATE = 0.1f;

        // Robot trajectory information
        private const float TRAJ_RATE = 0.02f;
        private List<float> gripper_positions = new List<float>();
        private bool gripperSet = false;
        float gripper_pos = 0.0f;
        [HideInInspector]
        public int finger_index;
        float gripperPos;

        // Double tap variables and parameters
        int TapCount;
        public float MaxDoubleTapTime = 0.1f;
        float NewTime;

        // Tracked object variables and parameters
        [SerializeField]
        private GameObject trackedObject; // Specify tracked object prefab
        Dictionary<string, GameObject>  tracked_object_dict = new Dictionary<string, GameObject>(StringComparer.OrdinalIgnoreCase);
        private bool tracking;
        private string child_frame_name;
        private Vector3 tracked_translation;
        private Quaternion tracked_rotation;
        private uint tracked_secs;
        [SerializeField]
        private Color highlighted; 
        [SerializeField]
        private Color regular; 
        private string closest_task_object;

        // User assistance variables and parameters
        private int stay_counter = 0;
        private bool robotShown = true;
        private UnityEngine.UI.Image status_bg;
        private UnityEngine.UI.Text status;
        private UnityEngine.UI.Toggle toggle;
        private bool isRobotRunning = false;
        private string selStatus;
        private int indexToClose = -1;
        private int indexToOpen = -1;
        private bool lastClosed = false;
        private bool lastOpen = false;
        private bool addedUserWpt = false;
        private bool stopAutoGrip = false;

        private bool responded_to_help = false;

        private bool snapSet = false;
        private List<moveit_msgs.RobotTrajectory> snapTrajectories = new List<moveit_msgs.RobotTrajectory>();
        private List<GameObject> snapWaypoints = new List<GameObject>();
        
        private List<float> snap_gripper_positions = new List<float>();
        private Vector3 snap_last_position;
        bool showSnapConfirmation = false;
        bool addSnapWaypoints = false;
        bool inSnapPreview = false;
        //bool should_close_gripper = false;
        float[] snapJointConfig = new float[7]; // MODIFY THE SIZE OF THE ARRAY ACCORDING TO THE NUMBER OF JOINTS YOUR ROBOT HAS
        private bool finishedSnap;

        private List<List<string>> snapJointNames = new List<List<string>>();
        private List<List<double>> snapJoints = new List<List<double>>();
        private List<int> snapTypes = new List<int>();
        private bool snapStarted = false;

        private uint currTime;

        // Dictionary for translating Unity joint names to ROS and vice versa
        Dictionary<string, string> ros_unity_joints = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase) // MODIFY THE DICTIONARY TO REFLECT THE JOINT NAMES OF YOUR ROBOT ON ROS AND UNITY
        {
            {"shoulder_pan_joint", "shoulder_link"},
            {"shoulder_lift_joint", "upper_arm_link"},
            {"elbow_joint", "forearm_link"},
            {"wrist_1_joint", "wrist_1_link"},
            {"wrist_2_joint", "wrist_2_link"},
            {"wrist_3_joint", "wrist_3_link"},
            {"finger_joint", "right_outer_knuckle"}
        };

        /*** Setup ***/
        void Start()	
        {
            gripper = GameObject.Find("fts_toolside");  // End effector link
            rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol("ws://192.168.0.139:9090"));  // IP address of ROS server, change as needed
            wpt_publication_id = rosSocket.Advertise<std_msgs.Bool>("addWaypoint");  // Publishing to "/waypoints" topic on ROS
            del_publication_id = rosSocket.Advertise<std_msgs.Int32>("indicesToDel");   // Publishing to "/indicesToDel" topic on ROS
            add_publication_id = rosSocket.Advertise<std_msgs.Int32>("indicesToAdd");   // Publishing to "/indicesToAdd" topic on ROS
            insp_publication_id = rosSocket.Advertise<std_msgs.Int32>("indicesToInsp"); // Publishing to "/indicesToInsp" topic on ROS
            grip_publication_id = rosSocket.Advertise<std_msgs.Int32>("indicesToGrip"); // Publishing to "/indicesToGrip" topic on ROS
            close_publication_id = rosSocket.Advertise<std_msgs.Float32>("amountToGrip");   // Publishing to "/amountToGrip" topic on ROS
            pick_publication_id = rosSocket.Advertise<std_msgs.String>("objectToPick");
            snap_publication_id = rosSocket.Advertise<SnapWaypoints>("snapWaypoints");
            trajs_subscription_id = rosSocket.Subscribe<WaypointTrajectories>("trajectories", TrajsPlanHandler);  // Subscribing to "/trajectories" topic on ROS
            snap_wpt_subscription_id = rosSocket.Subscribe<SnapWaypoint>("addSnapWaypoints", SnapWaypointHandler);
            tf_subscription_id = rosSocket.Subscribe<geometry_msgs.TransformStamped>("tracked_object_poses", TransformHandler);	 // Subscribing to "/tracked_object_poses" topic on ROS
            client = gripper.GetComponent<ExecutePathActionClient>();
            gradient = new Gradient();
            tr = GameObject.Find("trail").GetComponent<TrailRenderer>();

            colorKey = new GradientColorKey[2];
            colorKey[0].color = Color.blue; // Oldest waypoints are blue
            colorKey[0].time = 0.0f;
            colorKey[1].color = Color.red; // Newest waypoints are red
            colorKey[1].time = 1.0f;

            // All waypoints should be opaque
            alphaKey = new GradientAlphaKey[2];
            alphaKey[0].alpha = 1.0f;
            alphaKey[0].time = 0.0f;
            alphaKey[1].alpha = 1.0f;
            alphaKey[1].time = 1.0f;

            gradient.mode = GradientMode.Blend;
            gradient.SetKeys(colorKey, alphaKey);

            tr.enabled = false; // Don't want the trail rendered unless the user specifies it should be
            trailSeen = false;

            TapCount = 0;

            status_bg = GameObject.Find("StatusBG").GetComponent<Image>();
            status = GameObject.Find("PreviewIndicator").GetComponent<Text>();
            toggle = GameObject.Find("Toggle").GetComponent<Toggle>();
        }

        /*** Repeating functions ***/
        List<double> lastSnapJoints;// = new List<float>();
        int lastSnapType;
        void Update()
        {
            GameObject.Find("RosConnector").GetComponent<JointStateSubscriber>().joint_names.CopyTo(jointNames, 0);
            GameObject.Find("RosConnector").GetComponent<JointStateSubscriber>().joint_states.CopyTo(jointStates, 0);

            rosSocket.CallService<rosapi.GetTimeRequest, rosapi.GetTimeResponse>("rosapi/get_time", TimeServiceHandler, new rosapi.GetTimeRequest());

            if (robotShown == true)
            {
                GameObject.Find("right_outer_knuckle").GetComponent<JointStateReader>().Read(out string name, out float position, out float velocity, out float effort);
                gripperPos = position;
            }

            UpdateWaypoints();  // Update indices and display of waypoints

            if (indexToClose != -1)
            {
                if (indexToClose < generatedWaypoints.Count)
                {
                    WaypointScript script = generatedWaypoints[indexToClose].GetComponent<WaypointScript>();
                    grip_msg.data = indexToClose;
                    rosSocket.Publish(grip_publication_id, grip_msg);
                    script.SetGripperPosition(0.5f);
                    close_msg.data = 0.5f;
                    gripper_positions[grip_msg.data] = 0.5f;
                    rosSocket.Publish(close_publication_id, close_msg);
                    indexToClose = -1;
                }
            }

            if (indexToOpen != -1)
            {
                if (indexToOpen < generatedWaypoints.Count)
                {
                    WaypointScript script = generatedWaypoints[indexToOpen].GetComponent<WaypointScript>();
                    grip_msg.data = indexToOpen;
                    rosSocket.Publish(grip_publication_id, grip_msg);
                    script.SetGripperPosition(0.1f);
                    close_msg.data = 0.1f;
                    gripper_positions[grip_msg.data] = 0.1f;
                    rosSocket.Publish(close_publication_id, close_msg);
                    indexToOpen = -1;
                }
            }

            List<GameObject> selectedObjects = Camera.main.GetComponent<Click>().selectedObjects;

            if (inPreview == false && inWptPreview == false && inSnapPreview == false) // Only update Unity to reflect the physical robot's configuration if not currently previewing plan
            {
                showPhysicalRobot();
                if (isRobotRunning == false)
                {
                    if (numSelected > 0)	// Selection status bar
                    {
                        status_bg.color = Color.blue;
                        status.color = Color.white;
                        selStatus = "Selected waypoints: ";
                        foreach (GameObject selected in selectedObjects)
                        {
                            selStatus += selected.GetComponent<WaypointScript>().GetIndex().ToString();
                            selStatus += ", ";
                        }
                        status.text = selStatus.Substring(0, selStatus.Length-2);
                    }
                    else
                    {
                        status_bg.color = new Color(1.0f, 1.0f, 1.0f, 0f);
                        status.color = new Color(1.0f, 1.0f, 1.0f, 0f);
                    }
                }
            }
            else 
            {
                status_bg.color = Color.black;
                status.color = Color.white;
                status.text = "Preview mode";
            }

            if (numSelected == 1)
            {
                bool invisible = (toggle.isOn == false);
                WaypointScript waypoint_script = selectedObjects[0].GetComponent<WaypointScript>();
                if (waypoint_script.pressed == true)    // If the user's mouse is pressed down on a waypoint, preview the robot's joint configuration at that waypoint (make sure robot is visible)
                {
                    if (invisible)
                    {
                        toggle.isOn = true;
                        GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                    }
                    inWptPreview = true;
                    showJointConfig(jointNames, waypoint_script.GetJointConfiguration());
                }
                else if (waypoint_script.released == true && inSnapPreview == false)
                {
                    if (invisible)
                    {
                        toggle.isOn = false;
                        GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
                    }
                    showPhysicalRobot();
                    waypoint_script.pressed = false;
                    waypoint_script.released = false;
                }
            }

            if (showConfirmation == true)
            {
                toggle.interactable = true;
                confirmation = (GameObject) Instantiate(confirmBox, confirmBox.transform.position, confirmBox.transform.rotation);
                confirmation.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
                Button execute = GameObject.Find("executeButton").GetComponent<Button>();
                execute.onClick.AddListener(executeOnRobot);
                Button cancel = GameObject.Find("cancelButton").GetComponent<Button>();
                cancel.onClick.AddListener(cancelExecution);
                showConfirmation = false;
            }

            if (Input.touchCount == 1) 
            {
                Touch touch = Input.GetTouch(0);
                 
                if (touch.phase == TouchPhase.Ended) 
                {
                    TapCount += 1;
                }

                if (TapCount == 1) 
                {
                    NewTime = Time.time + MaxDoubleTapTime;

                    if (clickedButton == false)
                    {
                        displayMenu = false;
                    }
                }
                else if(TapCount == 2 && Time.time <= NewTime)
                {
                    isMoving = true;
                    displayMenu = true;
                    menu_position.x = touch.position.x;
                    menu_position.y = Screen.height - touch.position.y;    // Convert from screen space to GUI space          
                    TapCount = 0;
                }
            }
                 
            if (Time.time > NewTime) 
            {
                TapCount = 0;
            }

            if (Input.GetMouseButtonDown(1))    // Right click means the user wants to see the waypoints menu
            {
                displayMenu = true;
                menu_position.x = Input.mousePosition.x;
                menu_position.y = Screen.height - Input.mousePosition.y;    // Convert from screen space to GUI space
            }

            if (numSelected > 0 && Input.GetKey(KeyCode.Delete))    // Delete waypoints if user selected waypoints and pressed 'Delete' on the keyboard
            {
                DeleteWaypoints();
            }

            // Check if user is dragging any windows
            if (inputIndex != null)
            {
                isDragging = inputIndex.GetComponent<Drag>().isDragging;
            }
            else if (confirmation != null)
            {
                isDragging = confirmation.GetComponent<Drag>().isDragging;
            }
            else if (assist != null)
            {
                isDragging = assist.GetComponent<Drag>().isDragging;
            }
            else if (confirmObject != null)
            {
                isDragging = confirmObject.GetComponent<Drag>().isDragging;
            }
            else if (correctObject != null)
            {
                isDragging = correctObject.GetComponent<Drag>().isDragging;
            }
            else if (confirmSnap != null)
            {
                isDragging = confirmSnap.GetComponent<Drag>().isDragging;
            }
            else if (closeWarning != null)
            {
                isDragging = closeWarning.GetComponent<Drag>().isDragging;
            }
            else
            {
                isDragging = false;
            }

            if (requested_index != -1)  // The user submitted a valid index for manually adding a waypoint
            {
                AddUserWaypoint();
                tr.enabled = false;
                trailSeen = false;
            }

            // Handle object tracking - YOU CAN MODIFY THE LINES BELOW TO WORK WITH YOUR (TAGGED) TASK OBJECTS
            if (tracking == true)
            {
                if (!String.Equals(child_frame_name, "ar_master_22")) // ar_master_22 is the fixed reference frame
                {
                    if (tracked_object_dict.ContainsKey(child_frame_name) == false) // First time seeing this object
                    {
                        var tracked_object = (GameObject) Instantiate(trackedObject, tracked_translation, tracked_rotation);
                        if (String.Equals(child_frame_name, "ar_master_0") || String.Equals(child_frame_name, "ar_master_0_filtered")) // ar_master_0 is a cube that we are tracking - changed the string as needed
                        {
                            tracked_object.transform.localScale = new Vector3(0.057f, 0.057f, 0.057f); // Change this line to assign the 3D model of your object to tracked_object
                        }
                        else if (String.Equals(child_frame_name, "ar_master_24_filtered")) // ar_master_24 is a box that we are tracking - changed the string as needed
                        {
                            tracked_object.transform.localScale = new Vector3(0.404f, 0.236f, 0.244f); // Change this line to assign the 3D model of your object to tracked_object
                        }
                        tracked_object_dict.Add(child_frame_name, tracked_object);
                        tracked_object.GetComponent<TrackedObjectScript>().tracked_object_id = child_frame_name;
                        tracked_object.GetComponent<TrackedObjectScript>().lastTime = tracked_secs;
                    }
                    else if (tracked_object_dict.ContainsKey(child_frame_name) == true)
                    {
                        tracked_object_dict[child_frame_name].transform.position = tracked_translation;
                        tracked_object_dict[child_frame_name].transform.rotation = tracked_rotation;
                        tracked_object_dict[child_frame_name].GetComponent<TrackedObjectScript>().lastTime = tracked_secs;
                    }
                }
                tracking = false;
            }


            List<string> keysToRemove = new List<string>();
            foreach (KeyValuePair<string, GameObject> entry in tracked_object_dict)
            {
                TrackedObjectScript trackedObjectScript = entry.Value.GetComponent<TrackedObjectScript>();

                if (((currTime - trackedObjectScript.lastTime) > 3) && (trackedObjectScript.lastTime != 0) && (currTime != 0))
                {
                    // Remove objects from dictionary and scene
                    Destroy(entry.Value);
                    keysToRemove.Add(trackedObjectScript.tracked_object_id);
                }
                else
                {
                    if (trackedObjectScript.currentlySelected == false)
                    {
                        if (trackedObjectScript.currentlyHighlighted == false)
                        {
                            entry.Value.GetComponent<Renderer>().material.SetColor("_Color", regular);
                        }
                        else
                        {
                            entry.Value.GetComponent<Renderer>().material.SetColor("_Color", highlighted);
                        }
                    }
                    else
                    {
                        entry.Value.GetComponent<Renderer>().material.SetColor("_Color", selected);
                        trackedObjectScript.currentlyHighlighted = false;
                    }   
                }
            }

            foreach (string key in keysToRemove)
            {
                tracked_object_dict.Remove(key);
            }


            keysToRemove.Clear();


            // Handle robot visibility
            if (toggle.isOn == false)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
            }
            else
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
            }

            bool invisible2 = (toggle.isOn == false);
            if (invisible2)
            {
                toggle.interactable = false;
                toggle.isOn = true;
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
            }

            for (int i = 0; i < snapJoints.Count; i++)
            {
            	inSnapPreview = true;
	            int index, mimic_index;

                if (lastSnapType == 1)
                {
	                should_grip = true;
                }
	            
                showJointConfig2(snapJointNames[i].ToArray(), snapJoints[i].ToArray(), should_grip);

	            if (!firstTime)
	            {
		            var wpt = (GameObject) Instantiate(waypoint, gripper.transform.position, gripper.transform.rotation);
		            generatedWaypoints.Add(wpt);
	                WaypointScript waypoint_script = wpt.GetComponent<WaypointScript>();
                    waypoint_script.SetAsSnap();
	                waypoint_script.SetPosition(gripper.transform.position);  // Store the gripper position at which the waypoint was added
                    if (should_grip == false)
                    {
	                   waypoint_script.SetJointConfiguration((float)lastSnapJoints[2], 0.1f, (float)lastSnapJoints[1], (float)lastSnapJoints[0], (float)lastSnapJoints[3], (float)lastSnapJoints[4], (float)lastSnapJoints[5]);
                    }
                    else
                    {
                        waypoint_script.SetJointConfiguration((float)lastSnapJoints[2], 0.5f, (float)lastSnapJoints[1], (float)lastSnapJoints[0], (float)lastSnapJoints[3], (float)lastSnapJoints[4], (float)lastSnapJoints[5]);
                    }
	                last_position = gripper.transform.position;

	                if (lastSnapType == 1)
	                {
	                	finger_index = 1;
	                	generatedWaypoints[generatedWaypoints.Count - 1].GetComponent<WaypointScript>().SetGripperPosition(0.5f);
	                	gripper_positions.Add(0.5f);
	                }

	                if (snapTypes[i] == 2)
	                {
	                	CancelInvoke("AddWaypoint");
	                	hasStarted = false;
	                	snapStarted = false;

                        confirmSnap = (GameObject) Instantiate(confirmSnapBox, confirmSnapBox.transform.position, confirmSnapBox.transform.rotation);
                        confirmSnap.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
                        Button yes = GameObject.Find("YesSnap").GetComponent<Button>();
                        Button no = GameObject.Find("NoSnap").GetComponent<Button>();
                        Button close = GameObject.Find("CloseSnap").GetComponent<Button>();
                        close.onClick.AddListener(CloseSnapWindow);
                        yes.onClick.AddListener(TellROSAboutSnap);
                        no.onClick.AddListener(CloseSnapWindow);
	                }
                        if (lastSnapType != 1)
                        {
	                	  gripper_positions.Add(0.0f);
                        }
	            }     

	            lastSnapJoints = new List<double>(snapJoints[i]); 
	            lastSnapType = snapTypes[i];         
	            firstTime = false;
            }

            snapJoints.Clear();
            snapJointNames.Clear();
            snapTypes.Clear();

            if (invisible2)
            {
                toggle.interactable = true;
                toggle.isOn = false;
                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
            }
        }

        void TellROSAboutSnap()
        {
            clickedButton = true;
            Destroy(confirmSnap);
            confirmSnap = null;

            // Tell ROS about the snap waypoints
            snap_wpts_msg = new msgs.SnapWaypoints(generatedWaypoints.Count - firstSnapIdx);
            snap_wpts_msg.waypoints = new msgs.Waypoint[generatedWaypoints.Count - firstSnapIdx];
            WaypointScript waypoint_script;
            for (int j = firstSnapIdx; j < generatedWaypoints.Count; j++)
            {
                waypoint_script = generatedWaypoints[j].GetComponent<WaypointScript>();
                snap_wpts_msg.waypoints[j-firstSnapIdx] = new msgs.Waypoint();
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values = new float[6];
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[0] = waypoint_script.GetJointConfiguration()[3];
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[1] = waypoint_script.GetJointConfiguration()[2];
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[2] = waypoint_script.GetJointConfiguration()[0];
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[3] = waypoint_script.GetJointConfiguration()[4];
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[4] = waypoint_script.GetJointConfiguration()[5];
                snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[5] = waypoint_script.GetJointConfiguration()[6];
                snap_wpts_msg.waypoints[j-firstSnapIdx].grip_action = gripper_positions[j];
                waypoint_script.SetJointConfiguration(snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[0], snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[1], snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[2], snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[3], snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[4], snap_wpts_msg.waypoints[j-firstSnapIdx].joint_values[5], waypoint_script.GetJointConfiguration()[1]);
                waypoint_script.ClearSnap();
            } 
            rosSocket.Publish(snap_publication_id, snap_wpts_msg);

            firstSnapIdx = -1;
            inSnapPreview = false;
            firstTime = true;
        }

        // AddWaypoint: Adds a waypoint at the current gripper pose
        void AddWaypoint()
        {
            bool invisible = (toggle.isOn == false);
            if (invisible)
            {
                toggle.interactable = false;
                toggle.isOn = true;
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
            }
            if (inWptPreview == false)
            {
                WaypointScript waypoint_script;
                Vector3 gripper_position;
                Quaternion gripper_rotation;
                gripper_position = gripper.transform.position;
                gripper_rotation = gripper.transform.rotation;
                //float distance_to_obj = 5;

                if (Vector3.Distance(gripper_position, last_position) < min_distance)
                {
                	stay_counter++;
                }
                else
                {
                	responded_to_help = false;
                	stay_counter = 0;
                }

                // Assist trigger - COMMENT OUT THE 11 LINES BELOW IF YOU WOULD LIKE TO DISABLE THE SNAP ASSISTANCE FUNCTIONALITY
                if (stay_counter >= 750 && assist == null && responded_to_help == false && inPreview == false && inSnapPreview == false && tracked_object_dict.Count > 0 &&  (jointStates[finger_index] < 0.3f))	// If user has rested in one spot for 750 consecutive frame updates (~12.5 s) and may need assistance gripping . . .
                {
                    if (tracked_object_dict.Count == 1)
                    {
                        HighlightOnlyObject();
                    }
                    else
                    {
                        HighlightTaskObject();
                    }
                }

                if (generatedWaypoints.Count > 0 && (snapStarted == false) /*&& (gripChange == false)*/)
                {  
                    gripChange = (stay_counter > 400 && Vector3.Distance(gripper_position, last_position) > (min_distance / 10)) && ((generatedWaypoints[generatedWaypoints.Count - 1].GetComponent<WaypointScript>().GetJointConfiguration()[finger_index] >= 0.35) && (jointStates[finger_index] < 0.35)) || ((generatedWaypoints[generatedWaypoints.Count - 1].GetComponent<WaypointScript>().GetJointConfiguration()[finger_index] < 0.35) && (jointStates[finger_index] >= 0.35));
                }

                // Only add a waypoint if it's far enough from the last one or user has been resting there a while or it involves a gripper action
                if ((Vector3.Distance(gripper_position, last_position) > min_distance || gripChange) && (snapStarted == false))
                {
                    if (stay_counter > 100 || gripChange) 
                    {
                        if (generatedWaypoints.Count > 0)
                        {
                            int closest_waypoint = GetClosestWaypoint(gripper_position);
                            Vector3 closest_position = generatedWaypoints[closest_waypoint].GetComponent<WaypointScript>().GetPosition();
                            float dist = Vector3.Distance(gripper_position, closest_position);
                            if (dist < min_distance)
                            {
                                if (closest_waypoint == generatedWaypoints.Count - 1)
                                {
                                    if (dist < 0.05 && generatedWaypoints[closest_waypoint].GetComponent<WaypointScript>().userSpecified == false)   
                                    {                                      
                                        // Tell ROS that we want to delete these waypoints
                                        int ind = generatedWaypoints[closest_waypoint].GetComponent<WaypointScript>().GetIndex();
                                        Destroy(generatedWaypoints[closest_waypoint]);  
                                        del_msg.data = ind;
                                        rosSocket.Publish(del_publication_id, del_msg);
                                        // Remove the selected waypoints from waypoint list
                                        generatedWaypoints.RemoveAt(ind);
                                        gripper_positions.RemoveAt(ind);     

                                        // Update indices of waypoints
                                        UpdateWaypoints();
                                    }
                                }
                            }
                        }
                    }
  
                    wpt_msg.data = true;
                    rosSocket.Publish(wpt_publication_id, wpt_msg);
                    var wpt = (GameObject) Instantiate(waypoint, gripper_position, gripper_rotation);
                    generatedWaypoints.Add(wpt);
                    gripper_positions.Add(0.0f);
                    waypoint_script = wpt.GetComponent<WaypointScript>();
                    waypoint_script.SetPosition(gripper_position);  // Store the gripper position at which the waypoint was added
                    waypoint_script.SetJointConfiguration((float)jointStates[0], (float)jointStates[1], (float)jointStates[2], (float)jointStates[3], (float)jointStates[4], (float)jointStates[5], (float)jointStates[6]);
                    last_position = gripper_position;
                }
                
                gripChange = false;
                if (invisible)
                {
                    toggle.interactable = true;
                    toggle.isOn = false;
                    GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
                }
            }
        }

        /*** Robot visualization ***/
        void showPhysicalRobot()
        {
                int index;
                int mimic_index;
                for (int i = 0; i < jointNames.Length; i++)
                {
                    index = JointNames.IndexOf(jointNames[i]);
                    if (index != -1)
                        JointStateWriters[index].Write((float) jointStates[i]);
                    if (String.Equals(jointNames[i], "finger_joint"))   // Update mimic joints
                    {
                        finger_index = i;
                        mimic_index = JointNames.IndexOf("right_outer_knuckle_joint");
                        if (mimic_index != -1)
                        {
                            JointStateWriters[mimic_index].Write((float) (-1 * jointStates[i]));
                        }
                        mimic_index = JointNames.IndexOf("left_inner_finger_joint");
                        if (mimic_index != -1)
                        {
                            JointStateWriters[mimic_index].Write((float) jointStates[i]);
                        }
                        mimic_index = JointNames.IndexOf("right_inner_finger_joint");
                        if (mimic_index != -1)
                        {
                            JointStateWriters[mimic_index].Write((float) jointStates[i]);
                        }
                        mimic_index = JointNames.IndexOf("left_inner_knuckle_joint");
                        if (mimic_index != -1)
                        {
                            JointStateWriters[mimic_index].Write((float) (-1 * jointStates[i]));
                        }
                        mimic_index = JointNames.IndexOf("right_inner_knuckle_joint");
                        if (mimic_index != -1)
                        {
                            JointStateWriters[mimic_index].Write((float) (-1 * jointStates[i]));
                        }
                    }
                }
                inWptPreview = false;
        }

        void showJointConfig(string[] joint_names, float[] joint_states)
        {
            int index;
            int mimic_index;
            for (int i = 0; i < joint_names.Length; i++)
            {
                index = JointNames.IndexOf(joint_names[i]);
                if (index != -1)
                    JointStateWriters[index].Write((float) joint_states[i]);
                if (String.Equals(joint_names[i], "finger_joint"))   // Update mimic joints
                {
                    finger_index = i;
                    mimic_index = JointNames.IndexOf("right_outer_knuckle_joint");
                    if (mimic_index != -1)
                    {
                        JointStateWriters[mimic_index].Write((float) (-1 * joint_states[i]));
                    }
                    mimic_index = JointNames.IndexOf("left_inner_finger_joint");
                    if (mimic_index != -1)
                    {
                        JointStateWriters[mimic_index].Write((float) joint_states[i]);
                    }
                    mimic_index = JointNames.IndexOf("right_inner_finger_joint");
                    if (mimic_index != -1)
                    {
                        JointStateWriters[mimic_index].Write((float) joint_states[i]);
                    }
                    mimic_index = JointNames.IndexOf("left_inner_knuckle_joint");
                    if (mimic_index != -1)
                    {
                        JointStateWriters[mimic_index].Write((float) (-1 * joint_states[i]));
                    }
                    mimic_index = JointNames.IndexOf("right_inner_knuckle_joint");
                    if (mimic_index != -1)
                    {
                        JointStateWriters[mimic_index].Write((float) (-1 * joint_states[i]));
                    }
                }
            }
        }

        void showJointConfig2(string[] joint_names, double[] joint_states, bool should_grip)
        {
            int index;
            int mimic_index;
            for (int i = 0; i < joint_names.Length; i++)
            {
                index = JointNames.IndexOf(joint_names[i]);
                if (index != -1)
                {
                    JointStateWriters[index].Write((float) joint_states[i]);
                }
            }

            index = JointNames.IndexOf("finger_joint");
            if (should_grip)
            {
                JointStateWriters[index].Write((float) 0.5);
                mimic_index = JointNames.IndexOf("right_outer_knuckle_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) (-1 * 0.5));
                }
                mimic_index = JointNames.IndexOf("left_inner_finger_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) 0.5);
                }
                mimic_index = JointNames.IndexOf("right_inner_finger_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) 0.5);
                }
                mimic_index = JointNames.IndexOf("left_inner_knuckle_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) (-1 * 0.5));
                }
                mimic_index = JointNames.IndexOf("right_inner_knuckle_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) (-1 * 0.5));
                }
            }
            else
            {
                JointStateWriters[index].Write((float) 0.1);
                mimic_index = JointNames.IndexOf("right_outer_knuckle_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) (-1 * 0.1));
                }
                mimic_index = JointNames.IndexOf("left_inner_finger_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) 0.1);
                }
                mimic_index = JointNames.IndexOf("right_inner_finger_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) 0.1);
                }
                mimic_index = JointNames.IndexOf("left_inner_knuckle_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) (-1 * 0.1));
                }
                mimic_index = JointNames.IndexOf("right_inner_knuckle_joint");
                if (mimic_index != -1)
                {
                    JointStateWriters[mimic_index].Write((float) (-1 * 0.1));
                }
            }
        }

        /*** GUI functions ***/
        // Adds and monitors the status of various GUI buttons on the Demoshop interface
        void OnGUI()
        {
            String fb = client.PrintFeedback();
            if (fb.Contains("Unable") || fb.Contains("Failed") || fb.Contains("reexecute"))
            {
                tr.Clear();
                tr.enabled = false;
                trailSeen = false;
                GameObject.Find("Feedback").GetComponent<Text>().color = Color.red;
                isRobotRunning = false;
            }
            else if (fb.Contains("At") || fb.Contains("Closed") || fb.Contains("Opened") || fb.Contains("Planned") || fb.Contains("Moved") || fb.Contains("Successfully") || fb.Contains("Cleared"))
            {
                GameObject.Find("Feedback").GetComponent<Text>().color = Color.green;
            }
            else if (fb.Contains("Replanning"))
            {
                GameObject.Find("Feedback").GetComponent<Text>().color = Color.yellow;
            }
            else
            {
                GameObject.Find("Feedback").GetComponent<Text>().color = Color.black;
            }

            if (fb.Contains("execution") || fb.Contains("Closed") || fb.Contains("Opened") || fb.Contains("At"))
            {
                isRobotRunning = false;
            }

            GameObject.Find("Feedback").GetComponent<Text>().text = "Robot's status: " + fb;
            List<GameObject> selectedObjects = Camera.main.GetComponent<Click>().selectedObjects;

            if (hasStarted == false)    // User has not started the demo yet
            {
                if (GUI.Button(new Rect(15, 35, 100, 100), "Start\nDemo", buttonStyle))
                {
                    StartRecording();
                    hasStarted = true;
                    clickedButton = true;
                }
            }  
            else    // User has started the demo
            {
                if (GUI.Button(new Rect(15, 35, 100, 100), "Stop\nDemo", stopStyle))
                {
                    CancelInvoke();
                    hasStarted = false;
                    clickedButton = true;
                }
            }
            if (showExecute == false || generatedWaypoints.Count == 0)
            {
                GUI.enabled = false;
            }
            if (GUI.Button(new Rect(15, 140, 100, 100), "Preview and Run Demo", buttonStyle)) 
            {
                bool invisible = (toggle.isOn == false);
                if (invisible)
                {
                    toggle.isOn = true;
                    GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                }
                toggle.interactable = false;

                float position = jointStates[finger_index];
                if (position >= 0.5f && (gripper_positions[0] != 0.5f)) // Gripper is closed
                {
                    closeWarning = (GameObject) Instantiate(closeWarningBox, closeWarningBox.transform.position, closeWarningBox.transform.rotation);
                    closeWarning.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
                    Button grip_open = GameObject.Find("openGripperButton").GetComponent<Button>();
                    grip_open.onClick.AddListener(openGripperForUser);
                    Button grip_close = GameObject.Find("closeGripperButton").GetComponent<Button>();
                    grip_close.onClick.AddListener(cancelCloseWarning);
                    Button close_warn_close = GameObject.Find("closeCloseWarningButton").GetComponent<Button>();
                    close_warn_close.onClick.AddListener(cancelCloseWarning);
                }
                else
                {
                    tr.gameObject.transform.SetParent(gripper.transform, false);
                    tr.Clear();
                    tr.enabled = true;
                    trailSeen = true;
                    CancelInvoke();
                    hasStarted = false;
                    SendCartesianPathToROS();
                }
                clickedButton = true;
            }
            if (showExecute == false || generatedWaypoints.Count == 0)
            {
                GUI.enabled = true;
            }
            if (generatedWaypoints.Count == 0)
            {
                GUI.enabled = false;
            }
            if (GUI.Button(new Rect(15, 250, 100, 100), "Clear\nDemo", buttonStyle))
            {
                tr.Clear();
                trailSeen = false;
                CancelInvoke();
                hasStarted = false;
                ClearWaypoints();
                ClearPathInROS();
                clickedButton = true;
                ClearPathInROS();
            }
            if (generatedWaypoints.Count == 0)
            {
                GUI.enabled = true;
            }
            if (GUI.Button(new Rect(15, 360, 100, 100), "Add a Waypoint", buttonStyle))
            {  
                GetUserIndex();
            }

            if (trailSeen == false)
            {
                if (GUI.Button(new Rect(15, 470, 100, 100), "View\nTrail", yellowButtonStyle))
                {
                    if (toggle.isOn == false)
                    {
                        GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                        toggle.isOn = true;
                    }
                    tr.gameObject.transform.SetParent(gripper.transform, false);
                    //tr.Clear();
                    tr.enabled = true;
                    trailSeen = true;
                    clickedButton = true;
                }
                GUI.enabled = false;
                if (GUI.Button(new Rect(15, 580, 100, 100), "Delete\nTrail", yellowButtonStyle))
                {
                }
                GUI.enabled = true;
            }
            else
            {
                if (GUI.Button(new Rect(15, 470, 100, 100), "Hide\nTrail", yellowButtonStyle))
                {
                    tr.enabled = false;
                    trailSeen = false;
                    clickedButton = true;
                }
                if (GUI.Button(new Rect(15, 580, 100, 100), "Delete\nTrail", yellowButtonStyle))
                {
                    tr.Clear();
                    tr.enabled = false;
                    trailSeen = false;
                    clickedButton = true;
                }
            }
            if (isMoving)
            {
                if (GUI.Button(new Rect(15, 690, 100, 100), "Select\nTool", yellowButtonStyle))
                {
                    isMoving = false;   // Select mode
                    clickedButton = true;
                }
            }
            else
            {
                GUI.enabled = false;
                if (GUI.Button(new Rect(15, 690, 100, 100), "Select\nTool", yellowButtonStyle))
                {
                }
                GUI.enabled = true;

            }
            if (inPreview == false && inSnapPreview == false)
            {
                bool invisible = (toggle.isOn == false);
                float position = jointStates[finger_index];
                if (position < 0.2f)
                {
                    if (GUI.Button(new Rect(15, 800, 100, 100), "Close\nGripper!", redButtonStyle))
                    {
                        if (stopAutoGrip == false)
                        {
                            if (invisible)
                            {
                                toggle.interactable = false;
                                toggle.isOn = true;
                                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                            }
                            int closest_wpt = GetClosestWaypoint(gripper.transform.position);
                            if (closest_wpt != -1)
                            {
                                if ((generatedWaypoints[closest_wpt].GetComponent<WaypointScript>().userSpecified == true) && (Vector3.Distance(gripper.transform.position, generatedWaypoints[closest_wpt].GetComponent<WaypointScript>().GetPosition()) < 0.01) && (addedUserWpt == true))
                                {
                                    WaypointScript script = generatedWaypoints[closest_wpt].GetComponent<WaypointScript>();
                                    grip_msg.data = script.GetIndex();
                                    rosSocket.Publish(grip_publication_id, grip_msg);
                                    script.SetGripperPosition(0.5f);
                                    close_msg.data = 0.5f;
                                    gripper_positions[grip_msg.data] = 0.5f;
                                    rosSocket.Publish(close_publication_id, close_msg);
                                    addedUserWpt = false;
                                }
                            }
                            if (invisible)
                            {
                                toggle.interactable = true;
                                toggle.isOn = false;
                                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
                            }
                        }
                        stopAutoGrip = false;
                        client.SetRequest(CLOSE);
                        client.SendGoal();
                        clickedButton = true;
                        status_bg.color = Color.red;
                        status.color = Color.black;
                        status.text = "Real robot running";
                        isRobotRunning = true;
                    }
                }
                else if (position >= 0.2f)
                {
                    if (GUI.Button(new Rect(15, 800, 100, 100), "Open\nGripper!", redButtonStyle))
                    {
                        
                        if (stopAutoGrip == false)
                        {
                            bool invisible2 = (toggle.isOn == false);
                            if (invisible2)
                            {
                                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                                toggle.interactable = false;
                                toggle.isOn = true;
                            }
                            int closest_wpt = GetClosestWaypoint(gripper.transform.position);
                            if (closest_wpt != -1)
                            {
                                if ((generatedWaypoints[closest_wpt].GetComponent<WaypointScript>().userSpecified == true) && (Vector3.Distance(gripper.transform.position, generatedWaypoints[closest_wpt].GetComponent<WaypointScript>().GetPosition()) < 0.01) && (addedUserWpt == true))
                                {
                                    WaypointScript script = generatedWaypoints[closest_wpt].GetComponent<WaypointScript>();
                                    grip_msg.data = script.GetIndex();
                                    rosSocket.Publish(grip_publication_id, grip_msg);
                                    script.SetGripperPosition(0.1f);
                                    close_msg.data = 0.1f;
                                    gripper_positions[grip_msg.data] = 0.1f;
                                    rosSocket.Publish(close_publication_id, close_msg);
                                    addedUserWpt = false;
                                }
                            }
                            if (invisible2)
                            {
                                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
                                toggle.interactable = true;
                                toggle.isOn = false;
                            }
                         }
                        stopAutoGrip = false;
                        client.SetRequest(OPEN);
                        client.SendGoal();
                        clickedButton = true;
                        status_bg.color = Color.red;
                        status.color = Color.black;
                        status.text = "Real robot running";
                        isRobotRunning = true;
                    }
                } 
            }
            else
            {
                GUI.enabled = false;
                float position = jointStates[finger_index];
                if (position < 0.35f)
                {
                    if (GUI.Button(new Rect(15, 795, 100, 100), "Close\nGripper", buttonStyle))
                    {
                    }
                }
                else if (position >= 0.35f)
                {
                    if (GUI.Button(new Rect(15, 795, 100, 100), "Open\nGripper", buttonStyle))
                    {
                    }
                }
                GUI.enabled = true;
            }
            if (displayMenu == true && numSelected > 0) // Only display the menu if the user clicked the right mouse button and has selected waypoints
            {
                CancelInvoke();
                hasStarted = false;
                if (numSelected == 1)
                {
                    WaypointScript waypoint_script = selectedObjects[0].GetComponent<WaypointScript>();
                    float current_gripper_pos = waypoint_script.GetGripperPosition();
                    GUILayout.BeginArea(new Rect(menu_position.x, menu_position.y, 152, 275), GUI.skin.box);
                    GUILayout.Label("Waypoint Menu", menuFont);
                    if (GUILayout.Button("Inspect\nWaypoint!", inspectStyle))   // Have the robot navigate to the selected waypoint
                    {
                        clickedButton = true;
                        InspectWaypoint();
                        displayMenu = false;
                    }
                    if (current_gripper_pos == 0.1f)    // Gripper is open
                    {
                        if (GUILayout.Button("Close gripper", wptMenuButtonStyle))  
                        {
                            clickedButton = true;
                            grip_msg.data = waypoint_script.GetIndex();
                            rosSocket.Publish(grip_publication_id, grip_msg);
                            waypoint_script.SetGripperPosition(0.5f);
                            gripper_positions[grip_msg.data] = 0.5f;
                            close_msg.data = 0.5f;
                            rosSocket.Publish(close_publication_id, close_msg);
                            displayMenu = false;
                            stopAutoGrip = true;
                        }
                        if (GUILayout.Button("Clear gripper", wptMenuButtonStyle))  
                        {
                            clickedButton = true;
                            grip_msg.data = waypoint_script.GetIndex();
                            rosSocket.Publish(grip_publication_id, grip_msg);
                            waypoint_script.SetGripperPosition(0.0f);
                            gripper_positions[grip_msg.data] = 0.0f;
                            close_msg.data = 0.0f;
                            rosSocket.Publish(close_publication_id, close_msg);
                            displayMenu = false;
                            stopAutoGrip = true;
                        }
                    }
                    else if (waypoint_script.GetGripperPosition() == 0.5f) // Gripper is closed
                    {
                        if (GUILayout.Button("Open gripper", wptMenuButtonStyle))   
                        {
                            clickedButton = true;
                            grip_msg.data = waypoint_script.GetIndex();
                            rosSocket.Publish(grip_publication_id, grip_msg);
                            waypoint_script.SetGripperPosition(0.1f);
                            close_msg.data = 0.1f;
                            gripper_positions[grip_msg.data] = 0.1f;
                            rosSocket.Publish(close_publication_id, close_msg);
                            displayMenu = false;
                            stopAutoGrip = true;
                        }
                        if (GUILayout.Button("Clear gripper", wptMenuButtonStyle))  
                        {
                            clickedButton = true;
                            grip_msg.data = waypoint_script.GetIndex();
                            rosSocket.Publish(grip_publication_id, grip_msg);
                            waypoint_script.SetGripperPosition(0.0f);
                            close_msg.data = 0.0f;
                            gripper_positions[grip_msg.data] = 0.0f;
                            rosSocket.Publish(close_publication_id, close_msg);
                            displayMenu = false;
                            stopAutoGrip = true;
                        }
                    }
                    else
                    {
                        if (GUILayout.Button("Close gripper", wptMenuButtonStyle))  
                        {
                            clickedButton = true;
                            grip_msg.data = waypoint_script.GetIndex();
                            rosSocket.Publish(grip_publication_id, grip_msg);
                            waypoint_script.SetGripperPosition(0.5f);
                            close_msg.data = 0.5f;
                            gripper_positions[grip_msg.data] = 0.5f;
                            rosSocket.Publish(close_publication_id, close_msg);
                            displayMenu = false;
                            stopAutoGrip = true;
                        }
                        if (GUILayout.Button("Open gripper", wptMenuButtonStyle))   
                        {
                            clickedButton = true;
                            grip_msg.data = waypoint_script.GetIndex();
                            rosSocket.Publish(grip_publication_id, grip_msg);
                            waypoint_script.SetGripperPosition(0.1f);
                            close_msg.data = 0.1f;
                            gripper_positions[grip_msg.data] = 0.1f;
                            rosSocket.Publish(close_publication_id, close_msg);
                            displayMenu = false;
                            stopAutoGrip = true;
                        }
                    }
                    if (GUILayout.Button("Remove waypoint", wptMenuButtonStyle))    
                    {
                        clickedButton = true;
                        DeleteWaypoints();
                        displayMenu = false;
                        tr.enabled = false;
                        trailSeen = false;
                    }
                    if (GUILayout.Button("Close menu", wptMenuButtonStyle))    
                    {
                        clickedButton = true;
                        displayMenu = false;
                    }
                }                
                else
                {   
                    GUILayout.BeginArea(new Rect(menu_position.x, menu_position.y, 152, 130), GUI.skin.box);
                    GUILayout.Label("Waypoints Menu", menuFont);
                    if (GUILayout.Button("Remove waypoints", wptMenuButtonStyle))
                    {
                        clickedButton = true;
                        DeleteWaypoints();
                        displayMenu = false;
                        tr.enabled = false;
                        trailSeen = false;
                    }
                    if (GUILayout.Button("Close menu", wptMenuButtonStyle))    
                    {
                        clickedButton = true;
                        displayMenu = false;
                    }
                }

                GUILayout.EndArea();
            }
        }

        /*** Action Client functions ***/
        // SendCartesianPathToROS: ExecutePath action client requests its respective action server in the MoveIt! backend to move the robot according to the specified waypoints
        void SendCartesianPathToROS()
        {
            WaypointScript waypoint_script;
            tr.gameObject.transform.parent = GameObject.Find("fts_toolside").transform;
            tr.Clear();
            client.SetRequest(PREVIEW);
            client.SendGoal();
            tr.enabled = true;
            trailSeen = true;
        }

        private void executeOnRobot()
        {
            Destroy(confirmation);
            inPreview = false;
            tr.gameObject.transform.SetParent(null, false);
            client.SetRequest(EXECUTE);
            client.SendGoal();
            status_bg.color = Color.red;
            status.color = Color.black;
            status.text = "Real robot running";
            isRobotRunning = true;
        }

        // ClearWaypoints: Similar to ClearWaypoints, but clears the vector of waypoints in the MoveIt! backend
        void ClearPathInROS()
        {
            client.SetRequest(CLEAR);
            client.SendGoal();
        }

        void InspectWaypoint()
        {
            status_bg.color = Color.red;
            status.color = Color.black;
            status.text = "Real robot running";
            isRobotRunning = true;
            List<GameObject> selectedObjects = Camera.main.GetComponent<Click>().selectedObjects;
            insp_msg.data = selectedObjects[0].GetComponent<WaypointScript>().GetIndex();
            rosSocket.Publish(insp_publication_id, insp_msg);
            client.SetRequest(INSPECT);
            client.SendGoal();
        }

        /*** Waypoint helper functions ***/
        // Begins adding waypoints to the scene and on the MoveIt! backend
        void StartRecording()
        {
            InvokeRepeating("AddWaypoint", 0.0f, rate);
        }

        // Adds a waypoint specified by the user to the demonstration
        void AddUserWaypoint()
        {
            bool invisible = (toggle.isOn == false);
            if (invisible)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
            }
            addedUserWpt = true;
            Vector3 gripper_position;
            Quaternion gripper_rotation;
            gripper_position = gripper.transform.position;
            gripper_rotation = gripper.transform.rotation;

            // Tell the user if the waypoint is invalid
            if (generatedWaypoints.Count > 0)
            {
                int closest_waypoint = GetClosestWaypoint(gripper_position);
                Vector3 closest_position = generatedWaypoints[closest_waypoint].GetComponent<WaypointScript>().GetPosition();
                float dist = Vector3.Distance(gripper_position, closest_position);
                if (dist < min_distance)
                {
                    if (closest_waypoint == requested_index || closest_waypoint == (requested_index - 1) || closest_waypoint == (requested_index + 1))
                    {
                        if (dist < 0.05 && generatedWaypoints[closest_waypoint].GetComponent<WaypointScript>().userSpecified == false && closest_waypoint == requested_index)   // Replace automatically generated waypoint with user-specified one
                        {
                            Destroy(generatedWaypoints[closest_waypoint]);
                            // Tell ROS that we want to delete these waypoints
                            int ind = generatedWaypoints[closest_waypoint].GetComponent<WaypointScript>().GetIndex();
                            del_msg.data = ind;
                            rosSocket.Publish(del_publication_id, del_msg);
                            // Remove the selected waypoints from waypoint list
                            generatedWaypoints.RemoveAt(ind);
                            gripper_positions.RemoveAt(ind);
                            // Update indices of waypoints
                            UpdateWaypoints();
                        }
                    }
                }
            }
            
            // The user-specified waypoint is valid
            if (requested_index == generatedWaypoints.Count)
            {
                last_position = gripper_position;
            }

            // Display the waypoint in the scene
            var wpt = (GameObject) Instantiate(waypoint, gripper_position, gripper_rotation);
            WaypointScript waypoint_script = wpt.GetComponent<WaypointScript>();
            // User-specified waypoint looks different than automatically generated ones
            wpt.GetComponent<RectTransform>().localScale += new Vector3(0.01f, 0.01f, 0.01f);
            if (waypoint_script.currentlySelected == false)
            {
                wpt.GetComponent<Renderer>().material.SetColor("_Color", specified);
            }

            // Add waypoint to list
            waypoint_script.userSpecified = true;
            generatedWaypoints.Insert(requested_index, wpt);
            gripper_positions.Insert(requested_index, 0.0f);
            waypoint_script.SetPosition(gripper_position);  // Store the gripper position at which the waypoint was added
            waypoint_script.SetJointConfiguration((float)jointStates[0], (float)jointStates[1], (float)jointStates[2], (float)jointStates[3], (float)jointStates[4], (float)jointStates[5], (float)jointStates[6]);

            // Tell ROS to add the waypoint to its vector of waypoints
            add_msg.data = requested_index;
            rosSocket.Publish(add_publication_id, add_msg);

            // Reset index
            requested_index = -1;

            if (invisible)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
            }
        }

        // Asks user to submit the time index at which they want to add a waypoint
        void GetUserIndex()
        {
            bool invisible = (toggle.isOn == false);
            if (invisible)
            {
                toggle.isOn = true;
                toggle.interactable = false;
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
            }
            // Get the time at which the user wants to add their waypoint
            inputIndex = (GameObject) Instantiate(inputIndexBox, inputIndexBox.transform.position, inputIndexBox.transform.rotation);
            inputIndex.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
            if (generatedWaypoints.Count == 0)
            {
                GameObject.Find("NumberInput").GetComponent<InputField>().text = 0.ToString();
            }
            else
            {
                GameObject.Find("NumberInput").GetComponent<InputField>().text = (GetClosestWaypoint(gripper.transform.position) + 1).ToString();
            }
            Button close = GameObject.Find("Close").GetComponent<Button>();
            close.onClick.AddListener(CloseInputIndexWindow);
            Button submit = GameObject.Find("Submit").GetComponent<Button>();
            submit.onClick.AddListener(SubmitNumber);
            if (invisible)
            {
                toggle.isOn = false;
                toggle.interactable = true;
                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
            }
        }

        // Deletes all the waypoints that are currently selected by the user
        void DeleteWaypoints()
        {
            List<GameObject> selectedObjects = Camera.main.GetComponent<Click>().selectedObjects;

            int ind;
            for (int i = 0; i < selectedObjects.Count; i++)
            {
                // Remove the selected waypoints from the scene
                Destroy(selectedObjects[i]);
                // Tell ROS that we want to delete these waypoints
                ind = selectedObjects[i].GetComponent<WaypointScript>().GetIndex();
                del_msg.data = ind;
                rosSocket.Publish(del_publication_id, del_msg);
                // Remove the selected waypoints from waypoint list
                generatedWaypoints.RemoveAt(ind);
                gripper_positions.RemoveAt(ind);
                selectedObjects[i] = null;
                // Update indices of waypoints
                UpdateWaypoints();
            }
        }

        // Clears all waypoints from the scene 
        void ClearWaypoints()
        {
            GUI.enabled = false;
            foreach (var waypoint in generatedWaypoints)
            {
                Destroy(waypoint);
            }
            generatedWaypoints = new List<GameObject>();
            gripper_positions = new List<float>();
            GUI.enabled = true;
        }

        // Updates each waypoint's index, label, and color, and keeps track of the number of selected waypoints at each iteration 
        void UpdateWaypoints()
        {
            numSelected = 0;
            if (generatedWaypoints.Count > 0)
            {
                if (generatedWaypoints[0].GetComponent<WaypointScript>().GetJointConfiguration()[finger_index] >= 0.35)
                {
                    lastClosed = true;
                    lastOpen = false;
                }
                else
                {
                    lastClosed = false;
                    lastOpen = true;
                }
            }
            for (int i = 0; i < generatedWaypoints.Count; i++)
            {
                WaypointScript waypointScript = generatedWaypoints[i].GetComponent<WaypointScript>();

                // Update waypoint indices
                TextMesh tm = generatedWaypoints[i].GetComponentInChildren(typeof(TextMesh)) as TextMesh;
                waypointScript.SetIndex(i);
                float gripper_pos = gripper_positions[i];
                if (waypointScript.GetGripperPosition() != gripper_pos)
                {
                	waypointScript.SetGripperPosition(gripper_pos);
                }

                if (waypointScript.hovering == false)
                {
                    tm.text = i.ToString();
                }
                else
                {
                    if (gripper_pos == 0.5f)
                    {
                        tm.text = "Gripper close";
                    }
                    else if (gripper_pos == 0.1f)
                    {
                        tm.text = "Gripper open";
                    }
                    else
                    {
                        tm.text = i.ToString();
                    }
                }

                // Recolor waypoints
                float gradValue = (i + 1)/(float)generatedWaypoints.Count;
                if (waypointScript.currentlySelected == false)
                {
                    if (waypointScript.userSpecified)
                    {
                        generatedWaypoints[i].GetComponent<Renderer>().material.SetColor("_Color", specified);
                    }
                    else if (waypointScript.GetSnapStatus() == true)
                    {
                        generatedWaypoints[i].GetComponent<Renderer>().material.SetColor("_Color", snapPreviewed);
                    }
                    else
                    {
                        generatedWaypoints[i].GetComponent<Renderer>().material.SetColor("_Color", gradient.Evaluate(gradValue));
                    }
                }
                else
                {
                    numSelected++;
                    generatedWaypoints[i].GetComponent<Renderer>().material.SetColor("_Color", selected);
                }   

                if (stopAutoGrip == false && (snapStarted == false))
                {
                    if (waypointScript.GetJointConfiguration()[finger_index] >= 0.35f && (lastOpen == true))
                    {
                        grip_msg.data = waypointScript.GetIndex();
                        rosSocket.Publish(grip_publication_id, grip_msg);
                        waypointScript.SetGripperPosition(0.5f);
                        gripper_positions[grip_msg.data] = 0.5f;
                        close_msg.data = 0.5f;
                        rosSocket.Publish(close_publication_id, close_msg);
                        lastClosed = true;
                        lastOpen = false;
                    }
                    else if ((waypointScript.GetJointConfiguration()[finger_index] < 0.35f) && (lastClosed == true))
                    {
                        grip_msg.data = waypointScript.GetIndex();
                        rosSocket.Publish(grip_publication_id, grip_msg);
                        waypointScript.SetGripperPosition(0.1f);
                        gripper_positions[grip_msg.data] = 0.1f;
                        close_msg.data = 0.1f;
                        rosSocket.Publish(close_publication_id, close_msg);   
                        lastClosed = false;
                        lastOpen = true;
                    }

                }
            }
        }

        // Returns the index of the waypoint in the waypoints list that is closest to the specified position
        private int GetClosestWaypoint(Vector3 pos)
        {
            float dist;
            int minIndex = -1;
            float minDistance = Mathf.Infinity;
            for (int i = 0; i < generatedWaypoints.Count; i++)
            {
                dist = Vector3.Distance(pos, generatedWaypoints[i].GetComponent<WaypointScript>().GetPosition());
                if (dist < minDistance)
                {
                    minDistance = dist;
                    minIndex = i;
                }
            }

            return minIndex;
        }

        /*** Transform helper functions ***/
        // Converts a position of Unity's Vector3 type into ROS's geometry_msgs/Point type
        private Messages.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            Messages.Geometry.Point geometryPoint = new Messages.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        // Converts a quaternion of the Unity's Quaternion type into the ROS's geometry_msgs/Quaternion type
        private Messages.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            Messages.Geometry.Quaternion geometryQuaternion = new Messages.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }

        /*** Debugging Functions ***/
        // Prints a position of the Vector3 type for debugging
        private void PrintPosition(Vector3 position)
        {
            Debug.Log("(" + position.x + ", " + position.y + ", " + position.z + ")");
        }

        // Prints a point of the geometry_msgs/Point type for debugging
        private void PrintGeometryPoint(Messages.Geometry.Point point)
        {
            Debug.Log("(" + point.x + ", " + point.y + ", " + point.z + ")");
        }

        // Prints a quaternion of the Quaternion type for debugging
        private void PrintQuaternion(Quaternion quaternion)
        {
            Debug.Log("(" + quaternion.x + ", " + quaternion.y + ", " + quaternion.z + ", " + quaternion.w + ")");
        }

        // Prints a quaternion of the geometry_msgs/Quaternion type for debugging
        private void PrintGeometryQuaternion(Messages.Geometry.Quaternion quaternion)
        {
            Debug.Log("(" + quaternion.x + ", " + quaternion.y + ", " + quaternion.z + ", " + quaternion.w + ")");
        }

        private void PrintJointStates(string[] joint_names, double[] joint_states)
        {
            for (int i = 0; i < joint_states.Length; i++)
            {
                if (joint_names != null)
                {
                    Debug.Log(joint_names[i]);
                }
                Debug.Log(joint_states[i]);
            }
        }
    

        void CloseInputIndexWindow()
        {
            Destroy(inputIndex);
            inputIndex = null;
        }

        void SubmitNumber()
        {
             bool result = int.TryParse(GameObject.Find("NumberInput").GetComponent<InputField>().text, out int i);
             if (result == true)
             {
                if (i >= 0 && i <= generatedWaypoints.Count)
                {
                    requested_index = i;
                    Destroy(inputIndex);
                    inputIndex = null;
                }
             }
        }

        void CloseConfirmObject()
        {
        	responded_to_help = true;
        	Destroy(confirmObject);
        	confirmObject = null;
            snapStarted = false;
            //stopAutoGrip = false;
        }

        /*** Trajectory helper functions ***/
        double[] InterpolateCubic(trajectory_msgs.JointTrajectoryPoint p0, trajectory_msgs.JointTrajectoryPoint p1, float t_abs)
        {
            double T = p1.time_from_start.secs - p0.time_from_start.secs;
            double t = t_abs - p0.time_from_start.secs;
            double[] q = {0f, 0f, 0f, 0f, 0f, 0f};
            double a, b, c, d;

            for (int i = 0; i < p0.positions.Length; i++)
            {
                a = p0.positions[i];
                b = p0.velocities[i];
                c = (-3*p0.positions[i] + 3*p1.positions[i] - 2*T*p0.velocities[i] - T*p1.velocities[i]) / (T * T);
                d = (2*p0.positions[i] - 2*p1.positions[i] + T*p0.velocities[i] + T*p1.velocities[i]) / (T * T * T);

                q[i] = a + b*t + c*t*t + d*t*t*t;
            }

            return q;
        }

        void ExecuteTrajectoryOnUnity (trajectory_msgs.JointTrajectory joint_trajectory, float rate, int curr_wpt)
        {
            Vector3 gripper_position;
            Quaternion gripper_rotation;
            float t = 0;
            bool finishedTrajectory = false;
            while (finishedTrajectory == false)
            {
                // First point
                if (t == 0)
                {
                    int index;
                    int mimic_index;
                    for (int i = 0; i < joint_trajectory.points[0].positions.Length; i++)
                    {
                        index = JointNames.IndexOf(joint_trajectory.joint_names[i]);
                        if (index != -1)
                        {
                            JointStateWriters[index].Write((float) joint_trajectory.points[0].positions[i]);
                        }
                    }
                }
                // Last point
                else if (t >= joint_trajectory.points[joint_trajectory.points.Length - 1].time_from_start.secs)
                {
                    int index;
                    int mimic_index;
                    for (int i = 0; i < joint_trajectory.points[joint_trajectory.points.Length - 1].positions.Length; i++)
                    {
                        index = JointNames.IndexOf(joint_trajectory.joint_names[i]);
                        if (index != -1)
                        {
                            JointStateWriters[index].Write((float) joint_trajectory.points[joint_trajectory.points.Length - 1].positions[i]);
                        }
                    }
                    if (gripper_positions[curr_wpt] > 0.0f)
                    {
                        gripper_pos = gripper_positions[curr_wpt];
                        gripperSet = true;
                        index = JointNames.IndexOf("finger_joint");
                        if (index != -1)
                        {
                            JointStateWriters[index].Write((float) gripper_pos);
                            mimic_index = JointNames.IndexOf("right_outer_knuckle_joint");
                            if (mimic_index != -1)
                            {
                                JointStateWriters[mimic_index].Write((float) (-1 *  gripper_pos));
                            }
                            mimic_index = JointNames.IndexOf("left_inner_finger_joint");
                            if (mimic_index != -1)
                            {
                                JointStateWriters[mimic_index].Write((float)  gripper_pos);
                            }
                            mimic_index = JointNames.IndexOf("right_inner_finger_joint");
                            if (mimic_index != -1)
                            {
                                JointStateWriters[mimic_index].Write((float)  gripper_pos);
                            }
                            mimic_index = JointNames.IndexOf("left_inner_knuckle_joint");
                            if (mimic_index != -1)
                            {
                                JointStateWriters[mimic_index].Write((float) (-1 *  gripper_pos));
                            }
                            mimic_index = JointNames.IndexOf("right_inner_knuckle_joint");
                            if (mimic_index != -1)
                            {
                                JointStateWriters[mimic_index].Write((float) (-1 *  gripper_pos));
                            }
                        }
                    }
                    finishedTrajectory = true;
                }
                // Intermediary points
                else
                {
                    int j = 0;
                    while (joint_trajectory.points[j+1].time_from_start.secs < t)
                    {
                        j += 1;
                    }
                    double[] config = InterpolateCubic(joint_trajectory.points[j], joint_trajectory.points[j+1], t);
                    int index;
                    int mimic_index;
                    for (int i = 0; i < config.Length; i++)
                    {
                        index = JointNames.IndexOf(joint_trajectory.joint_names[i]);
                        if (index != -1)
                        {
                            JointStateWriters[index].Write((float) config[i]);
                        }
                    }
                }

                t += rate;
                Thread.Sleep((int)(rate * 1000));
            }
        }

        private void TrajsPlanHandler (WaypointTrajectories message)
        {
            gripperSet = false;
            gripper_pos = 0.0f;
            inPreview = true;
            showExecute = false;
            int i = 0;

            foreach (moveit_msgs.RobotTrajectory traj in message.trajectories)
            {
                ExecuteTrajectoryOnUnity(traj.joint_trajectory, TRAJ_RATE, i);
                i++;
            }
  
            showConfirmation = true;
            showExecute = true;
        }

        private void cancelExecution()
        {
            Destroy(confirmation);
            inPreview = false;
            tr.enabled = false;
            trailSeen = false;
            tr.Clear();
            trailSeen = false;
        }

        private void openGripperForUser()
        {
            clickedButton = true;
            Destroy(closeWarning);
            client.SetRequest(OPEN);
            client.SendGoal();
            status_bg.color = Color.red;
            status.color = Color.black;
            status.text = "Real robot running";
            isRobotRunning = true;
            if (toggle.isOn == false)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                toggle.isOn = true;
            }
            tr.gameObject.transform.SetParent(gripper.transform, false);
            tr.Clear();
            tr.enabled = true;
            trailSeen = true;
            CancelInvoke();
            hasStarted = false;
            SendCartesianPathToROS();
        }

        private void cancelCloseWarning()
        {
            Destroy(closeWarning);
            closeWarning = null;
            if (toggle.isOn == false)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                toggle.isOn = true;
            }
            tr.gameObject.transform.SetParent(gripper.transform, false);
            tr.Clear();
            tr.enabled = true;
            trailSeen = true;
            CancelInvoke();
            hasStarted = false;
            SendCartesianPathToROS();
        }

        /*** ROS-Unity conversion helper functions ***/
        private Vector3 RosVector3ToUnityVector3(geometry_msgs.Vector3 vec)
        {
            return new Vector3(vec.x, vec.y, vec.z).Ros2Unity();
        }

        private Quaternion RosQuaternionToUnityQuaternion(geometry_msgs.Quaternion quat)
        {
            return new Quaternion(quat.x, quat.y, quat.z, quat.w).Ros2Unity();
        }

        /*** Tracked object helper functions ***/
        private void TransformHandler(geometry_msgs.TransformStamped message)
        {
            if (tracking == false)
            {
                child_frame_name = message.child_frame_id;
                tracked_translation = RosVector3ToUnityVector3(message.transform.translation);
                tracked_rotation = RosQuaternionToUnityQuaternion(message.transform.rotation);
                tracked_secs = currTime;
                tracking = true;
            }
        }

        private string GetClosestTaskObject(Vector3 pos)
        {
            float dist;
            string minString = null;
            float minDistance = Mathf.Infinity;
            Vector3 obj_position;
            foreach (KeyValuePair<string, GameObject> entry in tracked_object_dict)
            {
                obj_position = entry.Value.transform.position;
                dist = Vector3.Distance(pos, obj_position);
                if (dist < minDistance)
                {
                    minDistance = dist;
                    minString = entry.Key;
                }
            }

            return minString;
        }

        private void HighlightOnlyObject()
        {
            snapStarted = true;
            clickedButton = true;
            responded_to_help = true;
            closest_task_object = child_frame_name;
            tracked_object_dict[closest_task_object].GetComponent<Renderer>().material.SetColor("_Color", highlighted);  // Highlight the object in the scene
            tracked_object_dict[closest_task_object].GetComponent<TrackedObjectScript>().currentlyHighlighted = true;
            confirmObject = (GameObject) Instantiate(confirmObjectBox, confirmObjectBox.transform.position, confirmObjectBox.transform.rotation);  // Get confirmation from the user on whether the highlighted box is the one that they actually want to grip
            confirmObject.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
            Button no = GameObject.Find("NoObject").GetComponent<Button>();
            no.onClick.AddListener(CloseConfirmWindow);
            Button close = GameObject.Find("CloseButton").GetComponent<Button>();
            close.onClick.AddListener(CloseConfirmWindow);
            Button yes = GameObject.Find("YesObject").GetComponent<Button>();
            yes.onClick.AddListener(PickHighlightedTaskObject);
        }

        void CloseConfirmWindow()
        {
            responded_to_help = true;
            Destroy(confirmObject);
            confirmObject = null;
            snapStarted = false;
        }

        private void HighlightTaskObject()
        {
        	snapStarted = true;
            clickedButton = true;
            responded_to_help = true;
            bool invisible = (toggle.isOn == false);
            if (invisible)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(1, 1, 1);
                toggle.isOn = true;
                toggle.interactable = false;
            }
            closest_task_object = GetClosestTaskObject(gripper.transform.position);  // Highlight the object closest to the robot's gripper
            if (invisible)
            {
                GameObject.Find("base_link").transform.localScale = new Vector3(0, 0, 0);
                toggle.isOn = false;
                toggle.interactable = true;
            }
            tracked_object_dict[closest_task_object].GetComponent<Renderer>().material.SetColor("_Color", highlighted);  // Highlight the object in the scene
            tracked_object_dict[closest_task_object].GetComponent<TrackedObjectScript>().currentlyHighlighted = true;
            confirmObject = (GameObject) Instantiate(confirmObjectBox, confirmObjectBox.transform.position, confirmObjectBox.transform.rotation);  // Get confirmation from the user on whether the highlighted box is the one that they actually want to grip
            confirmObject.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
            Button no = GameObject.Find("NoObject").GetComponent<Button>();
            no.onClick.AddListener(CorrectObject);
            Button close = GameObject.Find("CloseButton").GetComponent<Button>();
            close.onClick.AddListener(CloseConfirmObject);
            Button yes = GameObject.Find("YesObject").GetComponent<Button>();
            yes.onClick.AddListener(PickHighlightedTaskObject);
        }

        private void CorrectObject()
        {
            clickedButton = true;
            tracked_object_dict[closest_task_object].GetComponent<Renderer>().material.SetColor("_Color", regular);		// Stop highlighting the originally predicted object
            Destroy(confirmObject);
            confirmObject = null;
            correctObject = (GameObject) Instantiate(correctObjectBox, correctObjectBox.transform.position, correctObjectBox.transform.rotation);
            correctObject.GetComponent<RectTransform>().SetParent(GameObject.Find ("Canvas").GetComponent<RectTransform>(), false);
            Button no = GameObject.Find("NoCorrectedObject").GetComponent<Button>();
            no.onClick.AddListener(CancelHelp);
            Button close = GameObject.Find("CloseCorrectObject").GetComponent<Button>();
            close.onClick.AddListener(CancelHelp);
            Button yes = GameObject.Find("YesCorrectedObject").GetComponent<Button>();
            yes.onClick.AddListener(PickCorrectTaskObject);
        }

        private void CancelHelp()
        {
            clickedButton = true;
            Destroy(correctObject);
            correctObject = null;
            snapStarted = false;
        }

        private void PickCorrectTaskObject()
        {
            clickedButton = true;
            List<GameObject> selectedTaskObjects = Camera.main.GetComponent<Click>().selectedTaskObjects;
            if (selectedTaskObjects.Count == 1)	// The user has selected the object they actually wanted to grasp
            {
                Destroy(correctObject);
                correctObject = null;
                pick_msg.data = selectedTaskObjects[0].GetComponent<TrackedObjectScript>().tracked_object_id;
                rosSocket.Publish(pick_publication_id, pick_msg);
                client.SetRequest(PICK);
                client.SendGoal();
            }
        }

        private void PickHighlightedTaskObject()
        {
            clickedButton = true;
            Destroy(confirmObject);
            confirmObject = null;
            pick_msg.data = closest_task_object;  
            rosSocket.Publish(pick_publication_id, pick_msg);
            client.SetRequest(PICK);
            client.SendGoal();
        }

        private void CloseSnapWindow()
        {
            clickedButton = true;
            Destroy(confirmSnap);
            confirmSnap = null;

            GUI.enabled = false;

            if (firstSnapIdx != -1)
            {
                // Remove waypoints from scene
                for (int j = firstSnapIdx; j < generatedWaypoints.Count; j++)
                {
                    Destroy(generatedWaypoints[j]);
                }

                generatedWaypoints.RemoveRange(firstSnapIdx, (generatedWaypoints.Count - firstSnapIdx));
                gripper_positions.RemoveRange(firstSnapIdx, (generatedWaypoints.Count - firstSnapIdx));
            }
            GUI.enabled = true;

            firstSnapIdx = -1;
            inSnapPreview = false;
            firstTime = true;
        }
  
        bool firstTime = true;
        int firstSnapIdx = -1;
        bool should_grip = false;
        private void SnapWaypointHandler(SnapWaypoint message)
        {
        	if (firstSnapIdx == -1)
        	{
                should_grip = false;
        		firstSnapIdx = generatedWaypoints.Count;
        	}
        	stopAutoGrip = true;
        	snapJointNames.Add(new List<string>(message.names));
        	snapJoints.Add(new List<double>(message.joints));
        	snapTypes.Add(message.wpt_type);
        }

        private void TimeServiceHandler(rosapi.GetTimeResponse message)
        {
            currTime = message.time.secs;
        }

        // Clear any waypoints stored in the MoveIt! backend when quitting Demoshop
        void OnApplicationQuit()
        {
            ClearPathInROS();
            snapJoints.Clear();
            snapJointNames.Clear();
            snapTypes.Clear();
        }
    }
}