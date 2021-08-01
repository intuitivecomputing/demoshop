// WaypointScript.cs
// Based on code from https://connect.unity.com/u/c00pala-j

using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class WaypointScript : MonoBehaviour
{
	[HideInInspector]
	public bool currentlySelected = false;
    [HideInInspector]
    public bool userSpecified = false;
	private int index;
    private Vector3 position;
    private float[] joint_configuration;
    private float gripper_action;
    [HideInInspector]
    public bool hovering;
    [HideInInspector]
    public bool pressed;
    [HideInInspector]
    public bool released;
    private int finger_index;
    private bool isSnap = false;

    // Start is called before the first frame update
    void Start()
    {
        Camera.main.gameObject.GetComponent<Click>().selectableObjects.Add(this.gameObject);
        gripper_action = 0.0f;
        finger_index = Camera.main.GetComponent<RosSharp.RosBridgeClient.RecordDemo>().finger_index;
        pressed = false;
    }

    /*** Getter functions ***/
    public int GetIndex()
    {
    	return index;
    }

    public Vector3 GetPosition()
    {
        return position;
    }

    public float[] GetJointConfiguration()
    {
        return joint_configuration;
    }

    public float GetGripperPosition()
    {
        return gripper_action;
    }

    public bool GetSnapStatus()
    {
        return isSnap;
    }

    /*** Setter functions ***/
    public void SetIndex(int i)
    {
    	index = i;
    }

    public void SetPosition(Vector3 pos)
    {
        position = pos;
    }

    public void SetJointConfiguration(float joint_0, float joint_1, float joint_2, float joint_3, float joint_4, float joint_5, float joint_6)
    {
        joint_configuration = new float[7];
        joint_configuration[0] = joint_0;
        joint_configuration[1] = joint_1;
        joint_configuration[2] = joint_2;
        joint_configuration[3] = joint_3; 
        joint_configuration[4] = joint_4; 
        joint_configuration[5] = joint_5; 
        joint_configuration[6] = joint_6;
    }

    public void SetGripperPosition(float ga)
    {
        gripper_action = ga;
        joint_configuration[Camera.main.GetComponent<RosSharp.RosBridgeClient.RecordDemo>().finger_index] = ga;
    }

    public void SetAsSnap()
    {
        isSnap = true;
    }


    public void ClearSnap()
    {
        isSnap = false;
    }

    /*** Mouse event functions ***/
    void OnMouseDown()
    {
        pressed = true;
        released = false;
    }

    void OnMouseUp()
    {
        pressed = false;
        released = true;
    }

    void OnMouseOver()
    {
        hovering = true;
    }

    void OnMouseExit()
    {
        hovering = false;
    }
}
