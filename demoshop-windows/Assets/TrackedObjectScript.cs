// TrackedObjectScript.cs

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrackedObjectScript : MonoBehaviour
{
	[HideInInspector]
	public bool currentlySelected = false;
	[HideInInspector]
	public bool currentlyHighlighted = false;
	[HideInInspector]
	public string tracked_object_id = "";
	[HideInInspector]
	public uint lastTime;

    // Start is called before the first frame update
    void Start()
    {
        Camera.main.gameObject.GetComponent<Click>().selectableTaskObjects.Add(this.gameObject);
    }
}
