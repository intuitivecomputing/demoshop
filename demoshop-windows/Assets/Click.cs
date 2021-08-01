// Click.cs
// Based on code from https://connect.unity.com/u/c00pala-j

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Click : MonoBehaviour
{
    // Public Variables
    [HideInInspector]
    public List<GameObject> selectedObjects;    // List containing all currently selected objects
    [HideInInspector]
    public List<GameObject> selectableObjects;  // List containing objects that can be selected
    [HideInInspector]
    public List<GameObject> selectedTaskObjects;  // List containing all currently selected task objects
    [HideInInspector]
    public List<GameObject> selectableTaskObjects;  // List containing task objects that can be selected

    // Private variables
    // Positions
    private Vector3 mousePos1;
    private Vector3 mousePos2;  

    // Scripts
    private RosSharp.RosBridgeClient.RecordDemo recordDemoScript;

    // GUI
    private bool isMoving;
    private bool isDragging;
    private bool clickedButton;

    void Awake()
    {
        selectedObjects = new List<GameObject>();
        selectableObjects = new List<GameObject>();
        selectedTaskObjects = new List<GameObject>();
        recordDemoScript = Camera.main.GetComponent<RosSharp.RosBridgeClient.RecordDemo>();
    }

    void Update()
    {
    	// Status of GUI events
        isMoving = recordDemoScript.isMoving;
        isDragging = recordDemoScript.isDragging;
        clickedButton = recordDemoScript.clickedButton;

        if (Input.GetKey(KeyCode.C))    	// "C" is clear
        {
            ClearSelection();
        }

        if (Input.GetMouseButtonDown(0))    // Left mouse button click
        {
            mousePos1 = Camera.main.ScreenToViewportPoint(Input.mousePosition);
            RaycastHit rayHit;

            if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out rayHit, Mathf.Infinity)) // Clicked on an object
            {
                WaypointScript waypointScript = rayHit.collider.GetComponent<WaypointScript>();
                TrackedObjectScript trackedObjectScript = rayHit.collider.GetComponent<TrackedObjectScript>();

                if (waypointScript != null)
                {
                    if (Input.GetKey("left ctrl"))  // Select multiple if left control key clicked
                    {
                        if (waypointScript.currentlySelected == false)  // Selected waypoint is not already selected
                        {
                            selectedObjects.Add(rayHit.collider.gameObject);    
                            waypointScript.currentlySelected = true;
                        }
                        else    // Clicked a waypoint that is already selected, so deselect it
                        {
                            selectedObjects.Remove(rayHit.collider.gameObject);
                            waypointScript.currentlySelected = false;   
                        }
                    }
                    else    // Do not select multiple
                    {
                        if (!clickedButton) // User clicked somewhere that is not a button
                        {
                            ClearSelection();   
                        }

                        selectedObjects.Add(rayHit.collider.gameObject);
                        waypointScript.currentlySelected = true;
                    }
                }
                else if (trackedObjectScript != null)
                {
                    if (Input.GetKey("left ctrl"))  // Select multiple if left control key clicked
                    {
                        if (trackedObjectScript.currentlySelected == false)  // Selected tracked object is not already selected
                        {
                            selectedTaskObjects.Add(rayHit.collider.gameObject);    
                            trackedObjectScript.currentlySelected = true;
                        }
                        else    // Clicked a tracked object that is already selected, so deselect it
                        {
                            selectedTaskObjects.Remove(rayHit.collider.gameObject);
                            trackedObjectScript.currentlySelected = false;   
                        }
                    }
                    else    // Do not select multiple
                    {
                        if (!clickedButton) // User clicked somewhere that is not a button
                        {
                            ClearSelection();   
                        }

                        selectedTaskObjects.Add(rayHit.collider.gameObject);
                        trackedObjectScript.currentlySelected = true;
                    }
                }
            }
        }

        if (isMoving == false && isDragging == false)  // User is in select mode
        {
            if (Input.GetMouseButtonUp(0) && clickedButton == false)    // User finished clicking left mouse button
            {
                mousePos2 = Camera.main.ScreenToViewportPoint(Input.mousePosition);

                if (mousePos1 != mousePos2) // User dragged a rectangle
                {
                    SelectObjects();
                }

                recordDemoScript.isMoving = true;
                recordDemoScript.isDragging = true;
            }  
        }

        recordDemoScript.clickedButton = false; // Reset boolean indicating button click
    }

    /*** Helper Functions ***/
    // Selects all the objects within the rectangle that the user dragged
    void SelectObjects()
    {
        List<GameObject> remObjects = new List<GameObject>();   	// List of waypoints that were removed from scene since selection
        List<GameObject> remTaskObjects = new List<GameObject>();   // List of task objects that were removed from scene since selection

        if (Input.GetKey("left ctrl") == false && clickedButton == false)   // User is starting a new selection
        {
            ClearSelection();
        }

        Rect selectRect = new Rect(mousePos1.x, mousePos1.y, mousePos2.x - mousePos1.x, mousePos2.y - mousePos1.y); // Dragged rectangle
        foreach (GameObject selectObject in selectableObjects)  // Check if each selectable object is in the dragged rectangle
        {
            if (selectObject != null)
            {
                if (selectRect.Contains(Camera.main.WorldToViewportPoint(selectObject.transform.position), true))   // In rectangle
                {
                    selectedObjects.Add(selectObject);
                    selectObject.GetComponent<WaypointScript>().currentlySelected = true;
                }
            }
            else    // Object must have been removed from scene since selection
            {
                remObjects.Add(selectObject);
            }
        }

        foreach (GameObject selectTaskObject in selectableTaskObjects)  // Check if each selectable task object is in the dragged rectangle
        {
            if (selectTaskObject != null)
            {
                if (selectRect.Contains(Camera.main.WorldToViewportPoint(selectTaskObject.transform.position), true))   // In rectangle
                {
                    selectedTaskObjects.Add(selectTaskObject);
                    selectTaskObject.GetComponent<TrackedObjectScript>().currentlySelected = true;
                }
            }
            else    // Object must have been removed from scene since selection
            {
                remTaskObjects.Add(selectTaskObject);
            }
        }

        if (remObjects.Count > 0)   	// Objects need to be removed from selectable list since they were removed from the active scene
        {
            foreach (GameObject rem in remObjects)
            {
                selectableObjects.Remove(rem);
            }

            remObjects.Clear();
        }

        if (remTaskObjects.Count > 0)   // Task objects need to be removed from selectable list since they were removed from the active scene
        {
            foreach (GameObject remTask in remTaskObjects)
            {
                selectableTaskObjects.Remove(remTask);
            }

            remTaskObjects.Clear();
        }
    }

    // Clears the current selection
    public void ClearSelection()
    {
        if (selectedObjects.Count > 0)  // There are objects selected
        {
            foreach (GameObject obj in selectedObjects) 
            {
                if (obj != null)
                {
                    obj.GetComponent<WaypointScript>().currentlySelected = false;   
                }
            }

            selectedObjects.Clear();
        }

        if (selectedTaskObjects.Count > 0)  // There are task objects selected
        {
            foreach (GameObject obj in selectedTaskObjects) 
            {
                if (obj != null)
                {
                    obj.GetComponent<TrackedObjectScript>().currentlySelected = false;   
                }
            }

            selectedTaskObjects.Clear();
        }

        recordDemoScript.displayMenu = false;   // If waypoints menu is open, close it
    }
}

