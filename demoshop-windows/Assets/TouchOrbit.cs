// TouchOrbit.cs

 using UnityEngine;
 using UnityEngine;
 using System.Collections;
 
 // Handles moving around 3-D scene
 public class TouchOrbit : MonoBehaviour
 {
 	[SerializeField]
    private Transform Target;	// Specify the target around which to orbit
    [SerializeField]
    private float distance = 2.0f;
    [SerializeField]
    private float xSpeed = 20.0f;
    [SerializeField]
    private float ySpeed = 20.0f;
    [SerializeField]
    private float yMinLimit = -90f;
    [SerializeField]
    private float yMaxLimit = 90f;
    [SerializeField]
    private float distanceMin = 10f;
    [SerializeField]
    private float distanceMax = 10f;
    [SerializeField]
    private float smoothTime = 2f;
    private float rotationXAxis = 0.0f;
    private float rotationYAxis = 0.0f;
    private float velocityX = 0.0f;
    private float velocityY = 0.0f;
    private bool isMoving;  // Indicates whether the user is currently in move mode
    private bool isDragging;    // Indicates whether the user is currently dragging a window
    private RosSharp.RosBridgeClient.RecordDemo recordDemoScript;
    private ClickAndDrag clickAndDragScript;

    void Start()
    {
        recordDemoScript = Camera.main.GetComponent<RosSharp.RosBridgeClient.RecordDemo>();
        clickAndDragScript = Camera.main.GetComponent<ClickAndDrag>();
    }

    void Update()
    {
    	isMoving = recordDemoScript.isMoving;
        isDragging = recordDemoScript.isDragging;
        if (isMoving == true && isDragging == false)    // User is in move mode
        {
	        foreach (Touch touch in Input.touches) 
			{
				if (touch.phase == TouchPhase.Moved) 
				{
	            	clickAndDragScript.velocityX += xSpeed * Input.GetTouch(0).deltaPosition.x * distance * 0.02f;
	            	clickAndDragScript.velocityY += ySpeed * Input.GetTouch(0).deltaPosition.y * 0.02f;
	        	}
	    	}
	    }
    }

    // ClampAngle: Enforces a floor and ceiling for the specified angle
    public static float ClampAngle(float angle, float min, float max)
    {
        if (angle < 0f)
        {
            angle = 0f;
        }
        if (angle > 360f)
        {
            angle  = 360f;
        }
        return Mathf.Clamp(angle, min, max);
    }
 }



