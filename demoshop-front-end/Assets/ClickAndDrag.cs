// ClickAndDrag.cs

 using UnityEngine;
 using UnityEngine;
 using System.Collections;
 
 // Handles moving around 3-D scene
 public class ClickAndDrag : MonoBehaviour
 {
    [SerializeField]
    private Transform Target;   // Specify the target around which to orbit
    [SerializeField]
    public float distance = 2.0f;
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
    [HideInInspector]
    public float rotationXAxis = 0.0f;
    [HideInInspector]
    public float rotationYAxis = 0.0f;
    [HideInInspector]
    public float velocityX = 0.0f;
    [HideInInspector]
    public float velocityY = 0.0f;

    private bool isMoving;  	// Indicates whether the user is currently in move mode
    private bool isDragging;    // Indicates whether the user is currently dragging a window
    private bool clickedButton;	// Indicates whether the user clicked a button

    private RosSharp.RosBridgeClient.RecordDemo recordDemoScript;

    void Start()
    {
        recordDemoScript = Camera.main.gameObject.GetComponent<RosSharp.RosBridgeClient.RecordDemo>();
    }

    void Update()
    {
        isMoving = recordDemoScript.isMoving;
        isDragging = recordDemoScript.isDragging;
        clickedButton = recordDemoScript.clickedButton;

        if ((isMoving == true) && (isDragging == false) && (clickedButton == false))    // User is in move mode
        {
            if (Input.GetAxis("Mouse ScrollWheel") < 0)  // Scrolling forward
            {
                Camera.main.orthographicSize += 0.5f;
                distance += 0.5f;
            }  
            if (Input.GetAxis("Mouse ScrollWheel") > 0)  // Scrolling backward
            {
                Camera.main.orthographicSize -= 0.5f;
                distance -= 0.5f;
            }
            if (Input.GetMouseButton(0))  // Dragging
            {
                velocityX += xSpeed * Input.GetAxis("Mouse X") * distance * 0.02f;
                velocityY += ySpeed * Input.GetAxis("Mouse Y") * 0.02f;
            }
            rotationYAxis += velocityX;
            rotationXAxis -= velocityY;
            rotationXAxis = ClampAngle(rotationXAxis, yMinLimit, yMaxLimit);
            Quaternion rotation = Quaternion.Euler(rotationXAxis, rotationYAxis, 0);
            Vector3 negDistance = new Vector3(0.0f, 0.0f, -distance);
            Vector3 position = rotation * negDistance + Target.position;
            transform.rotation = rotation;
            transform.position = position;
            velocityX = Mathf.Lerp(velocityX, 0, Time.deltaTime * smoothTime);
            velocityY = Mathf.Lerp(velocityY, 0, Time.deltaTime * smoothTime);        
        }
    }

    /*** Helper Function ***/
    // Enforces a floor and ceiling for the specified angle
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
