// CameraHandler.cs
// From https://kylewbanks.com/blog/unity3d-panning-and-pinch-to-zoom-camera-with-touch-and-mouse-input

using UnityEngine;
using System.Collections;

public class CameraHandler : MonoBehaviour 
{
    private static readonly float PanSpeed = 20f;
    private static readonly float ZoomSpeedTouch = 0.1f;
    private static readonly float ZoomSpeedMouse = 0.5f;
    
    private static readonly float[] BoundsX = new float[]{-10f, 5f};
    private static readonly float[] BoundsZ = new float[]{-18f, -4f};
    private static readonly float[] ZoomBounds = new float[]{10f, 85f};
    
    private Camera cam;
    
    private Vector3 lastPanPosition;
    private int panFingerId; 				// Touch mode only
    
    private bool wasZoomingLastFrame; 		// Touch mode only
    private Vector2[] lastZoomPositions; 	// Touch mode only

    [HideInInspector]
    public bool isTouching;

    void Awake() 
    {
        cam = GetComponent<Camera>();
    }
    
    void Update() 
    {
        if (Input.touchSupported && (Application.platform != RuntimePlatform.WebGLPlayer)) 
        {
            HandleTouch();
        }
    }
    
    void HandleTouch() 
    {
        switch(Input.touchCount) 
        {
	        case 2: // Zooming
	            Vector2[] newPositions = new Vector2[]{Input.GetTouch(0).position, Input.GetTouch(1).position};
	            if (!wasZoomingLastFrame) 
	            {
	                lastZoomPositions = newPositions;
	                wasZoomingLastFrame = true;
	            } 
	            else 
	            {
	                // Zoom based on the distance between the new positions compared to the distance between the previous positions
	                float newDistance = Vector2.Distance(newPositions[0], newPositions[1]);
	                float oldDistance = Vector2.Distance(lastZoomPositions[0], lastZoomPositions[1]);
	                float offset = newDistance - oldDistance;
	    
	                ZoomCamera(offset, ZoomSpeedTouch);
	    
	                lastZoomPositions = newPositions;
	            }
	            break;     
	        default: 
	            wasZoomingLastFrame = false;
	            break;
        }
    }
    
    void ZoomCamera(float offset, float speed) 
    {
        if (offset == 0) 
        {
            return;
        }
    
        cam.fieldOfView = Mathf.Clamp(cam.fieldOfView - (offset * speed), ZoomBounds[0], ZoomBounds[1]);
    }

}
