// SelectWaypoints.cs
// Based on code from https://connect.unity.com/u/c00pala-j

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectWaypoints : MonoBehaviour
{
	[SerializeField]
	private RectTransform selectSquareImage;
	private Vector3 startPos;
	private Vector3 endPos;
	private bool isMoving;
	private bool isDragging;
	private RosSharp.RosBridgeClient.RecordDemo recordDemoScript;

    void Start()
    {
        selectSquareImage.gameObject.SetActive(false);
        recordDemoScript = Camera.main.GetComponent<RosSharp.RosBridgeClient.RecordDemo>();
    }
    
    void Update()
    {
    	isMoving = recordDemoScript.isMoving;
    	isDragging = recordDemoScript.isDragging;

    	if (isMoving == false && isDragging == false)
    	{
	        if (Input.GetMouseButtonDown(0))
	        {
	        	RaycastHit hit;

	        	if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hit, Mathf.Infinity))
	        	{
	        		startPos = hit.point;
	        	}
	        }

	        if (Input.GetMouseButtonUp(0))
	        {
	        	selectSquareImage.gameObject.SetActive(false);
	        }

	        if (Input.GetMouseButton(0))
	        {
	        	if (!selectSquareImage.gameObject.activeInHierarchy)
	        	{
	        		selectSquareImage.gameObject.SetActive(true);
	        	}

	        	endPos = Input.mousePosition;

	        	Vector3 squareStart = Camera.main.WorldToScreenPoint(startPos);
	        	squareStart.z = 0f;

	        	Vector3 center = (squareStart + endPos) / 2f;

	        	selectSquareImage.position = center;

	        	float sizeX = Mathf.Abs(squareStart.x - endPos.x);
	        	float sizeY = Mathf.Abs(squareStart.y - endPos.y);

	        	selectSquareImage.sizeDelta = new Vector2(sizeX, sizeY);
	        }
    	}
    }
}
