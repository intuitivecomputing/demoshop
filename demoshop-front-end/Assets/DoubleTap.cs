// DoubleTap.cs

using UnityEngine;

public class DoubleTap : MonoBehaviour
{
	int TapCount;
	public float MaxDoubleTapTime = 0.1f;
	float NewTime;

	void Start () 
	{
		TapCount = 0;
	}

	void Update () {
		if (Input.touchCount == 1) 
		{
			Touch touch = Input.GetTouch (0);
	         
		 	if (touch.phase == TouchPhase.Ended) 
		 	{
		     	TapCount += 1;
		 	}

		 	if (TapCount == 1) 
		 	{
		    	NewTime = Time.time + MaxDoubleTapTime;
			}
		 	else if(TapCount == 2 && Time.time <= NewTime)
		 	{           
		     	TapCount = 0;
		 	}
		}
	         
	    if (Time.time > NewTime) 
	    {
	        TapCount = 0;
	    }
    }
}