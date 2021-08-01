// Drag.cs

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class Drag : EventTrigger 
{
    [HideInInspector]
    public bool isDragging;  // Indicates if the user is dragging the game object to which this script is attached

    public void Update() 
    {
        if (isDragging) 
        {
            transform.position = new Vector2(Input.mousePosition.x, Input.mousePosition.y);
        }
    }

    public override void OnPointerDown(PointerEventData eventData) 
    {
        isDragging = true;
    }

    public override void OnPointerUp(PointerEventData eventData) 
    {
        isDragging = false;
    }
}