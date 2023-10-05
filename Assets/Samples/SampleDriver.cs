using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SampleDriver : MonoBehaviour
{
    public Rigidbody drived;
    public float velocity = 1.0f;
    void FixedUpdate()
    {
        var vel = new Vector3(0, 0, 0);
        if (Input.GetKey(KeyCode.W))
        {
            vel += Vector3.right;
        }
        if (Input.GetKey(KeyCode.S))
        {
            vel += Vector3.left;
        }
        if (Input.GetKey(KeyCode.A))
        {
            vel += Vector3.forward;
        }
        if (Input.GetKey(KeyCode.D))
        {
            vel += Vector3.back;
        }
        drived.MovePosition(drived.transform.position + vel.normalized * velocity * Time.fixedDeltaTime);
    }
}
