using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vw_standard_wheel : MonoBehaviour
{
    public Rigidbody rb;
    public GameObject go_wheel;
    public Transform tf_base;
    public bool motor;
    public bool steering;
    float z_r;
    float x_r;
    public float max_velocity;
    public float max_angular_vel;
    float x_p, z_p;
    // Start is called before the first frame update
    void Start()
    {
        max_velocity = 10;
        max_angular_vel = 10;
        rb = GetComponent<Rigidbody>();
        go_wheel = gameObject;
        Vector3 wheel_center_position = rb.transform.position;
        tf_base = go_wheel.transform.parent;
        Vector3 base_center_position = tf_base.position;
        z_r = 0;
        x_r = base_center_position.x + wheel_center_position.x;
        //Debug.Log(transform.name + " z_r: " + z_r);
        //Debug.Log(transform.name + " x_r: " + x_r);
    }
        
    void FixedUpdate()
    {
        // get velocity and angular velocity input
        float v = max_velocity * Input.GetAxis("Vertical");
        float w = max_angular_vel * Input.GetAxis("Horizontal");

        //Debug.Log(transform.name + "v: "+ v);
        //Debug.Log(transform.name + "w: "+ w);

        // Angle of rotation between base and world
        // negative sign because unity measures rotation leftwise and not rightwise
        // (positive rotation results in negative angle)
        float phi = - Vector3.SignedAngle(tf_base.forward, Vector3.forward, Vector3.up) * Mathf.Deg2Rad;
        //Debug.Log(transform.name + "phi: "+ phi * Mathf.Rad2Deg);

        // convert velocity to z and x components in world coordinates
        float z = v * Mathf.Cos(phi);
        float x = v * Mathf.Sin(phi);
        //Debug.Log(transform.name + "x: "+ x);
        //Debug.Log(transform.name + "y: "+ y);

        // calculate velocity components for point P (the wheel)
        z_p = z + w * (-Mathf.Sin(phi) * z_r - Mathf.Cos(phi) * x_r);
        x_p = x + w * (Mathf.Cos(phi) * z_r - Mathf.Sin(phi) * x_r);
        float v_p = Mathf.Sqrt(Mathf.Pow(x_p, 2) + Mathf.Pow(z_p, 2));
        float beta = Mathf.Atan(x_p/z_p);

        Debug.Log(transform.name + " beta: "+ beta);
        //Debug.Log(transform.name + " z_p: "+ z_p);
        //Debug.Log(transform.name + " x_p: "+ x_p);

        rb.velocity = new Vector3(x_p, 0, z_p);
        /*Quaternion rotation;
        rotation = rb.transform.rotation;
        Vector3 rot = rotation.eulerAngles;
        rot = new Vector3(rot.x, beta + phi, rot.z);
        rb.transform.rotation = Quaternion.Euler(rot);
        */
    }

    // Visualization for debugging
    void OnDrawGizmos()
    {
        // draw line for v
        Gizmos.color = Color.red;
        
        Gizmos.DrawLine(
            rb.transform.position, 
            new Vector3(
                rb.transform.position.x + x_p/10, 
                rb.transform.position.y, 
                rb.transform.position.z + z_p/10));

        // draw line for x_p
        // Gizmos.color = Color.green;
        // Gizmos.DrawLine(
        //     rb.transform.position, 
        //     new Vector3(
        //         rb.transform.position.x + x_p/10, 
        //         rb.transform.position.y, 
        //         rb.transform.position.z));
        }
}
