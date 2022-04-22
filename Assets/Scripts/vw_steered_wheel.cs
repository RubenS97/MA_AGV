using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vw_steered_wheel : MonoBehaviour
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

    float z_p, x_p, beta, beta_old = 0;
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
        z_r = 2;
        x_r = base_center_position.x + wheel_center_position.x;
    }
        
    void FixedUpdate()
    {
        // get velocity and angular velocity input
        float v = max_velocity * Input.GetAxis("Vertical");
        float w = max_angular_vel * Input.GetAxis("Horizontal");

        Debug.Log(transform.name + " v: "+ v);
        Debug.Log(transform.name + " w: "+ w);

        // Angle of rotation between base and world
        // negative sign because unity measures rotation leftwise and not rightwise
        // (positive rotation results in negative angle)
        float phi = - Vector3.SignedAngle(tf_base.forward, Vector3.forward, Vector3.up) * Mathf.Deg2Rad;
        Debug.Log(transform.name + " phi: "+ phi * Mathf.Rad2Deg);

        // convert velocity to z and x components in world coordinates
        float z = v * Mathf.Cos(phi);
        float x = v * Mathf.Sin(phi);

        // calculate velocity components for point P (the wheel)
        z_p = z + w * (-Mathf.Sin(phi) * z_r - Mathf.Cos(phi) * x_r);
        x_p = x + w * (Mathf.Cos(phi) * z_r - Mathf.Sin(phi) * x_r);

        // steering angle
        // is 0 if there is no angular velocity input w
        beta = Mathf.Atan(x_p/z_p) * Mathf.Rad2Deg;
        Debug.Log(transform.name + " beta: "+ beta);

        // rb.velocity = new Vector3(x_p, 0, z_p);

        float beta_delta = beta - beta_old;
        beta_old = beta;
        
        
        Quaternion rotation;
        rotation = rb.transform.rotation;
        Vector3 rot = rotation.eulerAngles;
        rot = new Vector3(rot.x, beta, rot.z);
        transform.rotation = Quaternion.Euler(rot);
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
                rb.transform.position.z + z_p/10)
                );

        // draw line for x_p
        // Gizmos.color = Color.green;
        // Gizmos.DrawLine(rb.transform.position, new Vector3(rb.transform.position.x + x_p/10, rb.transform.position.y, rb.transform.position.z));

        // draw angle beta
        /*Gizmos.color = Color.blue;
        Gizmos.DrawLine(rb.transform.position, 
        new Vector3(
        rb.transform.position.x + Mathf.Sin(beta)*4, 
        rb.transform.position.y, 
        rb.transform.position.z + Mathf.Cos(beta)*4));*/
    }
}
