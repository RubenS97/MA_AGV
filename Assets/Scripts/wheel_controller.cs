using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class wheel_controller : MonoBehaviour
{
    Transform[] tf_wheels;
    Rigidbody[] rb_wheels;
    ConfigurableJoint[] cj_wheels;

    Component[] cp_wheels;
    // max velocitys for hand-control
    float x_velocity, z_velocity, w_velocity;

    // calculation variables
    float[] x_dist, z_dist, beta, gamma, radius, vel;
    float phi;
    // constraint matrix H(phi) done with jagged matrix 
    float[][,] matH;

    // Start is called before the first frame update
    void Start()
    {
        x_velocity = 10;
        z_velocity = 10;
        w_velocity = 10 * 2;

        tf_wheels = GetComponentsInChildren<Transform>();
        Debug.Log(transform.name + " \t number child transforms: \t" + tf_wheels.Length);
        Debug.Log(transform.name + " \t child transforms 1 : \t" + tf_wheels[0]);

        rb_wheels = GetComponentsInChildren<Rigidbody>();
        Debug.Log(transform.name + " number child rigidbodies: \t" + rb_wheels.Length);
        cj_wheels = GetComponentsInChildren<ConfigurableJoint>();

        cp_wheels = GetComponentsInChildren<mecanum_wheel>();

        matH = new float[tf_wheels.Length][,];

        int wheel_index = 0;
        x_dist = new float[tf_wheels.Length];
        z_dist = new float[tf_wheels.Length];
        beta = new float[tf_wheels.Length];
        gamma = new float[tf_wheels.Length];
        radius = new float[tf_wheels.Length];
        foreach (Transform wheel in tf_wheels)
        {
            Debug.Log(transform.name + " \t child wheel.position.x  : \t" + wheel.position.x);
            Debug.Log(transform.name + " \t transform.position.x   : \t" + transform.position.x);
            Debug.Log(transform.name + " \t x_dist[wheel_index]   : \t" + x_dist[wheel_index]);
            x_dist[wheel_index] = wheel.position.x - transform.position.x;
            z_dist[wheel_index] = wheel.position.z - transform.position.z;
            Debug.Log(transform.name + " \t wheel.GetComponent<mecanum_wheel>().beta : \t" + cp_wheels[wheel_index].);
            beta[wheel_index] = wheel.GetComponent<mecanum_wheel>().beta;
            gamma[wheel_index] = wheel.GetComponent<mecanum_wheel>().gamma;
            radius[wheel_index] = wheel.GetComponent<mecanum_wheel>().r;

            matH[wheel_index][0, 0] =
            (1 / (radius[wheel_index] * Mathf.Cos(gamma[wheel_index]))) *
            z_dist[wheel_index] * Mathf.Sin(beta[wheel_index] + gamma[wheel_index]) -
            x_dist[wheel_index] * Mathf.Cos(beta[wheel_index] + gamma[wheel_index]);
            Debug.Log(transform.name + " matH() \t" + matH[wheel_index][0, 0] );

            wheel_index++;
        }
    }

    // FixedUpdate is called once every 2 ms
    void FixedUpdate()
    {
        // get x,y and w velocity input
        float x = x_velocity * Input.GetAxis("Horizontal");
        float z = z_velocity * Input.GetAxis("Vertical");
        float w = -w_velocity * Input.GetAxis("Orientation");

        // input velocity-matrix
        float[,] matVel = new float[3, 1];
        matVel[0, 0] = w;
        matVel[1, 0] = x;
        matVel[2, 0] = z;

        // Angle of rotation between base and world
        // negative sign because unity measures rotation leftwise and not rightwise
        // (positive rotation results in negative angle)
        phi = Vector3.SignedAngle(transform.forward, Vector3.right, Vector3.up) * Mathf.Deg2Rad;

        int wheel_index = 0;
        foreach (Transform wheel in tf_wheels)
        {
            matH[wheel_index][0, 1] = (1 / (radius[wheel_index] * Mathf.Cos(gamma[wheel_index]))) * Mathf.Cos(beta[wheel_index] + gamma[wheel_index] + phi);
            matH[wheel_index][0, 2] = (1 / (radius[wheel_index] * Mathf.Cos(gamma[wheel_index]))) * Mathf.Sin(beta[wheel_index] + gamma[wheel_index] + phi);

            vel[wheel_index] = MatrixMultiply(matH[wheel_index], matVel)[0,0];
            cj_wheels[wheel_index].targetAngularVelocity = Vector3.right * vel[wheel_index];
            wheel_index++;
        }
    }

    float[,] MatrixMultiply(float[,] matA, float[,] matB)
    {
        float[,] res;
        int rowsA = matA.GetLength(0);
        int columnsA = matA.GetLength(1);
        int rowsB = matB.GetLength(0);
        int columnsB = matB.GetLength(1);

        if (columnsA != rowsB)
        {
            res = null;
        }
        else
        {
            res = new float[rowsA, columnsB];
            for (int a = 0; a < columnsB; a++)
            {
                for (int i = 0; i < rowsA; i++)
                {
                    float sum = 0;
                    for (int j = 0; j < columnsA; j++)
                    {
                        sum += matA[i, j] * matB[j, a];
                    }
                    res[i, a] = sum;
                }
            }
        }
        return res;
    }

    // Visualization for debugging
    void OnDrawGizmos()
    {
        // draw line for v
        Gizmos.color = Color.red;

        int wheel_index = 0;
        foreach (Rigidbody rb_wheel in rb_wheels)
        {
            Gizmos.DrawLine(
                rb_wheel.transform.position,
                new Vector3(
                rb_wheel.transform.position.x + vel[wheel_index] * Mathf.Cos(phi) / 10,
                rb_wheel.transform.position.y,
                rb_wheel.transform.position.z + vel[wheel_index] * Mathf.Sin(phi) / 10)
                );
            wheel_index++;
        }
        // draw line for x_p
        // Gizmos.color = Color.green;
        // Gizmos.DrawLine(rb.transform.position, new Vector3(rb.transform.position.x + vel * Mathf.Cos(phi), rb.transform.position.y, rb.transform.position.z));

        // draw angle beta
        /*Gizmos.color = Color.blue;
        Gizmos.DrawLine(rb.transform.position, 
        new Vector3(
        rb.transform.position.x + Mathf.Sin(beta)*4, 
        rb.transform.position.y, 
        rb.transform.position.z + Mathf.Cos(beta)*4));*/
    }
}
