using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Canonical Simplifed Model for Nonholonomic Mobile Robots after Lynch et. al 2017
public class steered_wheel : MonoBehaviour
{
    public Rigidbody rb;
    public ConfigurableJoint cj;
    public GameObject go_wheel;
    public Transform tf_base;
    public bool motor;
    public bool steering;
    public float r;
    float x_velocity, z_velocity, w_velocity;
    float vel, phi;
    float x_dist, z_dist, l;

    float[,] matVel;
    // Start is called before the first frame update
    void Start()
    {
        x_velocity = 10;
        z_velocity = 10;
        w_velocity = 10;

        rb = GetComponent<Rigidbody>();
        cj = GetComponent<ConfigurableJoint>();
        go_wheel = gameObject;
        Vector3 wheel_center_position = rb.transform.position;
        tf_base = go_wheel.transform.parent;

        // calculate distance from reference point (center of base) to wheel_center
        x_dist = transform.position.x - tf_base.position.x;
        // z_dist = transform.position.z - tf_base.position.z;
        z_dist = 2;
        l = Mathf.Sqrt(Mathf.Pow(x_dist, 2) + Mathf.Pow(z_dist, 2));
    }

    void FixedUpdate()
    {
        // get x,y and w velocity input
        float x = x_velocity * Input.GetAxis("Horizontal");
        float z = z_velocity * Input.GetAxis("Vertical");
        float w = w_velocity * Input.GetAxis("Orientation");
        // Debug.Log(transform.name + " x: "+ x);
        // Debug.Log(transform.name + " y: "+ y);
        // Debug.Log(transform.name + " w: "+ w);

        // input velocity-matrix
        float[,] matXY = new float[2, 1];
        matXY[0, 0] = x;
        matXY[1, 0] = z;

        // input angular velocity matrix
        float[,] matW = new float[2, 2];
        matW[0, 0] = w;
        matW[0, 1] = 0;
        matW[1, 0] = 0;
        matW[1, 1] = w;

        // Angle of rotation between base and world
        // negative sign because unity measures rotation leftwise and not rightwise
        // (positive rotation results in negative angle)
        phi = -Vector3.SignedAngle(Vector3.right, tf_base.forward, Vector3.up) * Mathf.Deg2Rad;
        Debug.Log(transform.name + "phi: " + phi * Mathf.Rad2Deg);

        // Geometry-matrix
        float[,] matG = new float[2, 1];
        matG[0, 0] = z_dist;
        matG[1, 0] = x_dist;

        // Constraint-matrix
        float[,] matJ = new float[2, 2];
        matJ[0, 0] = -Mathf.Sin(phi);
        matJ[0, 1] = -Mathf.Cos(phi);
        matJ[1, 0] = Mathf.Cos(phi);
        matJ[1, 1] = -Mathf.Sin(phi);

        // calculate world-frame motion of point P (the wheel)
        matVel = MatrixMultiply(matJ, matG);
        matVel = MatrixMultiply(matW, matVel);
        matVel = MatrixAdd(matXY, matVel);

        // steering angle
        // is 0 if there is no angular velocity input w
        // float beta = Mathf.Atan(matVel[0, 0] / matVel[1, 0]) * Mathf.Rad2Deg;

        // Quaternion rotation;
        // rotation = rb.transform.rotation;
        // Vector3 rot = rotation.eulerAngles;
        // rot = new Vector3(rot.x, beta, rot.z);
        // transform.rotation = Quaternion.Euler(rot);

        // inverse Jacobian (world to robot)
        float[,] matI = new float[2, 2];

        // special case reference point p on line z_r = 0
        if (z_dist == 0)
        {
            matI[0, 0] = Mathf.Cos(phi);
            matI[0, 1] = Mathf.Sin(phi);
            matI[1, 0] = -Mathf.Sin(phi);
            matI[1, 1] = Mathf.Cos(phi);

            // calculate robot-frame motion
            float[,] matVW = MatrixMultiply(matI, matVel);
            vel = matVW[0, 0];
        }
        else
        {
            matI[0, 0] = z_dist * Mathf.Cos(phi) - x_dist * Mathf.Sin(phi);
            matI[0, 1] = z_dist * Mathf.Sin(phi) + x_dist * Mathf.Cos(phi);
            matI[1, 0] = -Mathf.Sin(phi);
            matI[1, 1] = Mathf.Cos(phi);

            // z_dist
            float[,] matZ_R = new float[2, 2];
            matZ_R[0, 0] = 1 / z_dist;
            matZ_R[0, 1] = 0;
            matZ_R[1, 0] = 0;
            matZ_R[1, 1] = 1 / z_dist;

            // calculate robot-frame motion
            float[,] matVW = MatrixMultiply(matI, matVel);
            matVW = MatrixMultiply(matZ_R, matVW);
            vel = matVW[0, 0];
            cj.targetAngularVelocity = Vector3.up * matVW[1,0];
        }

        
        // cj.targetAngularVelocity = Vector3.right * vel;
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

    float[,] MatrixAdd(float[,] matA, float[,] matB)
    {
        float[,] res;
        int rowsA = matA.GetLength(0);
        int columnsA = matA.GetLength(1);
        int rowsB = matB.GetLength(0);
        int columnsB = matB.GetLength(1);

        res = new float[rowsA, columnsA];

        for (int i = 0; i < rowsA; i++)
        {
            for (int j = 0; j < columnsA; j++)
            {
                res[i, j] = matA[i, j] + matB[i, j];
            }
        }
        return res;
    }

    // Visualization for debugging
    void OnDrawGizmos()
    {
        // draw line for v
        Gizmos.color = Color.red;

        Gizmos.DrawLine(
            rb.transform.position,
            new Vector3(
                rb.transform.position.x + matVel[0,0] / 10,
                rb.transform.position.y,
                rb.transform.position.z + matVel[1,0] / 10
                )
                );

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
