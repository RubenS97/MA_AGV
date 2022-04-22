using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class mecanum_wheel : MonoBehaviour
{
    public Rigidbody rb;
    public ConfigurableJoint cj;
    public GameObject go_wheel;
    public Transform tf_base;
    public float gamma, beta, r;
    float x_velocity, z_velocity, w_velocity;
    float alpha, l;
    float phi, vel;
    float x_dist, z_dist;
    // Start is called before the first frame update
    void Start()
    {
        x_velocity = 10;
        z_velocity = 10;
        w_velocity = 10 * 2;

        rb = GetComponent<Rigidbody>();
        cj = GetComponent<ConfigurableJoint>();
        go_wheel = gameObject;
        tf_base = go_wheel.transform.parent;

        // calculate distance from reference point (center of base) to wheel_center
        x_dist = transform.position.x - tf_base.position.x;
        z_dist = transform.position.z - tf_base.position.z;
        l = Mathf.Sqrt(Mathf.Pow(x_dist, 2) + Mathf.Pow(z_dist, 2));
        // Debug.Log(transform.name + " l: "+ l);

        // get angle between robot forward direction and where the wheel is
        if (z_dist > 0 && x_dist > 0)
        {
            alpha = 360 - Mathf.Atan(x_dist/z_dist) * Mathf.Rad2Deg;
        }
        else if (z_dist < 0)
        {
            alpha = 180 - Mathf.Atan(x_dist/z_dist) * Mathf.Rad2Deg;
        }
        else alpha = - Mathf.Atan(x_dist/z_dist) * Mathf.Rad2Deg;
        Debug.Log(transform.name + " alpha: "+ alpha);

        beta = 0;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // get x,y and w velocity input
        float x = x_velocity * Input.GetAxis("Horizontal");
        float z = z_velocity * Input.GetAxis("Vertical");
        float w = -w_velocity * Input.GetAxis("Orientation");
        // Debug.Log(transform.name + " x: "+ x);
        // Debug.Log(transform.name + " y: "+ y);
        // Debug.Log(transform.name + " w: "+ w);

        // input velocity-matrix
        float[,] matVel = new float[3, 1];
        matVel[0, 0] = w;
        matVel[1, 0] = x;
        matVel[2, 0] = z;

        // Angle of rotation between base and world
        // negative sign because unity measures rotation leftwise and not rightwise
        // (positive rotation results in negative angle)
        phi =  Vector3.SignedAngle(tf_base.forward, Vector3.right, Vector3.up) * Mathf.Deg2Rad;
        // Debug.Log(transform.name + " phi: "+ phi * Mathf.Rad2Deg);

        // rotation-matrix between World an Robot
        float[,] matROT = new float[3, 3];
        // matROT[0, 0] = Mathf.Cos(phi);
        // matROT[0, 1] = Mathf.Sin(phi);
        // matROT[0, 2] = 0;
        // matROT[1, 0] = -Mathf.Sin(phi);
        // matROT[1, 1] = Mathf.Cos(phi);
        // matROT[1, 2] = 0;
        // matROT[2, 0] = 0;
        // matROT[2, 1] = 0;
        // matROT[2, 2] = 1;
        matROT[0, 0] = 1;
        matROT[0, 1] = 0;
        matROT[0, 2] = 0;
        matROT[1, 0] = 0;
        matROT[1, 1] = Mathf.Cos(phi);
        matROT[1, 2] = Mathf.Sin(phi);
        matROT[2, 0] = 0;
        matROT[2, 1] = -Mathf.Sin(phi);
        matROT[2, 2] = Mathf.Cos(phi);

        // Position-matrix
        float[,] matP = new float[2, 3];
        matP[0,0] = - x_dist;
        matP[0,1] = 1;
        matP[0,2] = 0;
        matP[1,0] = z_dist;
        matP[1,1] = 0;
        matP[1,2] = 1;

        // Constraint-matrix
        float [,] matJ = new float[2, 2];
        matJ[0, 0] = Mathf.Cos(beta);
        matJ[0, 1] = Mathf.Sin(beta);
        matJ[1, 0] = - Mathf.Sin(beta);
        matJ[1, 1] = Mathf.Cos(beta);
        // float [,] matJ = new float[1, 3];
        // matJ[0, 0] = Mathf.Cos(alpha + beta + gamma);
        // matJ[0, 1] = Mathf.Sin(alpha + beta + gamma);
        // matJ[0, 2] = l * Mathf.Sin(beta + gamma);

        // Geometry-matrix
        float[,] matG = new float[1, 2];
        matG[0, 0] = 1/r;
        matG[0, 1] = (Mathf.Tan(gamma))/r;

        float[,] mat = MatrixMultiply(matROT, matVel);
        mat = MatrixMultiply(matP, mat);
        mat = MatrixMultiply(matJ, mat);
        mat = MatrixMultiply(matG, mat);
        vel = mat[0,0];

        // float[,] mat = MatrixMultiply(MatrixMultiply(matJ, matROT), matVel);
        // vel = -1/Mathf.Sin(gamma) * mat[0,0];
        Debug.Log(transform.name + " vel: "+ vel);
        
        cj.targetAngularVelocity = Vector3.right * vel;
        // rb.AddRelativeTorque(Vector3.right * vel * 10, ForceMode.VelocityChange);
        // rb.velocity = new Vector3(vel * Mathf.Cos(phi), 0, vel * Mathf.Sin(phi));
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
        Gizmos.DrawLine(
            rb.transform.position, 
            new Vector3(
            rb.transform.position.x + vel * Mathf.Cos(phi)/10, 
            rb.transform.position.y, 
            rb.transform.position.z + vel * Mathf.Sin(phi)/10)
            );

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
