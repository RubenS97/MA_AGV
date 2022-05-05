using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class standard_wheel_controller : MonoBehaviour
{
    Component[] cp_wheels;
    float x_velocity, z_velocity, w_velocity;
    // calculation variables
    float[] x_dist, z_dist, beta, alpha, radius, vel, vel_max, len;
    float phi;

    // constraint matrix H(phi) done with jagged matrix 
    float[][,] matH;
    // wheel velocity matrix 
    float[][,] matP;
    float[][,] matVel;

    // Start is called before the first frame update
    void Start()
    {
        x_velocity = 10;
        z_velocity = 10;
        w_velocity = 10;

        cp_wheels = GetComponentsInChildren<standard_wheel_param>();
        vel_max = new float[cp_wheels.Length];
        radius = new float[cp_wheels.Length];

        // initialisation
        matH = new float[cp_wheels.Length][,];
        matP = new float[cp_wheels.Length][,];
        matVel = new float[cp_wheels.Length][,];
        vel = new float[cp_wheels.Length];
        vel_max = new float[cp_wheels.Length];
        x_dist = new float[cp_wheels.Length];
        z_dist = new float[cp_wheels.Length];
        radius = new float[cp_wheels.Length];
        beta = new float[cp_wheels.Length];
        alpha = new float[cp_wheels.Length];
        len = new float[cp_wheels.Length];

        int wheel_index = 0;
        foreach (standard_wheel_param wheel in cp_wheels)
        {
            matH[wheel_index] = new float[1, 3];
            // matH[wheel_index] = new float[2, 1];
            matP[wheel_index] = new float[2, 1];
            matVel[wheel_index] = new float[2, 1];
            x_dist[wheel_index] = wheel.GetComponent<Transform>().position.x - transform.position.x;
            // parent transform should have same z-component but isnt
            z_dist[wheel_index] = wheel.GetComponent<Transform>().position.z - transform.position.z;
            z_dist[wheel_index] = 0;
            // calculate magnitude(length) of wheel position vector
            len[wheel_index] = Mathf.Sqrt(Mathf.Pow(z_dist[wheel_index], 2) + Mathf.Pow(x_dist[wheel_index], 2));

            alpha[wheel_index] = Mathf.Atan(z_dist[wheel_index] / x_dist[wheel_index]);

            beta[wheel_index] = 0;
            radius[wheel_index] = wheel.GetComponent<standard_wheel_param>().r;
            vel_max[wheel_index] = wheel.GetComponent<standard_wheel_param>().max_vel;
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
        float[,] matXYW = new float[2, 1];
        matXYW[0, 0] = x;
        matXYW[1, 0] = z;
        matXYW[2, 0] = w;

        // Angle of rotation between base and world
        // negative sign because unity measures rotation leftwise and not rightwise
        // (positive rotation results in negative angle)
        phi = Vector3.SignedAngle(transform.forward, Vector3.right, Vector3.up) * Mathf.Deg2Rad;

        // rotation matrix world-->robot
        float[,] matR = new float[3, 3];
        matR[0, 0] = Mathf.Cos(phi);
        matR[0, 1] = Mathf.Sin(phi);
        matR[0, 2] = 0;
        matR[1, 0] = -Mathf.Sin(phi);
        matR[1, 1] = Mathf.Cos(phi);
        matR[1, 2] = 0;
        matR[2, 0] = 0;
        matR[2, 1] = 0;
        matR[2, 2] = 1;

        int wheel_index = 0;
        foreach (standard_wheel_param wheel in cp_wheels)
        {
            matH[wheel_index][0, 0] = -Mathf.Sin(alpha[wheel_index] + beta[wheel_index]);
            matH[wheel_index][0, 1] = Mathf.Cos(alpha[wheel_index] + beta[wheel_index]);
            matH[wheel_index][0, 2] = len[wheel_index] * Mathf.Cos(beta[wheel_index]);

            vel[wheel_index] = -MatrixMultiply(MatrixMultiply(matH[wheel_index], matR), matXYW)[0, 0];



            // matH[wheel_index][0, 0] =
            // w * (
            //     -Mathf.Sin(phi) * z_dist[wheel_index] - Mathf.Cos(phi) * x_dist[wheel_index]
            // );
            // // Debug.Log(transform.name + " \t matH[wheel_index][0, 0]   : \t" + matH[wheel_index][0, 0]);

            // matH[wheel_index][1, 0] =
            // w * (
            //     Mathf.Cos(phi) * z_dist[wheel_index] - Mathf.Sin(phi) * x_dist[wheel_index]
            // );

            // matP[wheel_index] = MatrixAdd(matXY, matH[wheel_index]);

            // // convert world-frame motion to robot-frame
            // matP[wheel_index] = MatrixMultiply(matR, matP[wheel_index]);

            // // calculate wheel velocity
            // vel[wheel_index] = Mathf.Sqrt(Mathf.Pow(matP[wheel_index][0, 0], 2) + Mathf.Pow(matP[wheel_index][1, 0], 2));

            // // calculate steering angle
            // beta[wheel_index] = Mathf.Atan(matP[wheel_index][1, 0] / matP[wheel_index][0, 0]);

            // // check quadrant of steering angle and correct if nessescary
            // // Q1 no correction required
            // // Q2 no correction required
            // // Q3 beta = beta - 180° 
            // else if (matP[wheel_index][1, 0] < 0 && matP[wheel_index][0, 0] < 0)
            // {
            //     beta[wheel_index] = beta[wheel_index] - Mathf.PI;
            // }
            // // Q4 beta = beta + 360°
            // else if (matP[wheel_index][1, 0] < 0 && matP[wheel_index][0, 0] > 0)
            // {
            //     beta[wheel_index] = beta[wheel_index] + 2 * Mathf.PI;
            // }

            // Debug.Log(transform.name + " \t matP[wheel_index][0, 0]   : \t" + matP[wheel_index][0, 0]);

            // // special case reference point p on line z_r = 0
            // if (z_dist[wheel_index] == 0)
            // {
            //     // v
            //     matVel[wheel_index][0, 0] =
            //     Mathf.Cos(phi) * matP[wheel_index][0, 0] +
            //     Mathf.Sin(phi) * matP[wheel_index][1, 0];
            //     // Debug.Log(transform.name + " \t matVel[wheel_index][0, 0]   : \t" + matVel[wheel_index][0, 0]);

            //     //w
            //     matVel[wheel_index][1, 0] =
            //     0;
            // }
            // else
            // {
            //     // v
            //     matVel[wheel_index][0, 0] =
            //     (1 / z_dist[wheel_index]) * (
            //         (z_dist[wheel_index] * Mathf.Cos(phi) - x_dist[wheel_index] * Mathf.Sin(phi)) * matP[wheel_index][0, 0] +
            //         (z_dist[wheel_index] * Mathf.Sin(phi) + x_dist[wheel_index] * Mathf.Cos(phi)) * matP[wheel_index][1, 0]
            //     );

            //     //w
            //     matVel[wheel_index][1, 0] =
            //     -(1 / z_dist[wheel_index]) * (
            //         -Mathf.Sin(phi) * matP[wheel_index][0, 0] +
            //         Mathf.Cos(phi) * matP[wheel_index][1, 0]
            //     );
            // }
            // Debug.Log(transform.name + " \t matVel[wheel_index][0, 0]   : \t" + matVel[wheel_index][0, 0]);

            // set target velocity
            wheel.GetComponent<ConfigurableJoint>().targetAngularVelocity = Vector3.right * matVel[wheel_index][0, 0];
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

        int wheel_index = 0;
        foreach (standard_wheel_param wheel in cp_wheels)
        {
            Gizmos.DrawLine(
                wheel.transform.position,
                new Vector3(
                wheel.transform.position.x + matVel[wheel_index][0, 0] * Mathf.Cos(phi) / 10,
                wheel.transform.position.y,
                wheel.transform.position.z + matVel[wheel_index][0, 0] * Mathf.Sin(phi) / 10)
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
