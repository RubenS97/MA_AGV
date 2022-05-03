using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class mecanum_wheel_controller : MonoBehaviour
{
    Component[] cp_wheels;
    // max velocitys for hand-control
    float x_velocity, z_velocity, w_velocity;

    // calculation variables
    float[] x_dist, z_dist, beta, gamma, radius, vel, vel_max;
    float phi;
    // constraint matrix H(phi) done with jagged matrix 
    float[][,] matH;

    // Start is called before the first frame update
    void Start()
    {
        x_velocity = 10;
        z_velocity = 10;
        w_velocity = 10;

        cp_wheels = GetComponentsInChildren<mecanum_param>();
        // Debug.Log(transform.name + " \t wheel.GetComponent<mecanum_param>().length: \t" + cp_wheels.Length);

        // initialisation
        matH = new float[cp_wheels.Length][,];
        vel = new float[cp_wheels.Length]; 
        vel_max = new float[cp_wheels.Length]; 
        x_dist = new float[cp_wheels.Length];
        z_dist = new float[cp_wheels.Length];
        beta = new float[cp_wheels.Length];
        gamma = new float[cp_wheels.Length];
        radius = new float[cp_wheels.Length];

        // calculate first column of H(phi)
        int wheel_index = 0;
        foreach (mecanum_param wheel in cp_wheels)
        {
            matH[wheel_index] = new float[1, 3];
            // Debug.Log(transform.name + " \t child wheel.position.x  : \t" + wheel.GetComponent<Transform>().position.x);
            // Debug.Log(transform.name + " \t transform.position.x   : \t" + transform.position.x);
            // Debug.Log(transform.name + " \t x_dist[wheel_index]   : \t" + x_dist[wheel_index]);
            x_dist[wheel_index] = wheel.GetComponent<Transform>().position.x - transform.position.x;
            z_dist[wheel_index] = wheel.GetComponent<Transform>().position.z - transform.position.z;
            beta[wheel_index] = wheel.GetComponent<mecanum_param>().beta;
            gamma[wheel_index] = wheel.GetComponent<mecanum_param>().gamma;
            radius[wheel_index] = wheel.GetComponent<mecanum_param>().r;
            vel_max[wheel_index] = wheel.GetComponent<mecanum_param>().max_vel;
            Debug.Log(transform.name + " \t maxvel \t" + wheel.GetComponent<mecanum_param>().max_vel);

            // Debug.Log(transform.name + " \t matH[wheel_index][0, 0]   : \t" + matH[wheel_index][0, 0]);
            matH[wheel_index][0, 0] =
            (1 / (radius[wheel_index] * Mathf.Cos(gamma[wheel_index]))) *
            z_dist[wheel_index] * Mathf.Sin(beta[wheel_index] + gamma[wheel_index]) -
            x_dist[wheel_index] * Mathf.Cos(beta[wheel_index] + gamma[wheel_index]);
            // Debug.Log(transform.name + " matH() \t" + matH[wheel_index][0, 0]);
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

        // calculate wheel velocity
        int wheel_index = 0;
        foreach (mecanum_param wheel in cp_wheels)
        {
            matH[wheel_index][0, 1] = (1 / (radius[wheel_index] * Mathf.Cos(gamma[wheel_index]))) * Mathf.Cos(beta[wheel_index] + gamma[wheel_index] + phi);
            matH[wheel_index][0, 2] = (1 / (radius[wheel_index] * Mathf.Cos(gamma[wheel_index]))) * Mathf.Sin(beta[wheel_index] + gamma[wheel_index] + phi);

            vel[wheel_index] = MatrixMultiply(matH[wheel_index], matVel)[0, 0];
            // check if calc. velocity is higher than allowed wheel velocity
            // calculate factor to limit velocity
            // problem left: when velocity gets limited by its angular componenten (H(phi) column 1) 
            // then the AGV mostly turns because most of the calculated velocity is used for turning
            // and the already small portion used for driving is deminished even more
            // EG: h(phi) = (10, ,1 ,1), k = 0.3 --> h(phi) = (3.33, 0.333, 0.333) 
            // problem 2: the velocity of the wheels before the velocity violation arent getting their vel changed
            // float v = Mathf.Abs(vel[wheel_index]);
            // if (wheel.GetComponent<mecanum_param>().max_vel < v)
            // {
            //     k = wheel.GetComponent<mecanum_param>().max_vel/v;
            // }
            wheel_index++;
        }  

        // check if calc. velocity is higher than allowed wheel velocity
        // calculate factor to limit velocity
        wheel_index = 0;
        float k = 1;
        foreach (mecanum_param wheel in cp_wheels) 
        {
            float v = Mathf.Abs(vel[wheel_index]);
            if (k * v > vel_max[wheel_index])
            {
                k = vel_max[wheel_index]/v;
                Debug.Log(transform.name + " \t k: \t" + k);
            }
            wheel_index++;
        }

        // set target velocity
        wheel_index = 0;
        foreach (mecanum_param wheel in cp_wheels)
        {
            wheel.GetComponent<ConfigurableJoint>().targetAngularVelocity = Vector3.right * k * vel[wheel_index];
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
        foreach (mecanum_param wheel in cp_wheels)
        {
            Gizmos.DrawLine(
                wheel.transform.position,
                new Vector3(
                wheel.transform.position.x + vel[wheel_index] * Mathf.Cos(phi) / 10,
                wheel.transform.position.y,
                wheel.transform.position.z + vel[wheel_index] * Mathf.Sin(phi) / 10)
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
