using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class standard_wheel_param : MonoBehaviour
{
    public float r, max_vel, jd_position_damper;
    public bool motor;
    // Start is called before the first frame update
    void Start()
    {
        if (motor == true)
        {
            // set configurable joint position damper
            ConfigurableJoint cj = GetComponent<ConfigurableJoint>();
            JointDrive jd = new JointDrive();
            jd.positionSpring = 0;
            jd.positionDamper = jd_position_damper;
            jd.maximumForce = Mathf.Infinity;
            cj.angularXDrive = jd;
        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}
