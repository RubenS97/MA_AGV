using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class mecanum_param : MonoBehaviour
{
    // wheel geometry parametes set by user
    public float gamma, beta, r, max_vel, jd_position_damper;
    ConfigurableJoint cj;
    // Start is called before the first frame update
    void Start()
    {
        // set configurable joint position damper
        cj = GetComponent<ConfigurableJoint>();
        JointDrive jd = new JointDrive();
        jd.positionSpring = 0;
        jd.positionDamper = jd_position_damper;
        jd.maximumForce = Mathf.Infinity;
        cj.angularXDrive = jd;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
