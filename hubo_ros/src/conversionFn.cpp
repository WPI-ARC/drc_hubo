bool ACHtoHuboState(hubo_state * robot_state, HuboState * msg)
{
    if (robot_state && msg)
    {
        //Copy data from a hubo_state struct used in ACH to
        //the HuboState message used in ROS
        //First, copy the joint values
        for (int i = 0; i < HUBO_JOINT_COUNT; i++)
        {
            //Copy an individual joint
            msg->joints[i].position = robot_state->joint[i].pos;
            msg->joints[i].velocity = robot_state->joint[i].vel;
            msg->joints[i].current = robot_state->joint[i].cur;
            msg->joints[i].temperature = robot_state->joint[i].tmp;
            msg->joints[i].active = (int)robot_state->joint[i].pos;
            msg->joints[i].zeroed = (int)robot_state->joint[i].zeroed;
        }
        //Now, copy the three IMUs
        //IMU "1" the main IMU
        msg->imu.x_acceleration = robot_state->imu[0].a_x;
        msg->imu.y_acceleration = robot_state->imu[0].a_y;
        msg->imu.z_acceleration = robot_state->imu[0].a_z;
        msg->imu.x_rotation = robot_state->imu[0].w_x;
        msg->imu.x_rotation = robot_state->imu[0].w_y;
        msg->imu.x_rotation = robot_state->imu[0].w_z;
        //IMU "2" the left foot IMU
        msg->left_foot.x_acceleration = robot_state->imu[1].a_x;
        msg->left_foot.y_acceleration = robot_state->imu[1].a_y;
        msg->left_foot.z_acceleration = robot_state->imu[1].a_z;
        msg->left_foot.x_rotation = robot_state->imu[1].w_x;
        msg->left_foot.x_rotation = robot_state->imu[1].w_y;
        msg->left_foot.x_rotation = robot_state->imu[1].w_z;
        //IMU "3" the right foot IMU
        msg->right_foot.x_acceleration = robot_state->imu[2].a_x;
        msg->right_foot.y_acceleration = robot_state->imu[2].a_y;
        msg->right_foot.z_acceleration = robot_state->imu[2].a_z;
        msg->right_foot.x_rotation = robot_state->imu[2].w_x;
        msg->right_foot.x_rotation = robot_state->imu[2].w_y;
        msg->right_foot.x_rotation = robot_state->imu[2].w_z;
        //Now, copy the four force-torque sensors
        //F-T "1" the left wrist
        msg->left_wrist.Mx = robot_state->ft[0].m_x;
        msg->left_wrist.My = robot_state->ft[0].m_y;
        msg->left_wrist.Fz = robot_state->ft[0].f_z;
        //F-T "2" the right wrist
        msg->right_wrist.Mx = robot_state->ft[1].m_x;
        msg->right_wrist.My = robot_state->ft[1].m_y;
        msg->right_wrist.Fz = robot_state->ft[1].f_z;
        //F-T "3" the left ankle
        msg->left_ankle.Mx = robot_state->ft[2].m_x;
        msg->left_ankle.My = robot_state->ft[2].m_y;
        msg->left_ankle.Fz = robot_state->ft[2].f_z;
        //F-T "4" the right ankle
        msg->right_ankle.Mx = robot_state->ft[3].m_x;
        msg->right_ankle.My = robot_state->ft[3].m_y;
        msg->right_ankle.Fz = robot_state->ft[3].f_z;
        //
        return true;
    }
    else
    {
        return false;
    }
}
