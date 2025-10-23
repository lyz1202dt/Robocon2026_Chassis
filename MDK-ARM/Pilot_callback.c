#include "Pilot_callback.h"
#include "AutoPilot.h"
#include "ForceChassis.h"
#include "JY61.h"

extern Vector3D exp_pos;
extern JY61_Typedef JY61;
extern Chassis_t chassis;
extern Vector3D exp_vel;
void SetRobotPos_Callback(Vector3D pos)
{
    exp_pos.x = pos.x;
    exp_pos.y = pos.y;
    exp_pos.z = pos.z;
}

void SetRobotVel_Callback(Vector3D vel)
{
    exp_vel.x = vel.x;
    exp_vel.y = vel.y;
    exp_vel.z = vel.z;
}

