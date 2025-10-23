#include "Pilot_callback.h"
#include "AutoPilot.h"

extern Vector3D exp_pos;

void SetRobotPos_Callback(Vector3D pos)
{
    exp_pos.x = pos.x;
    exp_pos.y = pos.y;
    exp_pos.z = pos.z;
}

void SetRobotVel_Callback(Vector3D vel)
{
    
}
