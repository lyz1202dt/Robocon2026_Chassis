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

void SetRobotAcc_Callback(Vector3D acc)
{
    chassis.exp_acc.x = acc.x;
    chassis.exp_acc.y = acc.y;
    chassis.exp_acc.z = acc.z;
}

void Finished_Callback(AutopilotState state,AutoPilotReq_t *req,void* user_data)
{
    if (state == AUTOPILOT_STAGE_FINISH) {
        // 自动驾驶仪到达终点后的处理
        
    } else if (state == AUTOPILOT_STAGE_ERROR) {
        // 自动驾驶仪错误处理
    }
}
