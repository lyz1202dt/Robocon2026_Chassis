#include "slope.h"

/*
@brief 电机斜坡控制
@param 目标速度
@param 当前状态下电机需要设置的目标速度
@param MoterSlope属性结构体指针（电机斜坡控制句柄）
@retval 无
*/
void MoterSlope(float* target,MoterSlopeSet* slopeSet)
{
  float switch_point;
  float speed_change_rate=slopeSet->speed_change_rate;
  uint8_t type=slopeSet->type;
  
  if(type==3)   //使用S型曲线控制
  {
    if(fabs(slopeSet->last_target-*target)>slopeSet->died_space)    //如果速度差大于死区，则执行计算
    {
      switch_point=(speed_change_rate*speed_change_rate)/(2*slopeSet->alphe);   //对当前加速度归零的过程进行积分，预测何是否需要开始减小加速度
      if(fabs(*target-slopeSet->last_target)<switch_point)    //到达终点的转折点
      {
        if(speed_change_rate>0.0f)
          speed_change_rate=speed_change_rate-slopeSet->alphe;  //到达终点前减小加速度
        else                        //速度变化率减到小�?0，说明此时可以认为其大约位于正确的�?�度上了，直接设置为目标速度
          speed_change_rate=0.0f;
      }
      else if(speed_change_rate<slopeSet->max_change_rate)
        speed_change_rate=speed_change_rate+slopeSet->alphe;  //如果当前加�?�度小于�?大�?�度，又在终点转折点之外，一直加速直到最大�?�度
      
      if(*target>slopeSet->last_target)                              //更新电机当前状�?�下的目标�?�度
        *target=slopeSet->last_target+speed_change_rate;  //加�?�过�?
      else
        *target=slopeSet->last_target-speed_change_rate;  //减�?�过�?
    }
    //else      //误差小于死区，直接将目标赋�?�给当前目标
    //  *target;
  }
  else if (type==2)  //使用指数控制
  {
    if(fabs(*target-slopeSet->last_target)>slopeSet->switch_point)    //还没到转折点，使用线性控�?
      type=1;
    else if(fabs(*target-slopeSet->last_target)>slopeSet->died_space)   //已经到达转折点，未进入死�?,使用指数控制
      *target=slopeSet->last_target+slopeSet->alphe*(*target-slopeSet->last_target);  //为了使线性控制和指数控制平滑过渡，需要使得alphe*switch_point=max_change_rate(省略slopeSet->)
  }

  if (type==1)  //使用线性控制
  {
    float delta_v=*target-slopeSet->last_target;
    if(delta_v>(slopeSet->max_change_rate))     //约束条件
      *target=slopeSet->last_target+slopeSet->max_change_rate;
    else if(delta_v<(-slopeSet->max_change_rate))
      *target=slopeSet->last_target-slopeSet->max_change_rate;
    else
      *target=slopeSet->last_target+delta_v;
  }
  slopeSet->last_target=*target;
}