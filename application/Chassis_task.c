#include "Chassis_task.h"
#include "cmsis_os.h"
pid_struct_t motor_pid_chassis[4];
motor_info_t  motor_info_chassis[8];
 fp32 chassis_motor_pid [3]={30,0.5,10};   //用的原来的pid
volatile int16_t Vx=0,Vy=0,Wz=0;
volatile int16_t motor_speed_target[4];
volatile uint8_t slow_down_flag=0;
volatile uint8_t zero_current_mode_flag=0;
 extern RC_ctrl_t rc_ctrl;

   void Chassis_task(void const *pvParameters)
{
       
       for (uint8_t i = 0; i < 4; i++)
	{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 16384, 16384); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	}
  

    for(;;)
    { 
            RC_to_Vector();                          //遥控器信息转换为底盘速度Vy,Vx,Wz  Remote controller information converted to chassis speed Vy, Vx, Wz
            chassis_motol_speed_calculate();         //电机速度计算，即麦轮运动解算      Calculation of Mecanum Wheel Motion
            Motor_Speed_limiting(motor_speed_target);//限制最大速度                     limit maximum speed
            chassis_current_give() ;                 //发送电流                         send current
            osDelay(1);

    }



}

void RC_to_Vector()
{
    if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1)
    
    {
         zero_current_mode_flag =0;
	    if((rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50))
		{
            slow_down_flag=1;
			for(int i=0;i<4;i++)//减速  slow_down
			{
                 
				if(motor_info_chassis[i].rotor_speed>720||motor_info_chassis[i].rotor_speed<-720)
				{
					motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i],  motor_info_chassis[i].rotor_speed, 0);
				}
				else
				{
					motor_info_chassis[i].set_current=0;
				}
			}
     set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
		}
    
    else
    {
        slow_down_flag = 0;
        Vy= rc_ctrl.rc.ch[3]/660.0*8000;
        Vx= rc_ctrl.rc.ch[2]/660.0*8000;
        Wz= rc_ctrl.rc.ch[4]/660.0*8000;
    }

    }

    else
    {
        zero_current_mode_flag =1;
        motor_info_chassis[0].set_current=0;
        motor_info_chassis[1].set_current=0;
		    motor_info_chassis[2].set_current=0;
		    motor_info_chassis[3].set_current=0;
        set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
    }

 }

void chassis_motol_speed_calculate()
{
    motor_speed_target[CHAS_LF] =  Vy+Vx+Wz;
    motor_speed_target[CHAS_RF] =  Vx-Vy+Wz;
    motor_speed_target[CHAS_RB] =  -Vy-Vx+Wz; 
    motor_speed_target[CHAS_LB] =  Vy-Vx+Wz;
}

  void Motor_Speed_limiting(volatile int16_t *motor_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = 2000;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}

void chassis_current_give() 
{
    uint8_t i=0;
    if((! slow_down_flag )&& (! zero_current_mode_flag))
    {
        
    for(i=0 ; i<4; i++)
    {
        motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }
    	set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
    }

}
