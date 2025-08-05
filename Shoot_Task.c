#include "Shoot_Task.h"
#include "cmsis_os.h"
#include "Interrupt_Service.h"

#include "FreeRTOS_setting.h"
#include "Setting.h"
#include "PID_Shoot_setting.h"

#include "Remote.h"
#include "Offline.h"
#include "AimbotCan.h"
#include "Usb.h"
#include "user_lib.h"

#include "RefereeCan.h"
#include "math.h"

#include PARAMETER_FILE
#include KEYMAP_FILE

Shoot_t Shoot;              // 发射机构结构体
GimbalRc_t Remote_S;        // 遥控器数据
OfflineMonitor_t Offline_S; // 离线检测结构体
AimbotFrame_SCM_t Aimbot_S;   // 自瞄数据


uint32_t shoot_init_time = 0;
uint32_t rotor_init_time = 0;


fp32 Real_Ammo_Speed = AMMO_SPEEDSET_30MS;

AmmoMode_e Last_Ammo_Mode = AMMO_NOFORCE;
RotorMode_e Last_Rotor_Mode = ROTOR_NOFORCE;

extern RefereeInformation_t RefereeInformation;
uint8_t Ammo0Speed_update_flag = 0;

uint32_t rotormode_time = 0;
uint32_t debug_rotormode_time = 0;
uint8_t firetime_cutdown = 0;
int16_t rotor_one_time = ROTOR_ONE_TIME;


/**
 * @brief          Shoot任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void Shoot_Task(void const *pvParameters)
{
    osDelay(SHOOT_TASK_INIT_TIME);

    Shoot_Init();

    while (1)
    {
        Remote_S = *get_remote_control_point();  // 更新遥控器数据
        Offline_S = *get_device_offline_point(); // 更新离线保护数据
        Aimbot_S = *get_usb_aimbot_command_point();  // 更新自瞄数据

        Shoot_State_Update();
        Shoot_Rc_State_Update();
        Ammo_State_Update();
        Rotor_State_Update();
        Ammo_PID_Update();
        Rotor_PID_Update();
        Shoot_Motor_Measure_Update(&Shoot.MotorMeasure);

        xSemaphoreTake(ShootMotorMutexHandle, osWaitForever);
        Shoot_Command_Update();
        xSemaphoreGive(ShootMotorMutexHandle);

        osDelay(1);
    }
}

/**
 * @brief          Shoot结构体初始化
 * @param[in]      none
 * @retval         none
 */
void Shoot_Init(void)
{
    PID_init(&Shoot.PID.Left,
             PID_POSITION,
             AMMO_LEFT_SPEED_0MS,
             M3508_MAX_OUTPUT,
             M3508_MAX_IOUTPUT);
    PID_init(&Shoot.PID.Right,
             PID_POSITION,
             AMMO_RIGHT_SPEED_0MS,
             M3508_MAX_OUTPUT,
             M3508_MAX_IOUTPUT);
    PID_init(&Shoot.PID.Rotor,
             PID_POSITION,
             ROTOR_LIST[Shoot.RotorMode],
             M2006_MAX_OUTPUT,
             M2006_MAX_IOUTPUT);
    Shoot.Output.Left = 0;
    Shoot.Output.Right = 0;
    Shoot.Output.Rotor = 0;
}

/**
 * @brief          发射机构模式判断
 * @param[in]      none
 * @retval         none
 */
void Shoot_State_Update(void)
{
    if (Offline_S.Motor[AMMO_LEFT_MOTOR] == DEVICE_OFFLINE ||
        Offline_S.Motor[AMMO_RIGHT_MOTOR] == DEVICE_OFFLINE ||
        Offline_S.Motor[ROTOR_MOTOR] == DEVICE_OFFLINE)
    {
        Shoot.ShootMode = SHOOT_NOFORCE;
        return;
    }

    switch (Remote_S.s[SF])
    {
    case RC_SW_UP:
        switch (Remote_S.s[SE])
        {
        case RC_SW_UP:
            Shoot.ShootMode = SHOOT_MATCH;
            break;

        case RC_SW_MID:
            Shoot.ShootMode = SHOOT_VISION;
            break;

        case RC_SW_DOWN:
            Shoot.ShootMode = SHOOT_EC;
            break;

        default:
            break;
        }
        break;
	case RC_SW_MID:
		Shoot.ShootMode = SHOOT_NOFORCE;
    case RC_SW_DOWN:
        Shoot.ShootMode = SHOOT_NOFORCE;
        break;

    default:
        break;
    }
}

/**
 * @brief          发射机构是否有遥控器控制
 * @param[in]      none
 * @retval         none
 */
void Shoot_Rc_State_Update(void)
{
    if (Shoot.ShootMode != SHOOT_MATCH)
    {
        switch (Remote_S.s[SC])
        {
        case RC_SW_UP:
            Shoot.RcMode = SR_YES;
            break;

        case RC_SW_MID:
            Shoot.RcMode = SR_NO;
            break;

        case RC_SW_DOWN:
            Shoot.ShootMode = SHOOT_NOFORCE;
            break;

        default:
            break;
        }
    }
}

/**
 * @brief          摩擦轮模式判断
 * @param[in]      none
 * @retval         none
 */
void Ammo_State_Update(void)
{
    Last_Ammo_Mode = Shoot.AmmoMode;
    switch (Shoot.ShootMode)
    {
    case SHOOT_NOFORCE:
        Shoot.AmmoMode = AMMO_NOFORCE;
        break;

    case SHOOT_MATCH:
        Shoot.AmmoMode = AMMO_NORMALSPEED;
        break;

    default:
        switch (Remote_S.s[SB])
        {
        case RC_SW_UP:
            Shoot.AmmoMode = AMMO_NOFORCE;
            break;

        case RC_SW_MID:
            Shoot.AmmoMode = AMMO_NOFORCE;
            break;

        case RC_SW_DOWN:
            Shoot.AmmoMode = AMMO_NORMALSPEED;
            break;
        }
        break;
    }

    if (Shoot.AmmoMode == AMMO_NOFORCE && Last_Ammo_Mode != AMMO_NOFORCE)
    {
        if (Last_Ammo_Mode == AMMO_NOSPEED)
        {
            if ((GetSystemTimer() - shoot_init_time) > SHOOT_STOP_TIME)
            {
                Shoot.AmmoMode = AMMO_NOFORCE;
            }
            else
            {
                Shoot.AmmoMode = AMMO_NOSPEED;
            }
        }
        else
        {
            Shoot.AmmoMode = AMMO_NOSPEED;
            shoot_init_time = GetSystemTimer();
        }
    }
}

/**
 * @brief          拨盘模式判断
 * @param[in]      none
 * @retval         none
 */
void Rotor_State_Update(void)
{
    Last_Rotor_Mode = Shoot.RotorMode;
    switch (Shoot.ShootMode)
    {
    case SHOOT_NOFORCE:
        Shoot.RotorMode = ROTOR_NOFORCE;
        break;

    case SHOOT_MATCH:
		
	//	Shoot.RotorMode = ROTOR_NOFORCE;
		if(Offline_S.RefereeAmmoLimitNode0 == 0)
 {
        if (Aimbot_S.AimbotState & AIMBOT_SHOOT_REQUEST_OFFSET)
        {
            Shoot.RotorMode = ROTOR_FORWORD;
			rotormode_time = GetSystemTimer();
			if(RefereeInformation.Realtime.Ammo0Heat >= 350)
			{
				Shoot.RotorMode = ROTOR_STOP;
			}
			
        }
		else
		{
			if( Last_Rotor_Mode == ROTOR_FORWORD && Is_Time_Exceeded(rotormode_time, AUTO_SHOOT_TIME))
			{
				Shoot.RotorMode = ROTOR_STOP;
			}
			else if( Last_Rotor_Mode == ROTOR_STOP || Last_Rotor_Mode == ROTOR_NOFORCE )
			{
				Shoot.RotorMode = ROTOR_STOP;
				rotormode_time = GetSystemTimer();
			}
		}
 }
 else
 {
	 if((Aimbot.AimbotState & AIMBOT_SHOOT_REQUEST_OFFSET) && (firetime_cutdown <= 0))
	 {
		 Shoot.RotorMode = ROTOR_FORWORD;
		 firetime_cutdown = 80;
		 rotormode_time = GetSystemTimer();
	 }
	 else
	 {
		 if( Last_Rotor_Mode == ROTOR_FORWORD && Is_Time_Exceeded(rotormode_time, OFFINE_AUTO_SHOOT_TIME))
			{
				Shoot.RotorMode = ROTOR_STOP;
			}
			else if( Last_Rotor_Mode == ROTOR_STOP || Last_Rotor_Mode == ROTOR_NOFORCE )
			{
				Shoot.RotorMode = ROTOR_STOP;
				rotormode_time = GetSystemTimer();
			}
	 }
	 firetime_cutdown--;
 }
		
        break;

    case SHOOT_VISION:
 //       Shoot.RotorMode = ROTOR_STOP;
		if(Aimbot_S.AimbotState & AIMBOT_SHOOT_REQUEST_OFFSET)
		{
        if (switch_is_down(Remote_S.s[SB]) && switch_is_down(Remote_S.s[SH]) && Aimbot_S.AimbotState & AIMBOT_SHOOT_REQUEST_OFFSET)
        {
            Shoot.RotorMode = ROTOR_FORWORD;
					debug_rotormode_time = GetSystemTimer();
        }
		else
		{
			Shoot.RotorMode = ROTOR_STOP;
		}
		}
		else
		{
			if( Last_Rotor_Mode == ROTOR_FORWORD && Is_Time_Exceeded(debug_rotormode_time, AUTO_SHOOT_TIME))
			{
				Shoot.RotorMode = ROTOR_STOP;
			}
			else if( Last_Rotor_Mode == ROTOR_STOP || Last_Rotor_Mode == ROTOR_NOFORCE )
			{
				Shoot.RotorMode = ROTOR_STOP;
			}
		}
			
        break;

    case SHOOT_EC:
        switch (Remote_S.s[SB])
        {
        case RC_SW_UP:
            Shoot.RotorMode = ROTOR_STOP;
        //    if (switch_is_down(Remote_S.s[SH]))
         //   {
           //     Shoot.RotorMode = ROTOR_BACKWORD;
            //}
            break;

        case RC_SW_MID:
            Shoot.RotorMode = ROTOR_NOFORCE;
            break;

        case RC_SW_DOWN:
            Shoot.RotorMode = ROTOR_STOP;
            if (switch_is_down(Remote_S.s[SH]))
            {
                Shoot.RotorMode = ROTOR_FORWORD;
				if(RefereeInformation.Realtime.Ammo0Heat >= 350 && Offline_S.RefereePowerHeatNode0 ==0)
			{
				Shoot.RotorMode = ROTOR_STOP;
			}
            }
            break;
        }
        break;

    default:
        break;
    }

//    if (Shoot.RotorMode == ROTOR_NOFORCE && Last_Rotor_Mode != ROTOR_NOFORCE)
//    {
//        if (Last_Rotor_Mode == ROTOR_STOP)
//        {
//            if ((GetSystemTimer() - rotor_init_time) > ROTOR_STOP_TIME)
//            {
//                Shoot.RotorMode = ROTOR_NOFORCE;
//            }
//            else
//            {
//                Shoot.RotorMode = ROTOR_STOP;
//            }
//        }
//        else
//        {
//            Shoot.RotorMode = ROTOR_STOP;
//            rotor_init_time = GetSystemTimer();
//        }
//    }
}

/**
 * @brief          摩擦轮PID重装载
 * @param[in]      none
 * @retval         none
 */
void Ammo_PID_Update(void)
{
    if (Last_Ammo_Mode == Shoot.AmmoMode)
    {
        return;
    }

    PID_init(&Shoot.PID.Left,
             PID_POSITION,
             AMMO_LEFT_LIST[Shoot.AmmoMode],
             M3508_MAX_OUTPUT,
             M3508_MAX_IOUTPUT);
    PID_init(&Shoot.PID.Right,
             PID_POSITION,
             AMMO_RIGHT_LIST[Shoot.AmmoMode],
             M3508_MAX_OUTPUT,
             M3508_MAX_IOUTPUT);
}

/**
 * @brief          拨盘PID重装载
 * @param[in]      none
 * @retval         none
 */
void Rotor_PID_Update(void)
{
    if (Last_Rotor_Mode == Shoot.RotorMode)
    {
        return;
    }

    PID_init(&Shoot.PID.Rotor,
             PID_POSITION,
             ROTOR_LIST[Shoot.RotorMode],
             M2006_MAX_OUTPUT,
             M2006_MAX_IOUTPUT);
}

/**
 * @brief          发射机构数据更新
 * @param[in]      none
 * @retval         none
 */
void Shoot_Command_Update(void)
{
    switch (Shoot.AmmoMode)
    {
    case AMMO_NOFORCE:
        Shoot.Command.Left = 0;
        Shoot.Command.Right = 0;
        break;
    case AMMO_NORMALSPEED:
      //  Shoot.Command.Left = AMMO_SPEEDSET_LEFT_30MS * AMMO_LEFT_MOTOR_DIRECTION;
      //  Shoot.Command.Right = AMMO_SPEEDSET_RIGHT_30MS * AMMO_RIGHT_MOTOR_DIRECTION;
		      Shoot.Command.Left = Real_Ammo_Speed * AMMO_LEFT_MOTOR_DIRECTION;
		      Shoot.Command.Right = Real_Ammo_Speed * AMMO_RIGHT_MOTOR_DIRECTION;
        bullet_speed_limit();
        break;
    case AMMO_NOSPEED:
        Shoot.Command.Left = 0;
        Shoot.Command.Right = 0;
        break;
    default:
        break;
    }
    
    switch (Shoot.RotorMode)
    {
    case ROTOR_NOFORCE:
        Shoot.Command.Rotor = 0;
        break;
    case ROTOR_STOP:
        Shoot.Command.Rotor = 0;
        break;
    case ROTOR_FORWORD:
        Shoot.Command.Rotor = ROTOR_SPEEDSET * ROTOR_MOTOR_DIRECTION;
        break;
    case ROTOR_BACKWORD:
        Shoot.Command.Rotor = ROTOR_SPEEDSET * ROTOR_MOTOR_DIRECTION;
        break;
    }
    Shoot.Output.Left = PID_calc(&Shoot.PID.Left, Shoot.MotorMeasure.AmmoLeftMotorSpeed, Shoot.Command.Left);
    Shoot.Output.Right = PID_calc(&Shoot.PID.Right, Shoot.MotorMeasure.AmmoRightMotorSpeed, Shoot.Command.Right);
    Shoot.Output.Rotor = PID_calc(&Shoot.PID.Rotor, Shoot.MotorMeasure.RotorMotorSpeed, Shoot.Command.Rotor);
}

/**
 * @brief          获取摩擦轮电机控制数据
 * @param[in]      摩擦轮数据指针
 * @retval         none
 */
void get_shoot_output_data(ShootOutput_t *MotorOutput)
{
    xSemaphoreTake(ShootMotorMutexHandle, osWaitForever);
    memcpy(MotorOutput, &Shoot.Output, sizeof(ShootOutput_t));
    xSemaphoreGive(ShootMotorMutexHandle);
}

/**
 * @brief          限制子弹射速
 * @param[in]      none
 * @retval         none
 */
void bullet_speed_limit(void)
{
   if(Offline_S.RefereeAmmoSpeedNode0 == 0 && Ammo0Speed_update_flag == 1)
	 {
//		 if(RefereeInformation.Ammo0Speed > bullet_speed_max - 0.5f )
//		 {
//			 Shoot.Command.Left = (fabsf(Shoot.Command.Left) - 50);
//			 Shoot.Command.Right = (fabsf(Shoot.Command.Right) - 50);
//		 }

//		 else if(RefereeInformation.Ammo0Speed < bullet_speed_min + 0.5f)
//		 {
//			 Shoot.Command.Left = (fabsf(Shoot.Command.Left) + 50);
//			 Shoot.Command.Right = (fabsf(Shoot.Command.Right) + 50);
//		 }
		 float Speed_Error = 26 - RefereeInformation.Ammo0Speed;
		 
		 if(Speed_Error <= 2.0f && Speed_Error >= -2.0f)
		 {
			 Real_Ammo_Speed += Speed_Error*100.0f;
		 }
		 else if(Speed_Error > 2.0f || Speed_Error < -2.0f)
		 {
			 Real_Ammo_Speed += Speed_Error*150.0f;
		 }
	 
	// Shoot.Command.Left = fp32_constrain(Shoot.Command.Left, AMMO_SPEEDSET_LEFT_30MS - 600, AMMO_SPEEDSET_LEFT_30MS + 600);
	// Shoot.Command.Right = fp32_constrain(Shoot.Command.Right, AMMO_SPEEDSET_RIGHT_30MS - 600, AMMO_SPEEDSET_RIGHT_30MS + 600);
		 
		 Real_Ammo_Speed = fp32_constrain(Real_Ammo_Speed, AMMO_SPEEDSET_30MS - 1500, AMMO_SPEEDSET_30MS + 1000);
		 
		 Shoot.Command.Left = Real_Ammo_Speed * AMMO_LEFT_MOTOR_DIRECTION;
		 Shoot.Command.Right = Real_Ammo_Speed * AMMO_RIGHT_MOTOR_DIRECTION; 
		 
		 	 Ammo0Speed_update_flag = 0;
	 }
}