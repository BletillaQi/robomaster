#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "struct_typedef.h"
#include "Setting.h"
#include "Motor.h"
#include "user_lib.h"
#include "pid.h"

#define AUTO_SHOOT_TIME 30//80
#define OFFINE_AUTO_SHOOT_TIME 30

// 底盘拨盘的模式
typedef enum
{
    SHOOT_NOFORCE = 0x00, // 无力
    SHOOT_MATCH,          // 比赛模式
    SHOOT_VISION,         // 视觉模式
    SHOOT_EC,             // 电控，即按下扳机发射子弹
} ShootAmmoMode_e;

typedef enum
{
    SR_NO = 0x00, // 无遥控器控制
    SR_YES,       // 有遥控器控制
} ShootRcMode_e;

typedef enum
{
    AMMO_NOFORCE = 0x00, // 无力
    AMMO_NOSPEED,        // 速度为0
    AMMO_NORMALSPEED,    // 正常速度
} AmmoMode_e;

typedef enum
{
    ROTOR_NOFORCE = 0x00, // 无力
    ROTOR_STOP,           // 停止
    ROTOR_FORWORD,        // 前转
    ROTOR_BACKWORD,       // 反转
} RotorMode_e;

typedef struct
{
    fp32 Left;
    fp32 Right;
    fp32 Rotor;
} ShootCommand_t;

typedef struct
{
    int16_t Left;
    int16_t Right;
    int16_t Rotor;
} ShootOutput_t;

typedef struct
{
    pid_type_def Left;
    pid_type_def Right;
    pid_type_def Rotor;
} ShootPID_t;

typedef struct
{
    ShootCommand_t Command;
    ShootOutput_t Output;
    ShootPID_t PID;
    ShootMotorMeasure_t MotorMeasure;

    ShootAmmoMode_e ShootMode;
    ShootRcMode_e RcMode;
    AmmoMode_e AmmoMode;
    RotorMode_e RotorMode;

} Shoot_t;

/**
 * @brief          Shoot任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
extern void Shoot_Task(void const *pvParameters);

/**
 * @brief          Shoot结构体初始化
 * @param[in]      none
 * @retval         none
 */
extern void Shoot_Init(void);

/**
 * @brief          发射机构模式判断
 * @param[in]      none
 * @retval         none
 */
extern void Shoot_State_Update(void);

/**
 * @brief          发射机构是否有遥控器控制
 * @param[in]      none
 * @retval         none
 */
extern void Shoot_Rc_State_Update(void);

/**
 * @brief          摩擦轮模式判断
 * @param[in]      none
 * @retval         none
 */
extern void Ammo_State_Update(void);

/**
 * @brief          拨盘模式判断
 * @param[in]      none
 * @retval         none
 */
extern void Rotor_State_Update(void);

/**
 * @brief          发射机构数据更新
 * @param[in]      none
 * @retval         none
 */
extern void Shoot_Command_Update(void);

/**
 * @brief          摩擦轮PID重装载
 * @param[in]      none
 * @retval         none
 */
extern void Ammo_PID_Update(void);

/**
 * @brief          拨盘PID重装载
 * @param[in]      none
 * @retval         none
 */
extern void Rotor_PID_Update(void);

/**
 * @brief          获取摩擦轮电机控制数据
 * @param[in]      摩擦轮数据指针
 * @retval         none
 */
extern void get_shoot_output_data(ShootOutput_t *MotorOutput);

/**
 * @brief          限制子弹射速
 * @param[in]      none
 * @retval         none
 */
extern void bullet_speed_limit(void);

#endif