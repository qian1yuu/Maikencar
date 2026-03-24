#include <stdint.h>
#include "chassis.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "math.h"
#include "elrs.h"

/* ==================== PID参数配置 ==================== */
#define SPEED_KP        1.0f
#define SPEED_KI        2.0f
#define SPEED_KD        0.5f
#define MAX_PWM         75.0f  // PWM最大输出值（对应100%占空比）
#define MAX_INTEGRAL    50.0f   // 积分限幅，防止积分饱和


/* ==================== 全局变量 ==================== */
static PID_Controller speed_pid;        // PID控制器实例
float target_speed = 0.0f;   // 当前目标速度（带正负方向）
static uint8_t current_pwm = 0;         // 当前实际输出的PWM值

// 目标 PWM（运动学算出来的）
float motor_pwm = 0.0f;
int8_t pwm;

/////
#define S       0.03f   // 轮子半径m
#define A_PLUS_B 0.14f  // a+b 的值（轮子到旋转中心的距离，单位：m



uint8_t PID_Update(void)
{

    float final_pwm = PID_Calculate(&speed_pid, motor_pwm, pwm_v_s);

    // 限幅
    if (final_pwm > 75)  final_pwm = 75;
    if (final_pwm < -75) final_pwm = -75;

    //调试的pwm值
    pwm = (int8_t)final_pwm;

    // 返回绝对值给驱动
    return (uint8_t)fabs(final_pwm);
}










/**
* @brief 底盘初始化
* 初始化PID控制器，设置Kp、Ki、Kd等参数
*/
void Chassis_Init(void)
{
   PID_Init(&speed_pid, SPEED_KP, SPEED_KI, SPEED_KD, MAX_PWM, MAX_INTEGRAL);
}


// uint8_t PID_Update(void){
//    // PID计算：输入目标速度和实际速度，返回调整量
//
//     // motor_pwm来自chassis.c，是当前输出的PWM值
//
//    float pid_out = PID_Calculate(&speed_pid, motor_pwm, pwm_v_s);
//
//
//     // 速度平滑
//     // static float last_pwm = 0.0f;
//     // pwm = 0.8f * last_pwm + 0.2f * pid_out;
//     // last_pwm = pwm;
//
//
//
//    // 基础PWM（根据目标速度开环计算）+ PID调整量
//     pwm = pid_out;
//
//    // PWM限幅，确保在0-100之间
//    if (pwm > 75) pwm = 75;
//    if (pwm < 0) pwm = 0;
//
//     return (uint8_t)pwm;
// }




//////////////////////////
///
///2
///2
///2
void Chassis_Task(uint8_t ctrl_enable, float vx, float vy, float vw){
    // 步骤1：运动学算目标占空比 motor_pwm（你的核心目标值）
    kinematics_forward(vx, vy, vw);

    if (ctrl_enable == 0) {
        Mecanum_Stop();
        return;
    }

    // 步骤2：PID计算最终占空比（motor_pwm是目标，pwm_v_s是实际）
    uint8_t duty  = PID_Update()/6; // duty是占空比绝对值，pwm是带正负的最终值

    // 步骤3：方向由vx/vy/vw决定（不是pwm！），占空比用PID修正后的duty
    if (fabs(vx) > 10.0f) {
        if (vx > 0) { // 摇杆推前进 → 向前
            Mecanum_Move_Forward(duty);
        } else { // 摇杆推后退 → 向后
            Mecanum_Move_Backward(duty);
        }
    } else if (fabs(vy) > 10.0f) {
        if (vy < 0) { // 摇杆推左 → 左移
            Mecanum_Move_Left(duty);
        } else { // 摇杆推右 → 右移
            Mecanum_Move_Right(duty);
        }
    } else if (fabs(vw) > 10.0f) {
        if (vw < 0) { // 摇杆推左转 → 左转
            Mecanum_Rotate_Left(duty);
        } else { // 摇杆推右转 → 右转
            Mecanum_Rotate_Right(duty);
        }
    } else {
        Mecanum_Stop();
    }
}










// void Chassis_Task(uint8_t ctrl_enable, float vx, float vy, float vw){
//     // 控制禁用时直接停止
//     if (ctrl_enable == 0) {
//         Mecanum_Stop();
//         return;
//     }
//
//     // 前进/后退 > 左移/右移 > 旋转
//     if (fabs(vx) > 0.1) {
//         // 前进/后退：直接使用摇杆值作为PWM
//         uint8_t speed = (uint8_t)fabs(vx);
//         if (speed > 75) speed = 75;
//
//         if (vx > 0) {
//             Mecanum_Move_Forward(speed);
//         } else {
//             Mecanum_Move_Backward(speed);
//         }
//     } else if (fabs(vy) > 0.1) {
//         // 左移/右移
//         uint8_t speed = (uint8_t)fabs(vy);
//         if (speed > 75) speed = 75;
//
//         if (vy < 0) {
//             Mecanum_Move_Left(speed);
//         } else {
//             Mecanum_Move_Right(speed);
//         }
//     } else if (fabs(vw) > 0.1) {
//         // 旋转
//         uint8_t speed = (uint8_t)fabs(vw);
//         if (speed > 75) speed = 75;
//
//         if (vw < 0) {
//             Mecanum_Rotate_Left(speed);
//         } else {
//             Mecanum_Rotate_Right(speed);
//         }
//     } else {
//         // 停止
//         Mecanum_Stop();
//     }
// }



// void Chassis_Task(uint8_t ctrl_enable, float vx, float vy, float vw){
//
//     kinematics_forward( vx, vy, vw);//计算四个轮子的角速度和电机pwm,taget
//
//     // 控制禁用时直接停止
//     if (ctrl_enable == 0) {
//         Mecanum_Stop();
//         return;
//     }
//
//
//     // 前进/后退 > 左移/右移 > 旋转
//     if (fabs(vx) > 0.1) {
//         // 前进/后退
//         if (vx > 0) {
//             Mecanum_Move_Forward(pwm);
//         } else {
//             Mecanum_Move_Backward(pwm);
//         }
//     } else if (fabs(vy) > 0.1) {
//         // 左移/右移
//         if (vy < 0) {
//             Mecanum_Move_Left(pwm);
//         } else {
//             Mecanum_Move_Right(pwm);
//         }
//     } else if (fabs(vw) > 0.1) {
//         // 旋转
//         if (vw < 0) {
//             Mecanum_Rotate_Left(pwm);
//         } else {
//             Mecanum_Rotate_Right(pwm);
//         }
//     } else {
//         // 停止
//         Mecanum_Stop();
//     }
//
// }





// 倒向运动学：根据速度和角速度计算电机PWM值taget
// 输入：vx, vy, vw
// 输出：motor_pwm
void kinematics_forward(float vx, float vy, float vw) {
    // 公式：(1/s) * 矩阵 * [vx, vy, vw]^T = [ω0, ω1, ω2, ω3]^T

    float motor_out[4];
    float omega_out[4];

    // -------------------------- 角速度公式 + 位置标注 --------------------------
    omega_out[0] = (-1 * vx + 1 * vy + A_PLUS_B * vw) / S;  // 左上轮 角速度
    omega_out[1] = (-1 * vx - 1 * vy + A_PLUS_B * vw) / S;  // 左下轮 角速度
    omega_out[2] = (1 * vx - 1 * vy + A_PLUS_B * vw) / S;    // 右下轮 角速度
    omega_out[3] = (1 * vx + 1 * vy + A_PLUS_B * vw) / S;    // 右上轮 角速度

    // -------------------------- 转速公式 + 位置标注 --------------------------
    motor_out[0] = omega_out[0] / 2 / 3.1415926f * 5;  // 左上轮 转速
    motor_out[1] = omega_out[1] / 2 / 3.1415926f * 5;  // 左下轮 转速
    motor_out[2] = omega_out[2] / 2 / 3.1415926f * 5;  // 右下轮 转速
    motor_out[3] = omega_out[3] / 2 / 3.1415926f * 5;  // 右上轮 转速

    motor_pwm = motor_out[0] / 308 * 75; // 计算出来的转速/最大转速*75占空比


    // ====================== 关键修复：限幅 ======================
    // 不让目标超过 ±75，否则PID永远满转
    if(motor_pwm > 75) motor_pwm = 75;
    if(motor_pwm < -75) motor_pwm = -75;
}












/**
* @brief 获取当前底盘状态（用于调试）
* @param target: 输出当前目标速度(m/s)
* @param actual: 输出编码器测得的实际速度(m/s)
* @param pwm: 输出当前PWM值
*/
void Chassis_GetStatus(float* target, float* actual, uint8_t* pwm)
{
   *target = target_speed;   // 当前设定的目标速度
   *actual = wheel_v;            // 来自Encoder.c的实际速度
   *pwm = current_pwm;           // 当前输出的PWM值
}




