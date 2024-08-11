/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-05-14 10:22:51
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-14 14:18:27
 * @FilePath: \project\applications\Trajectory_planning.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Trajectory_planning.h"

void (*pCalCurve[])(CurveObjectType *curve) = {0, CalCurveSPTA, CalCurveSPTA};


void mine_plan(CurveObjectType* curve)
{
    if (curve->targetPos > 0)
    {
        MotorVelocityCurve(curve);
    }
    else
    {
        curve->targetPos = -curve->targetPos;
        curve->currentPos = -curve->currentPos;
        MotorVelocityCurve(curve);
        curve->targetPos = -curve->targetPos;
        curve->currentPos = -curve->currentPos;
    }
}


float motor_planning(CurveObjectType *curve)
{
    if (curve->targetPos > 0)
    {
        MotorVelocityCurve(curve);
    }
    else
    {
        curve->targetPos = -curve->targetPos;
        curve->currentPos = -curve->currentPos;
        MotorVelocityCurve(curve);
        curve->targetPos = -curve->targetPos;
        curve->currentPos = -curve->currentPos;
    }
    return curve->currentPos;
}

/* 电机曲线加减速操作-------------------------------------------------------- */
void MotorVelocityCurve(CurveObjectType *curve)
{
    float temp = 0;

    if ((fabs(curve->currentPos - curve->startPos) <= curve->stepPos) && (curve->maxTimes == 0))
    {
        /*自动计算最大时间长度*/
        curve->maxTimes = (int)(((float)fabs(curve->targetPos - curve->startPos) / curve->stepPos) + 1) * curve->intervel;
        curve->aTimes = 0;
    }

    if (curve->aTimes < curve->maxTimes )
    {
        /*单步计算*/
        pCalCurve[curve->curveMode](curve);
        curve->aTimes += curve->intervel;
    }
    else
    {
        curve->currentPos = curve->targetPos;
        curve->maxTimes = 0;
        curve->aTimes = 0;
    }
}

/*S型位置计算*/
static void CalCurveSPTA(CurveObjectType *spta)
{
    float power = 0.0;
    float Pos = 0.0;

    power = (2 * ((float)spta->aTimes) - ((float)spta->maxTimes)) / ((float)spta->maxTimes);
    power = (0.0f - ((float)spta->flexible)) * power;

    Pos = 1 + expf(power);
    Pos = (spta->targetPos - spta->startPos) / Pos;
    spta->currentPos = Pos + spta->startPos;

}