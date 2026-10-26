/**
 *******************************************************************************
 * @file    bsp_can_test.h
 * @author  杨镒铭
 * @version V1.0.0
 * @date    2026-10-21
 * @brief   志翔IMU CAN交互协议逐指令验证器(由main.c统一入口调用)
 *******************************************************************************
 */

#ifndef __BSP_CAN_TEST_H__
#define __BSP_CAN_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief  CAN指令验证器入口(依据协议文档3.3.3章, 帧格式逐ID对照固件核实)
     *
     * @param  pIfName  CAN接口名(如"can0")
     * @param  RunRisk  0=仅安全组(纯查询+无副作用动作) 1=安全组+风险组(RAM写入+自动还原)
     *
     * @return 0=全部通过 1=存在失败 -1=接口打开失败
     */
    int BspCanTestRun(const char *pIfName, int RunRisk);

#ifdef __cplusplus
}
#endif

#endif
