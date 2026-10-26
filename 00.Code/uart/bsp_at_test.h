/**
 *******************************************************************************
 * @file    bsp_at_test.h
 * @author  杨镒铭
 * @version V1.5.0
 * @date    2026-10-20
 * @brief   志翔IMU AT指令逐条验证器对外接口(由main.c统一入口调用)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.5.0<td>2026-10-20<td>杨镒铭<td>创建: 单程序整合版对外运行接口
 * </table>
 *******************************************************************************
 */

#ifndef BSP_AT_TEST_H
#define BSP_AT_TEST_H

/**
 * @brief AT指令逐条验证(依据协议文档第2章)
 *
 * @param pDev    串口设备路径(NULL=默认/dev/ttyUSB0)
 * @param Baud    波特率(<=0=默认921600)
 * @param RunRisk 0=仅安全组 1=安全组+风险组(改配置自动还原)
 *
 * @return 0=全部通过 1=存在失败 -1=串口打开失败
 */
int BspAtTestRun(const char *pDev, int Baud, int RunRisk);

#endif /* BSP_AT_TEST_H */
