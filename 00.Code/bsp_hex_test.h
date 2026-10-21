/**
 *******************************************************************************
 * @file    bsp_hex_test.h
 * @author  杨镒铭
 * @version V1.5.2
 * @date    2026-10-20
 * @brief   志翔IMU HEX交互协议逐报文验证器对外接口(由main.c统一入口调用)
 *
 * @par 修改日志
 * <table>
 * <tr><th>版本<th>日期<th>作者<th>描述
 * <tr><td>V1.5.2<td>2026-10-20<td>杨镒铭<td>创建: 单程序整合版对外运行接口
 * </table>
 *******************************************************************************
 */

#ifndef BSP_HEX_TEST_H
#define BSP_HEX_TEST_H

/**
 * @brief HEX交互协议逐报文验证(依据协议文档3.2章, 覆盖18个报文ID)
 *
 * @param pDev    串口设备路径(NULL=默认/dev/ttyUSB0)
 * @param Baud    波特率(<=0=默认921600)
 * @param RunRisk 0=仅安全组 1=安全组+风险组(写RAM自动还原)
 *
 * @return 0=全部通过 1=存在失败 -1=串口打开失败
 */
int BspHexTestRun(const char *pDev, int Baud, int RunRisk);

#endif /* BSP_HEX_TEST_H */
