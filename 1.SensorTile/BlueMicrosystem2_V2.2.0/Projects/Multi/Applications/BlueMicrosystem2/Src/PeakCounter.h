/**
  ******************************************************************************
  * @file    PeakCounter.h
  * @author  littleshrimp
  * @version V1.0.0
  * @date    16-March-2017
  * @brief   peak detection
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy;</center></h2>
  *
  * 通过给定样本和条件计算波峰
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
//sample当前样本
//minD = 两点间的最小差值，d值越大要求上升下降速度越快
//minCount 当两点差值大于d时counter++，如果counter少于minCount说明上升和下降时间不够
//maxSampleCount一次谷~峰~谷允许的最大样本数量
uint8_t PeakCounter(int32_t sample,uint32_t minD,uint32_t minCount,uint32_t maxSampleCount);