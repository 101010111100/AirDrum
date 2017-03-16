/**
  ******************************************************************************
  * @file    PeakCounter.c
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
#include "PeakCounter.h"
//保存数据最大值（在一次谷~峰~谷数据内）
int32_t max = 0;
//数据计数，一次谷~峰~谷必需在指定样本数量内完成
uint32_t counter = 0;
uint32_t rCounter = 0;
uint32_t fCounter = 0;
uint32_t oldSample = 0;
//sample当前样本
//minD = 两点间的最小差值，d值越大要求上升下降速度越快
//minCount 当两点差值大于d时counter++，如果counter少于minCount说明上升和下降时间不够
//maxSampleCount一次谷~峰~谷允许的最大样本数量
uint8_t PeakCounter(int32_t sample,uint32_t minD,uint32_t minCount,uint32_t maxSampleCount)
{
	int32_t d = sample - oldSample;
	//在指定样本数量内（maxSampleCount）没有检测到谷峰谷
	if(counter++ > maxSampleCount)
	{
                max = 0;
		counter = 0;
		rCounter = 0;
		fCounter = 0;
	}
	//上升
	if(d > minD && sample > max)
	{
		//保存最大值
		max = sample;
		//上升计数
		rCounter++;
		//清除下降计数
		fCounter = 0;
	}else if(d < -100)//下降
	{
		fCounter++;
		//如果下降满足条件判断上升是否满足条件，如果下降不满足条件还可能会继续上升
		//所以下降不满足条件前不判断上升
		if(fCounter >= minCount)
		{
			//判断如果上升满足条件
			if(rCounter >= minCount)
			{
                                max = 0;
				counter = 0;
				rCounter = 0;
				fCounter = 0;
				//返回计数
				return 1;
			}else
			{	//上升不满足条件，重头开始	
                                max = 0;
				counter = 0;
				rCounter = 0;
				fCounter = 0;
			}
		}
		//下降不满足条件时不处理
		//如果一直不满足条件最后由if(counter++ > maxSampleCount)重置
	}
        else
        {
          
        }
        oldSample = sample;
	//不满足条件时返回0
	return 0;
}