#ifndef __IMU_H
#define __IMU_H

#include "stm32f10x.h"
#include "math.h"
#include "mpu6050.h"
			
			typedef struct{
				float AX;
				float AY;
				float AZ;
				float GX;
				float GY;
				float GZ;
							}param_imu;
			
		    typedef struct{
				float Pitch;
				float Roll;
				float Yaw;
							}param_Angle;
			
			extern param_Angle imu_Angle;
							
							
void IMU_getEuleranAngles(void);
							


#endif

