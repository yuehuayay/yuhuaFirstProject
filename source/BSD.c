/*
 * BSD.c
 *
 *  Created on: Oct 12, 2022
 */

#include "../Header/AssisedDriing.h"
#include <math.h>


#define M_PI 3.14159265358979323846


int32_t BSD_Init(BSD_Handle_t* handle, BSD_Param_t* param)
{
	memset((void*)handle, 0, sizeof(BSD_Handle_t));
	memcpy((void*)&(handle->param), (void*)param, sizeof(BSD_Param_t));
	return 0;
}

static uint8_t BSD_ActiveStateUpdate(BSD_Handle_t* handle, Type_VehicleInfo* vehicle)
{
	uint8_t ret = INACTIVE;

#ifndef OFFLINE_ADAS_APP_//lower computer

		if (nubGetARSGearStu() == (uint8_t)Gear_D && (nsdGetARSSpeed() >= handle->param.activeSpeed)) /*Step 1: Turn on bsd function in gear D.*/ //VehicleSpeed.c
		{
			ret = ACTIVE;
		}
		else
		{
			ret = INACTIVE;
		}

		if (ubflgBSDactive == 1u)
		{
			ret = ACTIVE;
		}
		else
		{
			ret = INACTIVE;
		}
#else//upper computer
	ret = ACTIVE;
#endif/*#ifndef OFFLINE_ADAS_APP_*/

	return ret;
}

int32_t BSD_Run(BSD_Handle_t* handle, Type_VehicleInfo* vehicle, target_infov2_t* obj, uint16_t gnumObjs)
{

		handle->enable = ENABLE;

		if (BSD_ActiveStateUpdate(handle, vehicle) == ACTIVE)
		{
			handle->active = ACTIVE;
			BSD_Func(handle, vehicle, obj, gnumObjs);
		}
		else
		{
			handle->active = INACTIVE;
			handle->warning = WARNING_NONE;
			handle->exitingTime = 0u;
		}


	return 0;
}


uint8_t BSD_Func(BSD_Handle_t* handle, Type_VehicleInfo* VehicleInfo, target_infov2_t* obj, uint16_t gnumObjs)
{
	uint32_t idx = 0;
	BSD_Param_t* param = (BSD_Param_t*)&handle->param;
	uint32_t ID_Of_Obj = 0;
	uint8_t  warningLevel = WARNING_NONE;
	static int64_t count = 0;
	uint8_t misinformer_point_num = 0;

	float BSD_point_x_max = -10;
	float BSD_point_x_min = 10;
	uint8_t BSD_point_x_max_id = 0;//BSD_point_x_max ��¼x���᷽����ԶĿ�꣬Ŀ�꿿����ͷ�����ٶ�ʱ������λʹ��
	uint8_t BSD_point_x_min_id = 0; //BSD_point_x_min ��¼x���Ḻ����ԶĿ�꣬Ϊ��β��Զ����Զ�복ͷ�������ٶ�ʱ����λʹ��
	//float BSD_point_v_sum = 0;
	float BSD_point_v_average = 0;


	uint8_t misinformer_zone_true_point_flag = 0;
	float radar_angle_posi_nega = 1;

	uint8_t TurnAngle = 0;
	if (fabs(TurnAngle) >= 45)//todo:Turning optimization
	{
		handle->ObjState[ID_Of_Obj].isMoving = 0u;
		handle->ObjState[ID_Of_Obj].inAlarmZone = 0u;
		handle->ObjState[ID_Of_Obj].isWarning = 0u;
	}

		///* On a staight road */
		//for (idx = 0u; idx < gnumObjs; idx++)//obj->ObjectNum
		//{
		//	//ID_Of_Obj = (obj->ResultDataStArr[idx].ObjectID > DF_TRACKTREE_MAX_NUM - 1) ? (DF_TRACKTREE_MAX_NUM - 1) : obj->ResultDataStArr[idx].ObjectID;
		//	if ((pRdrRepDataSt->OrinPosLongArr[idx] >= param->misinformerZone.x_min)
		//		&& (pRdrRepDataSt->OrinPosLongArr[idx] <= param->misinformerZone.x_max)
		//		&& (pRdrRepDataSt->OrinPosLateArr[idx] >= param->misinformerZone.y_min)
		//		&& (pRdrRepDataSt->OrinPosLateArr[idx] <= param->misinformerZone.y_max))
		//	{
		//		//if(obj->ResultDataStArr[idx].PosLong > 0.5 || obj->ResultDataStArr[idx].PosLong < -0.5)
		//		{
		//			misinformer_point_num += 1;
		//			if (pRdrRepDataSt->OrinPosLongArr[idx] > BSD_point_x_max) // Update the farthest target in the forward direction
		//			{
		//				if ((pRdrRepDataSt->OrinRangeArr[idx] < 5) && (fabs(pRdrRepDataSt->OrinSpeedArr[idx]) > 40))//
		//				{
		//					// Skip, as the target is a speed misinformer, not used, inaccurate angle, RCS not standard, etc.
		//				}
		//				else
		//				{
		//					BSD_point_x_max = pRdrRepDataSt->OrinPosLongArr[idx];//obj->ResultDataStArr[idx].PosLong;
		//					BSD_point_x_max_id = idx;
		//				}
		//			}

		//			if (pRdrRepDataSt->OrinPosLongArr[idx] < BSD_point_x_min) // Update the nearest target in the forward direction
		//			{
		//				if ((pRdrRepDataSt->OrinRangeArr[idx] < 5) && (fabs(pRdrRepDataSt->OrinSpeedArr[idx]) > 40))
		//				{
		//					// Skip, as the target is a speed misinformer, not used, inaccurate angle, RCS not standard, etc.
		//				}
		//				else
		//				{
		//					BSD_point_x_min = pRdrRepDataSt->OrinPosLongArr[idx];
		//					BSD_point_x_min_id = idx;
		//				}
		//			}
		//			//BSD_point_v_sum += pRdrRepDataSt->OrinSpeedArr[idx]/cos(pRdrRepDataSt->OrinAZArr[idx]);
		//			//BSD_misinformer_point_v_list[misinformer_point_num] = obj->ResultDataStArr[idx].VelLong;
		//			//BSD_misinformer_point_id_list[misinformer_point_num] = idx;
		//		}

		//	}
		//}

		//if (fabs(pRdrRepDataSt->OrinAZArr[BSD_point_x_min_id] + radar_angle_posi_nega * M_PI / 2) > M_PI / 18)// Target horizontal angle exceeds 10 degrees, radar_angle_posi_nega * M_PI / 2 makes the angle 0
		//{
		//	BSD_point_v_average = pRdrRepDataSt->OrinSpeedArr[BSD_point_x_min_id] / cos(pRdrRepDataSt->OrinAZArr[BSD_point_x_min_id]);
		//	misinformer_zone_true_point_flag = 1;
		//}
		//else if (fabs(pRdrRepDataSt->OrinAZArr[BSD_point_x_max_id] + radar_angle_posi_nega * M_PI / 2) > M_PI / 18)
		//{
		//	BSD_point_v_average = pRdrRepDataSt->OrinSpeedArr[BSD_point_x_max_id] / cos(pRdrRepDataSt->OrinAZArr[BSD_point_x_max_id]);
		//	misinformer_zone_true_point_flag = 1;
		//}
		//else
		//{
		//	misinformer_zone_true_point_flag = 0;
		//}

		//if (misinformer_zone_true_point_flag == 1)
		//{
		//	for (idx = 0u; idx < obj->ObjectNum; idx++)
		//	{
		//		//ID_Of_Obj = (obj->ResultDataStArr[idx].ObjectID > DF_TRACKTREE_MAX_NUM - 1) ? (DF_TRACKTREE_MAX_NUM - 1) : obj->ResultDataStArr[idx].ObjectID;
		//		if ((obj->ResultDataStArr[idx].PosLong >= param->misinformerZone.x_min)
		//			&& (obj->ResultDataStArr[idx].PosLong <= param->misinformerZone.x_max)
		//			&& (obj->ResultDataStArr[idx].PosLate >= param->misinformerZone.y_min)
		//			&& (obj->ResultDataStArr[idx].PosLate <= param->misinformerZone.y_max))
		//		{
		//			obj->ResultDataStArr[idx].VelLong = BSD_point_v_average;
		//		}
		//	}
		//}
		/*Preprocess 64 targets*/
		for (idx = 0u; idx < gnumObjs; idx++)
		{
			ID_Of_Obj = (obj[idx].id > DF_TRACKTREE_MAX_NUM - 1) ? (DF_TRACKTREE_MAX_NUM - 1) : obj[idx].id;
			handle->ObjState[ID_Of_Obj].inAlarmZone = 0u;
			handle->ObjState[ID_Of_Obj].isMoving = 0u;

#ifndef OFFLINE_ADAS_APP_	//lower computer
			if (((obj->ResultDataStArr[idx].VelLong + nsdGetARSSpeed()) >= param->alarmTargetSpeed)
				/* && (obj->ResultDataStArr[idx].StaticFlag == 0)*/ && (obj->ResultDataStArr[idx].GuardRailFlag == 0)
				&& (obj->ResultDataStArr[idx].RemovableFlag == 1))/*exclude  Static target*/   /*(obj->ResultDataStArr[idx].StaticFlag == 0)*/
#else				
			if (obj[idx].speed < 0  &&  (fabsf(obj[idx].speed)  >= param->alarmTargetSpeed))
#endif	/*#ifndef OFFLINE_ADAS_APP_*/

			{
				handle->ObjState[ID_Of_Obj].isMoving = 1u;
			}

			/*Alarm area setting*/
			if ((obj[idx].x >= (int16_t)param->alarmZone_Middle.x_min)
				&& (obj[idx].x <= (int16_t)param->alarmZone_Middle.x_max)
				&& (obj[idx].y >= (int16_t)param->alarmZone_Middle.y_min)
				&& (obj[idx].y <= (int16_t)param->alarmZone_Middle.y_max))
			{
				handle->ObjState[ID_Of_Obj].inAlarmZone = 1u;
			}
			else if ((obj[idx].x >= (int16_t)param->alarmZone_Left.x_min)
				&& (obj[idx].x <= (int16_t)param->alarmZone_Left.x_max)
				&& (obj[idx].y >= (int16_t)param->alarmZone_Left.y_min)
				&& (obj[idx].y <= (int16_t)param->alarmZone_Left.y_max))
			{
				handle->ObjState[ID_Of_Obj].inAlarmZone = 2u;
			}
			else if ((obj[idx].x >= (int16_t)param->alarmZone_Right.x_min)
				&& (obj[idx].x <= (int16_t)param->alarmZone_Right.x_max)
				&& (obj[idx].y >= (int16_t)param->alarmZone_Right.y_min)
				&& (obj[idx].y <= (int16_t)param->alarmZone_Right.y_max))
			{
				handle->ObjState[ID_Of_Obj].inAlarmZone = 3u;
			}
			else
			{
				handle->ObjState[ID_Of_Obj].inAlarmZone = 0u;
			}



			if ((handle->ObjState[ID_Of_Obj].inAlarmZone != 0u)
				&& (handle->ObjState[ID_Of_Obj].isMoving == 1u)
				/*&& (obj->ResultDataStArr[idx].ObjRCS >= BSD_RCS_EXCLUDE)*/)//todo:Because there will be an alarm target with a speed of 0 on the railing.*/)
			{
				count++;
				if (count >= 3)
				{
					if (obj[idx].speed > 3)
					{
						if ((handle->ObjState[ID_Of_Obj].inAlarmZone == 1))
						{
							if (obj[idx].speed >5)
							{
								handle->ObjState[ID_Of_Obj].isWarning = WARNING_BSD2_Middle;
								warningLevel = WARNING_BSD2_Middle;
								handle->warning = warningLevel;
							}
							else
							{
								handle->ObjState[ID_Of_Obj].isWarning = WARNING_BSD1_Middle;
								warningLevel = WARNING_BSD1_Middle;
								handle->warning = warningLevel;
							}

						}
						else if (handle->ObjState[ID_Of_Obj].inAlarmZone == 2)
						{
							if (obj[idx].speed > 5)
							{
								handle->ObjState[ID_Of_Obj].isWarning = WARNING_BSD2_Left;
								warningLevel = WARNING_BSD2_Left;
								handle->warning = warningLevel;
							}
							else
							{
								handle->ObjState[ID_Of_Obj].isWarning = WARNING_BSD1_Left;
								warningLevel = WARNING_BSD1_Left;
								handle->warning = warningLevel;
							}
						}
						else if (handle->ObjState[ID_Of_Obj].inAlarmZone == 3)
						{
							if (obj[idx].speed > 5)
							{
								handle->ObjState[ID_Of_Obj].isWarning = WARNING_BSD2_Right;
								warningLevel = WARNING_BSD2_Right;
								handle->warning = warningLevel;
							}
							else
							{
								handle->ObjState[ID_Of_Obj].isWarning = WARNING_BSD1_Right;
								warningLevel = WARNING_BSD1_Right;
								handle->warning = warningLevel;

							}
						}

					}
				}
			}
		else
		{
			handle->ObjState[ID_Of_Obj].isWarning = 0u;
		}

	}/*	for (idx = 0u; idx < obj->ObjectNum; idx++)*/

	if (warningLevel != WARNING_NONE)
	{
		handle->warningstartcount++;
		if (handle->warningstartcount > 6)
		{
			handle->warningstartcount = 6;
		}
	}
	else
	{
		if (0 == handle->warningstartcount)
		{
			handle->warningstartcount = 0;
		}
		else
		{
			handle->warningstartcount--;
		}

	}

	if ((warningLevel != WARNING_NONE) && (handle->warningstartcount >= 2))
	{
		handle->exitingTime = handle->param.exitTime;
	}
	else
	{
		if (handle->exitingTime != 0u)
		{
			handle->exitingTime--;
			if (handle->exitingTime == 0U)
			{
				count = 0;
			}

		}
	}
	     return 1u;
}


