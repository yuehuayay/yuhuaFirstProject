#include "../Header/Feature_Alarm.h"

int32_t RCW_Init(RCW_Handle_t* handle, RCW_Param_t* param)
{
    memset((void*)handle, 0, sizeof(RCW_Handle_t));
    memcpy((void*)&(handle->param), (void*)param, sizeof(RCW_Param_t));
    return 0;
}

int32_t BSD_Init(BSD_Handle_t* handle, BSD_Param_t* param)
{
	memset((void*)handle, 0, sizeof(BSD_Handle_t));
	memcpy((void*)&(handle->param), (void*)param, sizeof(BSD_Param_t));
	return 0;
}

int32_t RCW_Run(RCW_Handle_t* handle, Type_VehicleInfo* vehicle, target_infov2_t* obj, uint16_t gnumObjs)
{

        handle->enable = ENABLE;

        if (RCW_ActiveStateUpdate(handle, vehicle) == ACTIVE)
        {
            handle->active = ACTIVE;
            RCW_Func(handle, vehicle, obj, gnumObjs);
        }
        else
        {
            handle->active = INACTIVE;
            handle->warning = WARNING_NONE;
            handle->exitingTime = 0u;
        }

    return 0;
}

uint8_t RCW_ActiveStateUpdate(RCW_Handle_t* handle, Type_VehicleInfo* vehicle)
{
    uint8_t ret = INACTIVE;

#ifndef OFFLINE_ADAS_APP_//lower computer
   

    if (nubGetARSGearStu() == (uint8_t)Gear_D) /*Step 1: Turn on dow function in gear P.*/ //VehicleSpeed.c
    {
        ret = ACTIVE;
    }
	else
	{
		ret = INACTIVE;
	}
#else//upper computer
    ret = ACTIVE;
#endif

    return ret;
}

uint8_t RCW_Func(RCW_Handle_t* handle, Type_VehicleInfo* VehicleInfo, target_infov2_t* obj, uint16_t gnumObjs)
{
    RCW_Param_t* param = (RCW_Param_t*)&handle->param;
    uint32_t idx = 0;
    float closingSpeed = 0;
    uint32_t ID_Of_Obj = 0;
    static int64_t count = 0;
    uint8_t  warningLevel = WARNING_NONE;


    printf("numobj = %d\n", gnumObjs);
        /* On a staight road */
        for (idx = 0u; idx < gnumObjs; idx++)
        {
			ID_Of_Obj = obj[idx].id;
            handle->ObjState[ID_Of_Obj].ttc = -1;
			handle->ObjState[ID_Of_Obj].inAlarmZone = 0u;   //initialize

            closingSpeed = obj[idx].speed;

            /*Alarm area setting*/
            if ((obj[idx].x >= (int16_t)param->alarmZone.x_min)
                && (obj[idx].x <= (int16_t)param->alarmZone.x_max)
                && (obj[idx].y >= (int16_t)param->alarmZone.y_min)
                && (obj[idx].y <= (int16_t)param->alarmZone.y_max))
            {
                handle->ObjState[ID_Of_Obj].inAlarmZone = 1u;
            }

            handle->ObjState[ID_Of_Obj].isWarning = 0u;

            if (handle->ObjState[ID_Of_Obj].inAlarmZone == 1u)
            {
                if (closingSpeed > param->minClosingSpeed)   /* Target vehicle closing speeed */
                {
                        handle->ObjState[ID_Of_Obj].ttc = (obj[idx].y) / closingSpeed;

                        if (handle->ObjState[ID_Of_Obj].ttc <= TTC_TYPE_RCW)
                        {
                            count++;
                            if (count > 1)
                            {
                                handle->ObjState[ID_Of_Obj].isWarning = 1u;
                                warningLevel = WARNING_FIRST;
                                handle->warning = warningLevel;
                            }

                        }
                        else
                        {
                            handle->ObjState[ID_Of_Obj].isWarning = 0u;
                        }
                    
                }
            }

        }
    

    if (warningLevel != WARNING_NONE)
    {
        handle->exitingTime = handle->param.exitTime;
    }
    if (handle->exitingTime != 0u)
    {
        handle->exitingTime--;
        if (handle->exitingTime == 0U)
        {
            count = 0;
        }
    }
    else
    {
        handle->warning = WARNING_NONE;
    }
    return 0u;
}