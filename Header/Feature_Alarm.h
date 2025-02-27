#ifndef FEATURE_ALARM_H
#define FEATURE_ALARM_H

#include "FFTData.h"

#define TTC_TYPE_RCW                      (1.5f)
#define MAXIMUM_FUN_NUMBER_OF_BYTES         128
#define USED_NUMBER_OF_BYTES_BSD             67
#define USED_NUMBER_OF_BYTES_RCW             50
#define RCW_CRC_MAGICNUMBER          0x75245387
#define BSD_CRC_MAGICNUMBER          0x87631374

#define DISABLE                    ((uint8_t)0U)
#define ENABLE                     ((uint8_t)1U)
#define INACTIVE                   ((uint8_t)0U)
#define ACTIVE                     ((uint8_t)1U)
#define STANDBY                    ((uint8_t)2U)
#define OFFLINE_ADAS_APP_          ((uint8_t)1U)

#ifdef __cplusplus
extern "C" {
#endif

    typedef enum {
        Gear_P = 0,
        Gear_D,
        Gear_N,
        Gear_R,
        Gear_S,
    }Type_Gear;

    typedef enum
    {
        WARNING_NONE = 0,
        WARNING_FIRST,
        WARNING_SECONDARY,
        BREATH,
        WARNING_FIRST_COMMUNICATION,
        WARNING_SECONDARY_COMMUNICATION,
        BREATH_CLOSE,
        WARNING_FIRST_COMMUNICATION_CLOSE,
        WARNING_SECONDARY_COMMUNICATION_CLOSE,
    }Type_WarningSts;


    typedef struct
    {
        float nuhw_Speed;
        float nuhw_VehDynYawRate;
        float nuhw_TurnAngle;

        uint8_t  nub_VehDynYawRateDir;
        uint8_t  nub_TurnAngleDir;
        uint8_t  nub_TurnStu;
        uint8_t  nub_GearStu;
        uint8_t  nub_FLDoorStu;
        uint8_t  nub_FRDoorStu;
        uint8_t  nub_RLDoorStu;
        uint8_t  nub_RRDoorStu;
        uint8_t	 nub_TurnLampLeftStu;
        uint8_t	 nub_TurnLampRightStu;
        uint8_t  nub_BrakeLampStu;
    }Type_VehicleInfo;


    typedef struct Zone_tag
    {
        float x_min;
        float x_max;
        float y_min;
        float y_max;
    } Zone_t;

    /*BSD*/
    typedef struct BSD_ObjState_tag
    {
        uint32_t suppressionTime;
        float ttc;

        uint32_t isWarning : 1;
        uint32_t inAlarmZone : 1;
        uint32_t isMoving : 1;
        uint32_t suppressed : 1;
        uint32_t isttc : 1;
        uint32_t reserved : 27;
    }BSD_ObjState_t;
    typedef struct BSD_Param_tag
    {
        uint32_t CRCmagicnumber;	//4*4byte
        Zone_t  alarmZone;			//4*4byte
        Zone_t  misinformerZone;	//4*4byte
        float alarmTargetSpeed;
        float suppressVelSpeed;
        float suppressTime;
        float activeSpeed;
        uint8_t activeMode;
        uint8_t exitTime;
        uint8_t suppressEnable;

        uint8_t reserve[MAXIMUM_FUN_NUMBER_OF_BYTES - USED_NUMBER_OF_BYTES_BSD];//A total of 128 bytes,Adding a new member to a structure reduces the array size by one
    }BSD_Param_t;
    typedef struct BSD_Handle_tag
    {
        uint8_t enable;
        uint8_t active;
        uint8_t warning;
        uint8_t exitingTime;
        uint32_t warningstartcount;

        BSD_Param_t param;
        BSD_ObjState_t ObjState[DF_TRACKTREE_MAX_NUM];
    }BSD_Handle_t;


    typedef struct RCW_Param_tag
    {
        uint32_t CRCmagicnumber;		  	 //4*4byte
        Zone_t  alarmZone;					//4*4byte
        float collisionPoint;
        float alarmTTC;
        float minClosingSpeed;
        float activeSpeed;
        uint8_t activeMode;
        uint8_t exitTime;

        uint8_t reserve[MAXIMUM_FUN_NUMBER_OF_BYTES - USED_NUMBER_OF_BYTES_RCW];//A total of 128 bytes,Adding a new member to a structure reduces the array size by one

    }RCW_Param_t;
    typedef struct RCW_ObjState_tag
    {
        uint32_t isWarning : 1;
        uint32_t inCircleZone : 1;
        uint32_t inAlarmZone : 1;
        uint32_t reserved : 29;
        float ttc;
    }RCW_ObjState_t;
    typedef struct RCW_Handle_tag
    {
        uint8_t enable;
        uint8_t active;
        uint8_t warning;
        uint8_t exitingTime;
        RCW_Param_t param;
        RCW_ObjState_t ObjState[DF_TRACKTREE_MAX_NUM];
    }RCW_Handle_t;
    typedef struct RCW_save
    {
        uint8_t idx;
        float ttc;
    }RCW_save_t;
    /*RCW END*/

    typedef struct AppHandle_tag
    {
        BSD_Handle_t BSD_Handle;
        RCW_Handle_t RCW_Handle;
    } AppHandle_t;

    void Functional_alarm(AppHandle_t* appHandle, target_infov2_t* info, uint16_t gnumObjs);
    int32_t RCW_Init(RCW_Handle_t* handle, RCW_Param_t* param);
    uint8_t RCW_ActiveStateUpdate(RCW_Handle_t* handle, Type_VehicleInfo* vehicle);
    int32_t RCW_Run(RCW_Handle_t* handle, Type_VehicleInfo* vehicle, target_infov2_t* obj, uint16_t gnumObjs);
    uint8_t RCW_Func(RCW_Handle_t* handle, Type_VehicleInfo* VehicleInfo, target_infov2_t* obj, uint16_t gnumObjs);

    int32_t BSD_Init(BSD_Handle_t* handle, BSD_Param_t* param);

#ifdef __cplusplus
}
#endif

#endif // FEATURE_ALARM_H