#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../Header/senserconfig.h"
#include "../Header/Feature_Alarm.h"
#include <stdbool.h>

AppHandle_t  app_Handle;
extern uint16_t gnumObjs;
extern DPIF_PointCloud target[MAXNUM_OF_DETECTION];
Type_VehicleInfo VehicleInfo;

/*BSD*/
BSD_Param_t BSD_Param_Right =
{
    .CRCmagicnumber = BSD_CRC_MAGICNUMBER,

    {   /* alarmZone */
        .x_min = -5.0f,  /* -5.00m */
        .x_max = 2.5f,  /*  2.50m */
        .y_min = -3.5f,  /* -3.00m */
        .y_max = -0.5f,   /* -0.50m */
    },
    {	/*misinformerZone*/
            .x_min = -5.0f,  /* -5.00m */
            .x_max = 2.5f,  /*  2.50m */
            .y_min = -3.0f,  /* -3.00m */
            .y_max = -0.5f,   /* -0.50m */
    },
    .alarmTargetSpeed = 2, /* >1m/s */
    .suppressVelSpeed = -1,
    .suppressTime = 20,
    .activeSpeed = 2.8f,      /* >15kmh */
    .exitTime = 12,
    .suppressEnable = true,
};
BSD_Param_t BSD_Param_Left =
{
    .CRCmagicnumber = BSD_CRC_MAGICNUMBER,

    {   /* alarmZone */
        .x_min = -5.0f,  /* -5.00m */
        .x_max = 2.5f,  /*  2.50m */
        .y_min = 0.5f,  /*  0.50m */
        .y_max = 3.5f,  /*  3.00m */
    },
    {	/*misinformerZone*/
            .x_min = -5.0f,  /* -5.00m */
            .x_max = 2.5f,  /*  2.50m */
            .y_min = 0.5f,  /*  0.50m */
            .y_max = 3.0f,  /*  3.00m */
    },
    .alarmTargetSpeed = 2, /* >1m/s */
    .suppressVelSpeed = -1,
    .suppressTime = 20,
    .activeSpeed = 2.8f,      /* >15kmh */
    .exitTime = 12,
    .suppressEnable = true,
};



/*RCW*/
RCW_Param_t RCW_Param_Right =
{
    .CRCmagicnumber = RCW_CRC_MAGICNUMBER,

    {   /* alarmZone */
        .x_min = -30, /* -70.00m */
        .x_max = -2,  /* -2.00m */
        .y_min = 0,  /* -1.00m */
        .y_max = 1.5f,   /* 1.00m */
    },

    .collisionPoint = 0,
    .minClosingSpeed = 0.5f,
    .alarmTTC = 3.5f,
    .activeSpeed = 8.33f,     /*15kmh*/
    .exitTime = 6,
};

RCW_Param_t RCW_Param_Left =
{
    .CRCmagicnumber = RCW_CRC_MAGICNUMBER,

    {   /* alarmZone */
        .x_min = -30,  /* -70.00m */
        .x_max = -2,   /* -2.00m */
        .y_min = -1.5f,   /* -1.00m */
        .y_max = 0,    /* 1.00m  */
    },

    .collisionPoint = 0,
    .minClosingSpeed = 0.5f,
    .alarmTTC = 3.5f,
    .activeSpeed = 8.33f,  /*15kmh*/
    .exitTime = 6,
};


void Functional_alarm(AppHandle_t* appHandle,target_infov2_t*info, uint16_t gnumObjs)
{
    BSD_Init(&appHandle->BSD_Handle, &BSD_Param_Right);
    RCW_Init(&appHandle->RCW_Handle, &RCW_Param_Right);
    if (gnumObjs > MAXNUM_OF_DETECTION)
    {
        gnumObjs = MAXNUM_OF_DETECTION;
    }
	memset(&VehicleInfo, 0, sizeof(Type_VehicleInfo));
    printf("numobj = %d\n", gnumObjs);
    RCW_Run(&appHandle->RCW_Handle, &VehicleInfo, info, gnumObjs);
   // BSD_Run(&app_Handle.BSD_Handle, &VehicleInfo, obj);
}
