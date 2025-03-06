#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../Header/senserconfig.h"
#include "../Header/AssisedDriing.h"
#include <stdbool.h>


AppHandle_t  app_Handle;
extern uint16_t gnumObjs;
extern DPIF_PointCloud target[MAXNUM_OF_DETECTION];
Type_VehicleInfo VehicleInfo;

/*BSD*/
BSD_Param_t BSD_Param =
{
    .CRCmagicnumber = BSD_CRC_MAGICNUMBER,

    {   /* alarmZone_Left */
        .x_min = -4.0f,  /* -5.00m */
        .x_max = -0.5f,  /*  2.50m */
        .y_min = 0.0f,  /*  0.50m */
        .y_max = 5.0f,  /*  3.00m */
    },
    {   /* alarmZone_Right */
        .x_min = 0.5f,  /* -5.00m */
        .x_max = 4.0f,  /*  2.50m */
        .y_min = 0.0f,  /* -3.00m */
        .y_max = 5.0f,   /* -0.50m */
    },
    {   /* alarmZone_Middle */
        .x_min = -0.5f,  /* -5.00m */
        .x_max = 0.5f,  /*  2.50m */
        .y_min = 0.0f,  /* -3.00m */
        .y_max = 5.0f,   /* -0.50m */
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
        .activeSpeed = 0.83f,      /* >3kmh */
        .exitTime = 12,
        .suppressEnable = true,
};


/*RCW*/
RCW_Param_t RCW_Param =
{
    .CRCmagicnumber = RCW_CRC_MAGICNUMBER,

    {   /* alarmZon_Left */
        .x_min = -4,  /* -70.00m */
        .x_max = -0.5,   /* -2.00m */
        .y_min = 4,   /* -1.00m */
        .y_max = 30,    /* 1.00m  */
	},
	{   /* alarmZone_Right */
		.x_min = 0.5, /* -70.00m */
		.x_max = 4,  /* -2.00m */
		.y_min = 4,  /* -1.00m */
		.y_max = 30.0f,   /* 1.00m */
	},
	{   /* alarmZone_Middle */
		.x_min = -0.5,  /* -70.00m */
		.x_max = 0.5,   /* -2.00m */
		.y_min = 4,   /* -1.00m */
		.y_max = 30,    /* 1.00m  */
	},

    .collisionPoint = 0,
    .minClosingSpeed = 0.5f,
    .alarmTTC = 3.5f,
    .activeSpeed = 0.83f,     /*15kmh*/
    .exitTime = 6,
};


RCW_Param_t RCW_Param_Middle =
{
    .CRCmagicnumber = RCW_CRC_MAGICNUMBER,

    {   /* alarmZone */
        .x_min = -0.5,  /* -70.00m */
        .x_max =  0.5,   /* -2.00m */
        .y_min = 4,   /* -1.00m */
        .y_max = 30,    /* 1.00m  */
    },

    .collisionPoint = 0,
    .minClosingSpeed = 0.5f,
    .alarmTTC = 3.5f,
    .activeSpeed = 8.33f,  /*15kmh*/
    .exitTime = 6,
};

static const WarningDescriptor warning_config[] = {
    // BSD模块告警
    {MODULE_BSD, WARNING_BSD2_Middle,   1},
    {MODULE_BSD, WARNING_BSD2_Left,     2},
    {MODULE_BSD, WARNING_BSD2_Right,    3},
    {MODULE_BSD, WARNING_BSD1_Middle,   4},
    {MODULE_BSD, WARNING_BSD1_Left,     5},
    {MODULE_BSD, WARNING_BSD1_Right,    6},
    // RCW模块告警
    {MODULE_RCW, WARNING_RCW2_Middle,    7},
    {MODULE_RCW, WARNING_RCW2_Left,      8},
    {MODULE_RCW, WARNING_RCW2_Right,     9},
    {MODULE_RCW, WARNING_RCW1_Middle,    10},
    {MODULE_RCW, WARNING_RCW1_Left,      11},
    {MODULE_RCW, WARNING_RCW1_Right,     12},
    // ... 其他配置项
};

uint8_t get_highest_priority(AppHandle_t* handle,uint16_t gnumObjs) {
    uint8_t highest = UINT8_MAX;  // 初始化为最大值
    WarningType current = 0;
    // 遍历所有模块
    for (ModuleType mod = 0; mod < MODULE_COUNT; mod++) {
        // 获取当前模块的ObjState数组

        // 遍历模块内的所有对象
        for (uint8_t idx = 0; idx < gnumObjs; idx++) {
            if (mod == MODULE_BSD)
            {
				 current = handle->BSD_Handle.ObjState[idx].isWarning;
            }
            else if (mod == MODULE_RCW)
            {
                 current = handle->RCW_Handle.ObjState[idx].isWarning;
			}
			else
			{
				continue;
			}

            // 查表获取优先级
            for (size_t i = 0; i < sizeof(warning_config) / sizeof(warning_config[0]); i++) {
                if (warning_config[i].module == mod &&
                    warning_config[i].type == current) {
                    if (warning_config[i].priority < highest) {
                        highest = warning_config[i].priority;
                    }
                    break;
                }
            }
        }
    }

    return (highest != UINT8_MAX) ? highest : 0;  // 返回0表示无告警
}

// 主逻辑函数
void Alarm_priority_Logic(AppHandle_t* handle,uint16_t gnumObjs) {
    uint8_t warning_marker = get_highest_priority(handle, gnumObjs);
}

void Functional_alarm(AppHandle_t* appHandle,target_infov2_t*info, uint16_t gnumObjs)
{
    BSD_Init(&appHandle->BSD_Handle, &BSD_Param);
    RCW_Init(&appHandle->RCW_Handle, &RCW_Param);
    if (gnumObjs > MAXNUM_OF_DETECTION)
    {
        gnumObjs = MAXNUM_OF_DETECTION;
    }
	memset(&VehicleInfo, 0, sizeof(Type_VehicleInfo));
    printf("numobj = %d\n", gnumObjs);
    RCW_Run(&appHandle->RCW_Handle, &VehicleInfo, info, gnumObjs);
    BSD_Run(&appHandle->BSD_Handle, &VehicleInfo, info, gnumObjs);

    Alarm_priority_Logic(appHandle, gnumObjs);

}
