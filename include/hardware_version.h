#ifndef __HARDWARE_VERSION_H__
#define __HARDWARE_VERSION_H__

/* Board Version */
/* <--LH_SWRD_CL1_Mervins@2011.06.28:changne hardware version for cl1. */
enum BoardID_enum {
	BOARD_VERSION_UNKNOWN = 0,
	BOARD_ID_PRO  = 1,
	BOARD_ID_EVT = 2,
	BOARD_ID_DVT1  = 3,
	BOARD_ID_WIFI_DVT2  = 4,
	BOARD_ID_WIFI_PVT = 5,
	BOARD_ID_WIFI_MP = 6,
	BOARD_ID_3G_DVT2 = 7,
	BOARD_ID_3G_PVT = 8,
	BOARD_ID_3G_MP = 9,
};
/* LH_SWRD_CL1_Mervins@2011.06.28--> */

/** args main structure */
struct args_t
{
	enum BoardID_enum	 board_id;
  char software_version[45];
};

extern enum BoardID_enum g_BoardID;

#endif /* __HARDWARE_VERSION_H__ */
