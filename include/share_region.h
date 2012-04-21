/*
 * 1. The size of share region is 4KB.
 * 2. The share region locates at the end of the memory. 
 */

#ifndef __SHARE_REGION_H__
#define __SHARE_REGION_H__

#include <hardware_version.h>

#define CRC32_RESIDUE 0xdebb20e3UL /* ANSI X3.66 residue */

#ifndef PAGE_SIZE
# define PAGE_SIZE 4096
#endif

#define SHARE_REGION_SIZE PAGE_SIZE
#define UPGRADE_MEM_SIZE PAGE_SIZE
#define RESERVED_SIZE_T (SHARE_REGION_SIZE - sizeof(unsigned int) - \
																				   sizeof(enum BoardID_enum) - \
																				   sizeof(unsigned int) - \
																				   (sizeof(char)*64) - \
																				   sizeof(unsigned int) - \
																					sizeof(unsigned int) -\
																					sizeof(unsigned long))
#define RESERVED_SIZE (UPGRADE_MEM_SIZE - sizeof(struct share_region_flags) - sizeof(unsigned long))


/* Function Stat */
enum FlagStat_enum {
	FLAG_NOMALL						= 0,
	FLAG_FACTORY_DEFAULT  = 1,
	FLAG_RECOVER_MODE     = 2,
	FLAG_SHUTDOWN3				= 4,
	FLAG_REBOOT						= 8,
};



struct share_region_flags
{
	unsigned int boot_flag;
	unsigned int update_flag;
};

struct upgrade_mem_t
{
	struct share_region_flags flags;
	unsigned char reserved[RESERVED_SIZE];
	unsigned long checksum;
};

/*struct share_region_t
{
      //  struct share_region_flags flags;
        enum BoardID_enum hardware_id;
        unsigned int language;
        char software_version[64];
        unsigned char reserved[RESERVED_SIZE];
        unsigned long checksum;
};
struct share_region_t
{
//	unsigned int update_flag;
	unsigned int language;
	char software_version[64];
	enum BoardID_enum hardware_id;
	struct share_region_flags flags;
	unsigned char reserved[RESERVED_SIZE];
	unsigned long checksum;
};*/

/*Language ID */
enum Language_enum {
	LANG_ENGLISH						 = 0x0C09,
	LANG_END								 = 0xffff,
};

struct share_region_t
{
	unsigned int flags;
	enum BoardID_enum hardware_id;
	unsigned int language;
	char software_version[64];
	unsigned int debug_flag;
	unsigned int status_3G_exist;
	unsigned char reserved[RESERVED_SIZE_T];
	unsigned long checksum;
};

typedef struct share_region_t share_region_t;
typedef struct upgrade_mem_t upgrade_mem_t;

#ifdef CONFIG_EPXX_DDR_512MB
#define MEMORY_ADDRESS		0x9FE00000
#define SHARE_REGION_BASE (MEMORY_ADDRESS - 0x10000)
/*New adding by joe*/
#define UPGRADE_MEM_BASE (0x84000000)

#else
#define MEMORY_ADDRESS		0x8FE00000
#define SHARE_REGION_BASE (MEMORY_ADDRESS - 0x10000)
#endif

/*values of boot_flag*/
enum {
	BOOT_NORMAL = 0,
	BOOT_PRODUCT_LINE,
	BOOT_SYSTEM_UPGRADE,
	BOOT_FACTORY_RESET,
        BOOT_FASTBOOT
};

void reset_upgrade_mem(void);
extern void save_upgrade_mem(void);
extern int recovery_handle(void);
extern int  upgrade_mem_handle(void);
extern share_region_t * const share_region;
extern upgrade_mem_t * const upgrade_mem;
extern unsigned int gRebootRecovery;

#endif

