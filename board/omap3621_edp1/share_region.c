/***********************************************************
 * We can pass variables to the kernal through share region. 
 * Then we can use this region to do many things.
 ***********************************************************/
#include <common.h>
#include <share_region.h>
#include <twl4030.h>

share_region_t * const share_region = (share_region_t *)SHARE_REGION_BASE;
upgrade_mem_t * const upgrade_mem = (upgrade_mem_t *)UPGRADE_MEM_BASE;
unsigned int gRebootRecovery = 0;

unsigned long lowlevel_crc32(unsigned long crc, const unsigned char *buf, unsigned int len);
/* Return the CRC of the bytes buf[0..len-1]. */
static unsigned bg_crc32(unsigned char *buf, int len) 
{
   /* uboot crc32() has opposite semantics than the linux kernel crc32_le() */
   return crc32(0L, buf, len);
}

///////////////////////////////////////////////////////////////////////////////////
static int check_share_region(void)
{
	int result = 0;

	if ( share_region )
	{
		unsigned long computed_residue, checksum;

		checksum = share_region->checksum;

		computed_residue = ~bg_crc32((unsigned char *)share_region, SHARE_REGION_SIZE);

		result = CRC32_RESIDUE == computed_residue;
	}

	if ( result ) {
		//printf("share region: pass\n");
	} else {
		//printf("share region: fail\n");
	}

	return result;
}

void save_share_region(void)
{
    if ( share_region ) {
        share_region->checksum = bg_crc32((unsigned char *)share_region, SHARE_REGION_SIZE - sizeof (unsigned long) );
    }
}

void share_region_handle(void)
{
   if (FLAG_RECOVER_MODE == share_region->flags) {
	gRebootRecovery = 1;
	printf("share_region->flags=%d\n", share_region->flags);	
  }
  if(!check_share_region())
  {
		//printf("Share region: CRC invalid.\n"); 
	memset(share_region, 0x0, sizeof(share_region_t));
  }
	save_share_region();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////




static int check_upgrade_mem(void)
{
	int result = 0;

	if ( upgrade_mem )
	{
		unsigned long computed_residue, checksum;

	//	checksum = share_region->checksum;

		computed_residue = ~bg_crc32((unsigned char *)upgrade_mem, UPGRADE_MEM_SIZE);

		result = CRC32_RESIDUE == computed_residue;
	}

	if ( result ) {
		printf("upgrade: pass\n");
	} else {
		printf("upgrade: fail\n");
	}

	return result;
}



int recovery_handle(void)
{
	int ret;
	ret = BOOT_NORMAL;
	//power_key = value;
        printf("update_flag:%d\n", upgrade_mem->flags.update_flag);
	
	if(upgrade_mem->flags.update_flag != BOOT_NORMAL){
		printf("update_flag:%d\n", upgrade_mem->flags.update_flag);
		ret=upgrade_mem->flags.update_flag;
		goto out;
	}
	
	/******************about the product-line reset and upgrade**********************/	
	ret = twl4030_keypad_read_volume_key();
	if(ret == 1){
		upgrade_mem->flags.update_flag = BOOT_SYSTEM_UPGRADE; /*volume down for recovery */
		ret = BOOT_SYSTEM_UPGRADE;
		goto out;
	}
	else if (ret == 2){                        
		upgrade_mem->flags.update_flag = BOOT_FASTBOOT; /*volume up for fastboot */
		ret = BOOT_FASTBOOT;
		goto out;

	}
	                        

out:
	save_upgrade_mem();
	unsigned long *p = upgrade_mem;
	printf("upgrade_mem = %d\n", upgrade_mem->flags.update_flag);
	printf("upgrade_mem = %08x\n", *p);
	p++;
	printf("upgrade_mem = %08x\n", *p);
	return ret;
}


void save_upgrade_mem(void)
{
    if ( upgrade_mem) {
        upgrade_mem->checksum = bg_crc32((unsigned char *)upgrade_mem, UPGRADE_MEM_SIZE - sizeof (unsigned long) );
	printf("checksum=%08x\n", upgrade_mem->checksum);
    }
}

void reset_upgrade_mem(void)
{
    upgrade_mem->flags.update_flag = BOOT_NORMAL;
}

int upgrade_mem_handle(void)
{
	int ret_value;
	if(!check_upgrade_mem()){
		//printf("Share region: CRC invalid.\n"); 
		upgrade_mem->flags.update_flag = 0;
	//	memset(upgrade_mem, 0x0, sizeof(upgrade_mem_t));
	}
	ret_value = recovery_handle();
	save_upgrade_mem();
	return ret_value;
}
