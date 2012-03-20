//******************************************************************************
//  Note: Internal pull-ups may be used in this example for SDA & SCL
//        bq27541 7-bit Slave address = 1010101b
//
//  R. Wu
//  Texas Instruments Inc.
//  March 2010
//  Built with IAR Embedded Workbench Version: 4.21A
//******************************************************************************
#include <common.h>
#include <command.h>
#include <i2c.h>
#include <twl4030.h>
#include <cmd_tft.h>
#include "check_battery.h"                                // Device-specific header

#define ATRATE_MA            -100           // USER CONFIG: AtRate setting (mA)
#define I2CSLAVEADDR         0x55           // 7-bit slave address
#define BUFFERSIZE             20           // # of bytes for Tx & Rx buffers
#define OMAP_HSMMC2_BASE 	 0x480B4000
//#define NO_COMMON_BATTERY 		1
//#define DISPLAY_VERTICAL	 1			

#ifdef DISPLAY_VERTICAL
#include "mylogo_s.h"
#else
#include "mylogo.h"
#endif
unsigned char TxData[BUFFERSIZE];           // Stores data bytes to be TX'd
unsigned char RxData[BUFFERSIZE];           // Stores data bytes that are RX'd

void bq27541_read(unsigned char cmd, unsigned int bytes);
void bq27541_cmdWrite(unsigned char cmd, unsigned char data);
void bq27541_blockWrite(unsigned char *buffer, unsigned int length);
void bq27541_error(void);
void SHA1_authenticate(void);

UINT32 Rotl(UINT32 x, int n)
{
  return ( (x<<n) | (x>>(32-n)) );
}	

UINT32 W(int t)
{
  return (Rotl(Ws[t-3] ^ Ws[t-8] ^ Ws[t-14] ^ Ws[t-16], 1));
}	

UINT32 K(int t)
{
  if (t <= 19)
    return 0x5a827999;
  else if ( (t >= 20) && (t <= 39) )
    return 0x6ed9eba1;
  else if ( (t >= 40) && (t <= 59) )
    return 0x8f1bbcdc;
  else if ( (t >= 60) && (t <= 79) )
    return 0xca62c1d6;
  else
    return 0;		                    // Invalid value, not expected
}

UINT32 f(UINT32 x, UINT32 y, UINT32 z, int t)
{
  if (t <= 19)
    return ( (x & y) ^ ((~x) & z) );
  else if ( (t >= 20) && (t <= 39) )
    return (x ^ y ^ z);
  else if ( (t >= 40) && (t <= 59) )
    return ( (x & y) ^ (x & z) ^ (y & z) );
  else if ( (t >= 60) && (t <= 79) )
    return (x ^ y ^ z);
  else
    return 0;                               // Invalid value, not expected
}	

void bq27541_read(unsigned char cmd, unsigned int bytes)
{
//	int i = 0;
	i2c_read(I2CSLAVEADDR, cmd, 1, RxData, bytes);
}

void rtc_read(unsigned char cmd, unsigned int bytes)
{
	i2c_read(0x4b, cmd, 1, RxData, bytes);
}

void rtc_write(unsigned char cmd, unsigned char data)
{	
	i2c_write(0x4b, cmd, 1, &data, 1);
}

void bq27541_cmdWrite(unsigned char cmd, unsigned char data)
{
	i2c_write(I2CSLAVEADDR, cmd, 1, &data, 1);
}

void bq27541_blockWrite(unsigned char *buffer, unsigned int length)
{
	int i;
	unsigned char data_in[length];
	unsigned char data_in_tmp[length-1];
	for(i=0; i<length; i++)
		data_in[i] = *buffer++;

	for(i=1; i<length; i++)
		data_in_tmp[i-1] = data_in[i];

	i2c_write(I2CSLAVEADDR, data_in[0], 1, data_in_tmp, length-1);
}

void bq27541_error(void)
{
	printf("The battery is control must by  LENOVO's bq27541!\n");
}
#ifdef DISPLAY_VERTICAL
void display_draw_err_logo_vertical(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
//	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	int j;
	int k;
	for(k=0;k<600;k++)
		for(j=0; j<1024*2; j=j+4)
		{
            if(k>=100 && k<500 && j>=600 && j<800) 
				*pfb++= (UINT32)(gImage_mylogo_s[(k-100)*200+j-600])*0x00000001 +
                 		(UINT32)(gImage_mylogo_s[(k-100)*200+j-599])*0x00000100 +
                 		(UINT32)(gImage_mylogo_s[(k-100)*200+j-598])*0x00010000 +
                 		(UINT32)(gImage_mylogo_s[(k-100)*200+j-597])*0x01000000;
			else
				*pfb++ = 0xFFFFFFFF;
		}
	printf("display_draw_error_logo\n");
}

#else
void display_draw_err_logo_horizontal(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
//	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	int j;
	int k;
	for(k=0;k<600;k++)
		for(j=0; j<1024*2; j=j+4)
		{
            if(k>=250 && k<350 && j>=600 && j<1400) 
				*pfb++= (UINT32)(gImage_mylogo[(k-250)*800+j-600])*0x00000001 +
                 		(UINT32)(gImage_mylogo[(k-250)*800+j-599])*0x00000100 +
                 		(UINT32)(gImage_mylogo[(k-250)*800+j-598])*0x00010000 +
                 		(UINT32)(gImage_mylogo[(k-250)*800+j-597])*0x01000000;
			else
				*pfb++ = 0xFFFFFFFF;
		}
	printf("display_draw_error_logo\n");
}

#endif

int bq27541_sha(void)
{ 
  unsigned int i, bytes;
  unsigned int sum = 0;
  unsigned char checksum;
  unsigned char value;
  unsigned char time[4];
  unsigned int H_tmp[5];
 

  select_bus(0, 100);
  
  i2c_write(0x4B,0x2A,1,&value,1);
  value = 0x01;
  i2c_write(0x4B,0x29,1,&value,1);

  /* Henry Li@2011.8.12 Disable rtc timer and alarm interrupt */
  /*
  value = 0x0C;
  i2c_write(0x4B,0x2B,1,&value,1);
  */
  
  i2c_read(0x4B, 0x29, 1, &value, 1);
  value |= 0x40;
  i2c_write(0x4B,0x29,1,&value,1);


  i2c_read(0x4B, 0x1C, 1, &time, 4);
	
  Key[15] = 0x20;
  Key[14] = 0x11;
  Key[13] = 0x06;
  Key[12] = 0x07;
  Key[11] = 0x20;
  Key[10] = 0xCA;
  Key[ 9] = 0xCA;
  Key[ 8] = 0xCA;
  Key[ 7] = 0xCB;
  Key[ 6] = 0xDC;
  Key[ 5] = 0xBD;
  Key[ 4] = 0x02;
  Key[ 3] = 0x70;
  Key[ 2] = 0x60;
  Key[ 1] = 0x11;
  Key[ 0] = 0x02;

  Message[19] = 0x20;	
  Message[18] = 0xAA;
  Message[17] = 0xD3;
  Message[16] = 0xB8;
  
  Message[15] = 0xF0;
  Message[14] = 0xD1;
  Message[13] = time[3];  //0xFE;
  Message[12] = time[2];  //0x82;

  Message[11] = time[1];  //0x8E;
  Message[10] = time[0];  //0x07;
  Message[9] = 0x0A;
  Message[8] = 0x7C;

  Message[7] = 0x10;
  Message[6] = 0xDE;
  Message[5] = 0xC7;
  Message[4] = 0x26;

  Message[3] = 0xC8;
  Message[2] = 0x2C;
  Message[1] = 0xA3;
  Message[0] = 0xCA;

  
  SHA1_authenticate();
  //printf("11111111111111111111111111\n");
  select_bus(1, 400);
  
  bq27541_cmdWrite(bq27541CMD_CNTL_LSB, 0x20);
  
  bq27541_cmdWrite(bq27541CMD_CNTL_MSB, 0);

  for (i = 0; i < BUFFERSIZE; i++)          
  {
    TxData[i] = i;                          // Initialize data to be written
  }

  bq27541_cmdWrite(bq27541CMD_DFBLK, 0);// Select offset within the flash   success
  
  for (i = 0; i < BUFFERSIZE; i++)          // Compute the checksum of the block
  {
    sum += TxData[i];                       // Calculate the sum of the values  
  }
  checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
  
  udelay(200000);
  for (i = 0; i < BUFFERSIZE; i++)          // Write 32 bytes to Info Block A     success 20 bytes
  {
	//udelay(200000);
    bq27541_cmdWrite((bq27541CMD_ADF+i), TxData[i]);  
  }

  //bq27541_cmdWrite(bq27541CMD_ACKSDFD, checksum);    //success
  bq27541_read(bq27541CMD_ADF, 20);  // Read the contents of the block
 
  for (i = 0; i < 20; i++)          // Check if writes were successful
  {
//	printf("TxData[%d] = [%2d], RxData[%d] = [%2d]\n", i, TxData[i], i, RxData[i]);
    if (TxData[i] != RxData[i])             // Tx & Rx data values match?
    {
      bq27541_error();               // Signal error condition occurred
//	  printf("TxData[%d] = [%2d], RxData[%d] = [%2d]\n", i, TxData[i], i, RxData[i]);
    }
  }

  // TODO: Insert the private 128-bit key that is stored in the bq27541
  // Key[15..8] = K1 (highest 64 bits of the key)
  // Key[ 7..0] = K0 (lowest 64 bits of the key)
  // In this example 0x0123456789ABCDEFFEDCBA9876543210 is used since a fresh
  // unprogrammed bq27541 device has this as its default for the 128-bit key.
#ifdef NO_COMMON_BATTERY
  Key[15] = 0x20;
  Key[14] = 0x11;
  Key[13] = 0x06;
  Key[12] = 0x07;
  Key[11] = 0x20;
  Key[10] = 0xCA;
  Key[ 9] = 0xCA;
  Key[ 8] = 0xCA;
  Key[ 7] = 0xCB;
  Key[ 6] = 0xDC;
  Key[ 5] = 0xBD;
  Key[ 4] = 0x02;
  Key[ 3] = 0x70;
  Key[ 2] = 0x60;
  Key[ 1] = 0x11;
  Key[ 0] = 0x02;
#else
  Key[15] = 0x01;
  Key[14] = 0x23;
  Key[13] = 0x45;
  Key[12] = 0x67;
  Key[11] = 0x89;
  Key[10] = 0xAB;
  Key[ 9] = 0xCD;
  Key[ 8] = 0xEF;
  Key[ 7] = 0xFE;
  Key[ 6] = 0xDC;
  Key[ 5] = 0xBA;
  Key[ 4] = 0x98;
  Key[ 3] = 0x76;
  Key[ 2] = 0x54;
  Key[ 1] = 0x32;
  Key[ 0] = 0x10;
#endif

  Message[19] = (UINT8)(H[4]>>24)&0xFF;	
  Message[18] = (UINT8)(H[4]>>16)&0xFF;
  Message[17] = (UINT8)(H[4]>>8)&0xFF;
  Message[16] = (UINT8)(H[4]>>0)&0xFF;
  
  Message[15] = (UINT8)(H[3]>>24)&0xFF;
  Message[14] = (UINT8)(H[3]>>16)&0xFF;
  Message[13] = (UINT8)(H[3]>>8)&0xFF;
  Message[12] = (UINT8)(H[3]>>0)&0xFF;

  Message[11] = (UINT8)(H[2]>>24)&0xFF;
  Message[10] = (UINT8)(H[2]>>16)&0xFF;
  Message[9] = (UINT8)(H[2]>>8)&0xFF;
  Message[8] = (UINT8)(H[2]>>0)&0xFF;

  Message[7] = (UINT8)(H[1]>>24)&0xFF;
  Message[6] = (UINT8)(H[1]>>16)&0xFF;
  Message[5] = (UINT8)(H[1]>>8)&0xFF;
  Message[4] = (UINT8)(H[1]>>0)&0xFF;

  Message[3] = (UINT8)(H[0]>>24)&0xFF;
  Message[2] = (UINT8)(H[0]>>16)&0xFF;
  Message[1] = (UINT8)(H[0]>>8)&0xFF;
  Message[0] = (UINT8)(H[0]>>0)&0xFF;
  
  #if 0  
  printf("Radom data [4] = %8x:\n", H[4]);
  printf("Radom data [3] = %8x:\n", H[3]);
  printf("Radom data [2] = %8x:\n", H[2]);
  printf("Radom data [1] = %8x:\n", H[1]);
  printf("Radom data [0] = %8x:\n", H[0]);
#endif

  SHA1_authenticate();                      // Execute SHA-1/HMAC algorithm
  
  H_tmp[0] = H[0];
  H_tmp[1] = H[1];
  H_tmp[2] = H[2];
  H_tmp[3] = H[3];
  H_tmp[4] = H[4];
  
  Key[15] = 0x20;
  Key[14] = 0x11;
  Key[13] = 0x06;
  Key[12] = 0x07;
  Key[11] = 0x20;
  Key[10] = 0xCA;
  Key[ 9] = 0xCA;
  Key[ 8] = 0xCA;
  Key[ 7] = 0xCB;
  Key[ 6] = 0xDC;
  Key[ 5] = 0xBD;
  Key[ 4] = 0x02;
  Key[ 3] = 0x70;
  Key[ 2] = 0x60;
  Key[ 1] = 0x11;
  Key[ 0] = 0x02;
  
   SHA1_authenticate(); 
  
  // Authenticate the bq27541
 // bq27541_cmdWrite(bq27541CMD_DFDCNTL, 1); // BlockDataControl() = 0x01    error
  // Write block of random challenge to bq27541 (starting at location ADF)
  bytes = 0;
  TxData[bytes++] = bq27541CMD_ADF;
  for (i = 1; i <= RANDMESGNUMBYTES; i++)
  {
    TxData[bytes++] = Message[i-1]; 
  }
  
  bq27541_blockWrite(TxData, bytes);
  
  // Write checksum for the challenge to the bq27541
  sum = 0;
  for (i = 0; i < RANDMESGNUMBYTES; i++)    // Compute the checksum of the block
  {
    sum += Message[i];                      // Calculate the sum of the values  
  }
  checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
  bq27541_cmdWrite(bq27541CMD_ACKSDFD, checksum);    //success
  // Read back the digest from the bq27541
  
  udelay(500000);
  bq27541_read(bq27541CMD_ADF, RANDMESGNUMBYTES);// Read digest contents
  //udelay(200000);
  //bq27541_read(bq27541CMD_ADF, 20);
  // The 20 bytes of the digest returned by the bq27541 is arranged in 32-bit
  // words so that it can be compared with the results computed by the MCU
  Digest_32[4] = (UINT32)(RxData[ 0])*0x00000001 +
                 (UINT32)(RxData[ 1])*0x00000100 +
                 (UINT32)(RxData[ 2])*0x00010000 +
                 (UINT32)(RxData[ 3])*0x01000000;
  Digest_32[3] = (UINT32)(RxData[ 4])*0x00000001 +
                 (UINT32)(RxData[ 5])*0x00000100 +
                 (UINT32)(RxData[ 6])*0x00010000 +
                 (UINT32)(RxData[ 7])*0x01000000; 
  Digest_32[2] = (UINT32)(RxData[ 8])*0x00000001 +
                 (UINT32)(RxData[ 9])*0x00000100 +
                 (UINT32)(RxData[10])*0x00010000 +
                 (UINT32)(RxData[11])*0x01000000;
  Digest_32[1] = (UINT32)(RxData[12])*0x00000001 +
                 (UINT32)(RxData[13])*0x00000100 +
                 (UINT32)(RxData[14])*0x00010000 +
                 (UINT32)(RxData[15])*0x01000000;
  Digest_32[0] = (UINT32)(RxData[16])*0x00000001 +
                 (UINT32)(RxData[17])*0x00000100 +
                 (UINT32)(RxData[18])*0x00010000 +
                 (UINT32)(RxData[19])*0x01000000;

  if (((Digest_32[0] == H[0]) && (Digest_32[1] == H[1]) &&
       (Digest_32[2] == H[2]) && (Digest_32[3] == H[3]) &&
       (Digest_32[4] == H[4]) ) 
	   || ( (Digest_32[0] == H_tmp[0]) && (Digest_32[1] == H_tmp[1]) 
	   && (Digest_32[2] == H_tmp[2]) && (Digest_32[3] == H_tmp[3]) 
	   && (Digest_32[4] == H_tmp[4])))
	{
		return 1;
	}
	else
	{
		return 0;
	}
	  
}	  
void check_battery_is_right(void){
	int usb_in;
	gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
	int err = 0;
	int i;

	for(i=0; i<6; i++)
	{
		err = bq27541_sha();
		printf("err------>%d,  i----->%d\n", err, i);
		if(err == 1)
			break;
	}

	if(err == 1)
	  {
		printf("The battery model is ok!\n");
		select_bus(0, 100);
	  }
	  else
	  {	
		select_bus(0, 100);
		printf("The battery model is not right! System power off!!!\n");
	#ifdef DISPLAY_VERTICAL
		display_draw_err_logo_vertical();
	#else
		display_draw_err_logo_horizontal();
	#endif
		run_command("panel", 0);
		udelay(10000000);
		
		//usb_in = read_gpio_value((u32)&gpio2_base->datain, 19);
		usb_in = dc_ok_detect();
	//	printf("dc_ok_detect = %d!\n", usb_in);
		run_command("paneloff", 0);
		while(usb_in == 0)
		{
			sr32((u32)&gpio2_base->oe, 10, 1, 0);
			sr32((u32)&gpio2_base->setdataout, 10, 1, 1); 
			printf("The battery model is not right! System power off!!!\n");
			udelay(2000000);
			usb_in = dc_ok_detect();
		}
		twl4030_poweroff();// Error condition
	  }
 
}

void SHA1_authenticate(void)
{
  int i; // Used for doing two times the SHA1 as required by the bq26100
  int t; // Used for the indexes 0 through 79
  UINT32 temp; // Used as the temp variable during the loop in which the
               // working variables A, B, C1, D and E are updated
	
  // The 20 bytes of random message that are given to the bq26100 are arranged
  // in 32-bit words so that the microcontroller can compute the SHA1/HMAC
  Random[0] = (UINT32)(Message[16])*0x00000001 +
              (UINT32)(Message[17])*0x00000100 +
              (UINT32)(Message[18])*0x00010000 +
              (UINT32)(Message[19])*0x01000000;
  Random[1] = (UINT32)(Message[12])*0x00000001 +
              (UINT32)(Message[13])*0x00000100 +
              (UINT32)(Message[14])*0x00010000 +
              (UINT32)(Message[15])*0x01000000;
  Random[2] = (UINT32)(Message[ 8])*0x00000001 +
              (UINT32)(Message[ 9])*0x00000100 +
              (UINT32)(Message[10])*0x00010000 +
              (UINT32)(Message[11])*0x01000000;
  Random[3] = (UINT32)(Message[ 4])*0x00000001 +
              (UINT32)(Message[ 5])*0x00000100 +
              (UINT32)(Message[ 6])*0x00010000 +
              (UINT32)(Message[ 7])*0x01000000;
  Random[4] = (UINT32)(Message[ 0])*0x00000001 +
              (UINT32)(Message[ 1])*0x00000100 +
              (UINT32)(Message[ 2])*0x00010000 +
              (UINT32)(Message[ 3])*0x01000000;
  // The SHA1 is computed two times so that it complies with the bq26100 spec
  for (i = 0; i <= 1; i++)
  {
    // Work Schedule
    // The first four Working schedule variables Ws[0-3], are based on the key
    // that is implied that the bq26100 contains
    Ws[0] = (UINT32)(Key[12])*0x00000001 +
            (UINT32)(Key[13])*0x00000100 +
            (UINT32)(Key[14])*0x00010000 +
            (UINT32)(Key[15])*0x01000000;
    Ws[1] = (UINT32)(Key[ 8])*0x00000001 +
            (UINT32)(Key[ 9])*0x00000100 +
            (UINT32)(Key[10])*0x00010000 +
            (UINT32)(Key[11])*0x01000000;
    Ws[2] = (UINT32)(Key[ 4])*0x00000001 +
            (UINT32)(Key[ 5])*0x00000100 +
            (UINT32)(Key[ 6])*0x00010000 +
            (UINT32)(Key[ 7])*0x01000000;
    Ws[3] = (UINT32)(Key[ 0])*0x00000001 +
            (UINT32)(Key[ 1])*0x00000100 +
            (UINT32)(Key[ 2])*0x00010000 +
            (UINT32)(Key[ 3])*0x01000000;
    // On the first run of the SHA1 the random message is used 		
    if (i == 0)
    {
      Ws[4] = Random[0];
      Ws[5] = Random[1];
      Ws[6] = Random[2];
      Ws[7] = Random[3];
      Ws[8] = Random[4];
    }
    // On the second run of the SHA1, H(Kd || M) is used		
    else
    {
      Ws[4] = H[0];
      Ws[5] = H[1];
      Ws[6] = H[2];
      Ws[7] = H[3];
      Ws[8] = H[4];
    }
    // The Work schedule variables Ws[9-15] remain the same regardless of 
    // which run of the SHA1.  These values are as required by bq26100.
    Ws[9]  = 0x80000000;
    Ws[10] = 0x00000000;
    Ws[11] = 0x00000000;
    Ws[12] = 0x00000000;
    Ws[13] = 0x00000000;
    Ws[14] = 0x00000000;
    Ws[15] = 0x00000120;

    // The Work schedule variables Ws[16-79] are determined by the W(t) func
    for (t = 16; t <= 79; t++)
      Ws[t] = W(t);
    // Working Variables, always start the same regardless of which SHA1 run
    A  = 0x67452301;
    B  = 0xefcdab89;
    C1 = 0x98badcfe;
    D  = 0x10325476;
    E  = 0xc3d2e1f0;
    // Hash reads, always start the same regardless of what SHA1 run
    H[0] = A;
    H[1] = B;
    H[2] = C1;
    H[3] = D;
    H[4] = E;
    // Loop to change working variables A, B, C1, D and E
    // This is defined by FIPS 180-2 document
    for (t = 0; t <= 79; t++)
    {
      temp = Rotl(A,5) + f(B,C1,D,t) + E + K(t) + Ws[t];
      E = D;
      D = C1;
      C1 = Rotl(B,30);
      B = A;
      A = temp;
    }
    // 160-Bit SHA-1 Digest
    H[0] = (A  + H[0]);
    H[1] = (B  + H[1]);
    H[2] = (C1 + H[2]);
    H[3] = (D  + H[3]);
    H[4] = (E  + H[4]);
  }
}
