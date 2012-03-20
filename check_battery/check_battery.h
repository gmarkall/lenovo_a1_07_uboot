/* File: "check_battery.h" */

#define UINT8  unsigned char
#define UINT16 unsigned int
#define UINT32 unsigned long

#define RANDMESGNUMBYTES  20
#define DIGESTNUMBYTES    20
#define DEVICEIDNUMBYTES   8
#define SECRETKEYNUMBYTES 16



#define bq27541CMD_CNTL_LSB  0x00
#define bq27541CMD_CNTL_MSB  0x01
#define bq27541CMD_AR_LSB    0x02
#define bq27541CMD_AR_MSB    0x03
#define bq27541CMD_ARTTE_LSB 0x04
#define bq27541CMD_ARTTE_MSB 0x05
#define bq27541CMD_TEMP_LSB  0x06
#define bq27541CMD_TEMP_MSB  0x07
#define bq27541CMD_VOLT_LSB  0x08
#define bq27541CMD_VOLT_MSB  0x09
#define bq27541CMD_FLAGS_LSB 0x0A
#define bq27541CMD_FLAGS_MSB 0x0B
#define bq27541CMD_NAC_LSB   0x0C
#define bq27541CMD_NAC_MSB   0x0D
#define bq27541CMD_FAC_LSB   0x0E
#define bq27541CMD_FAC_MSB   0x0F
#define bq27541CMD_RM_LSB    0x10
#define bq27541CMD_RM_MSB    0x11
#define bq27541CMD_FCC_LSB   0x12
#define bq27541CMD_FCC_MSB   0x13
#define bq27541CMD_AI_LSB    0x14
#define bq27541CMD_AI_MSB    0x15
#define bq27541CMD_TTE_LSB   0x16
#define bq27541CMD_TTE_MSB   0x17
#define bq27541CMD_TTF_LSB   0x18
#define bq27541CMD_TTF_MSB   0x19
#define bq27541CMD_SI_LSB    0x1A
#define bq27541CMD_SI_MSB    0x1B
#define bq27541CMD_STTE_LSB  0x1C
#define bq27541CMD_STTE_MSB  0x1D
#define bq27541CMD_MLI_LSB   0x1E
#define bq27541CMD_MLI_MSB   0x1F
#define bq27541CMD_MLTTE_LSB 0x20
#define bq27541CMD_MLTTE_MSB 0x21
#define bq27541CMD_AE_LSB    0x22
#define bq27541CMD_AE_MSB    0x23
#define bq27541CMD_AP_LSB    0x24
#define bq27541CMD_AP_MSB    0x25
#define bq27541CMD_TTECP_LSB 0x26
#define bq27541CMD_TTECP_MSB 0x27
#define bq27541CMD_RSVD_LSB  0x28
#define bq27541CMD_RSVD_MSB  0x29
#define bq27541CMD_CC_LSB    0x2A
#define bq27541CMD_CC_MSB    0x2B
#define bq27541CMD_SOC_LSB   0x2C
#define bq27541CMD_SOC_MSB   0x2D
#define bq27541CMD_DCAP_LSB  0x3C
#define bq27541CMD_DCAP_MSB  0x3D
#define bq27541CMD_DFCLS     0x3E
#define bq27541CMD_DFBLK     0x3F
#define bq27541CMD_ADF       0x40
#define bq27541CMD_ACKSDFD   0x54
#define bq27541CMD_DFDCKS    0x60
#define bq27541CMD_DFDCNTL   0x61
#define bq27541CMD_DNAMELEN  0x62
#define bq27541CMD_DNAME     0x63

UINT8  Message[RANDMESGNUMBYTES];           // Random message
UINT8  Key[SECRETKEYNUMBYTES];              // Secret key
UINT32 Ws[80];			            // Global Work schedule variable
UINT32 A;
UINT32 B;
UINT32 C1;
UINT32 D;
UINT32 E;
UINT32 H[5];
UINT32 Random[5]; // 16 bytes random message for bq26100 to use in SHA1/HMAC
UINT32 Digest_32[5]; // Result of SHA1/HMAC obtained by MCU is contained here


