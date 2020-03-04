/*******************************************************************************
  myRL78

  Definições de símbolos para uso com o RL78 (IAR C)

  by Fábio Pereira (2013)
  www.sctec.com.br/blog
*******************************************************************************/

#define BIT15 0x8000
#define BIT14 0x4000
#define BIT13 0x2000
#define BIT12 0x1000
#define BIT11 0x0800
#define BIT10 0x0400
#define BIT9  0x0200
#define BIT8  0x0100
#define BIT7  0x80
#define BIT6  0x40
#define BIT5  0x20
#define BIT4  0x10
#define BIT3  0x08
#define BIT2  0x04
#define BIT1  0x02
#define BIT0  0x01

// Registradores de opções *****************************************************
#define bWDTINIT            BIT7
#define WDT_WINDOW50        BIT5
#define WDT_WINDOW75        BIT6
#define WDT_WINDOW100       BIT6 | BIT5
#define bWDTON              BIT4
#define WDT_OFF             WDT_WINDOW100
#define WDT_3ms             0
#define WDT_7ms             BIT1
#define WDT_14ms            BIT2
#define WDT_29ms            BIT2 | BIT1
#define WDT_118ms           BIT3
#define WDT_474ms           BIT3 | BIT1
#define WDT_949ms           BIT3 | BIT2
#define WDT_3799ms          BIT3 | BIT2 | BIT1
#define bWDSTBYON           BIT0

#define bVPOC2              BIT7
#define bVPOC1              BIT6
#define bVPOC0              BIT5
#define bPORTSELB           BIT4
#define bLVIS1              BIT3
#define bLVIS0              BIT2
#define bLVIMDS1            BIT1
#define bLVIMDS0            BIT0
#define LVD_INT_MODE        BIT4 | bLVIMDS0
#define LVD_INTRES_MODE     BIT4 | bLVIMDS1
#define LVD_RESET_MODE      BIT4 | bLVIMDS1 | bLVIMDS0
#define LVD_VMODE3          BIT4 | bLVIS0
#define LVD_VMODE0          LVD_VMODE3 | bLVIS1
#define LVD_VMODE2          LVD_VMODE0 | bVPOC0
#define LVD_VMODE1          BIT4 | bLVIS1
#define LVD_VMODE4          LVD_VMODE1 | bVPOC0
#define LVD_VMODE5          LVD_VMODE3 | bVPOC0
#define LVD_VMODE6          LVD_VMODE0 | bVPOC1
#define LVD_VMODE7          LVD_VMODE1 | bVPOC1
#define LVD_VMODE8          LVD_VMODE3 | bVPOC1
#define LVD_VMODE9          LVD_VMODE2 | bVPOC1
#define LVD_VMODE10         BIT4
#define LVD_VMODE11         LVD_VMODE4 | bVPOC1
#define LVD_VMODE12         LVD_VMODE5 | bVPOC1
#define LVD_VMODE13         BIT4 | bVPOC0
#define LVD_VMODE14         BIT4 | bVPOC1
#define LVD_VMODE15         LVD_VMODE14 | bVPOC0
#define LVD_OFF             BIT4 | bVPOC2 | bLVIMDS0

#define FLASH_LV            BIT5
#define FLASH_LS            BIT7 | BIT5
#define FLASH_HS            BIT7 | BIT6 | BIT5
#define CLK_64MHZ           BIT5 | BIT4 | BIT3
#define CLK_48MHZ           BIT5 | BIT4
#define CLK_32MHZ           BIT5 | BIT3
#define CLK_24MHZ           BIT5
#define CLK_16MHZ           BIT5 | BIT3 | BIT0
#define CLK_12MHZ           BIT5 | BIT0
#define CLK_8MHZ            BIT5 | BIT3 | BIT1
#define CLK_4MHZ            BIT5 | BIT3 | BIT1 | BIT0
#define CLK_1MHZ            BIT5 | BIT3 | BIT2 | BIT0

#define DEBUG_ON_ERASE      BIT7 | BIT2
#define DEBUG_ON_NO_ERASE   BIT7 | BIT2 | BIT0
#define DEBUG_OFF           BIT2

// ADM0 ************************************************************************
#define bADCS               BIT7
#define bADMD               BIT6
#define bFR2                BIT5
#define bFR1                BIT4
#define bFR0                BIT3
#define bLV1                BIT2
#define bLV0                BIT1
#define bADCE               BIT0
#define ADCLK_DIV64         0
#define ADCLK_DIV32         1<<3
#define ADCLK_DIV16         2<<3
#define ADCLK_DIV8          3<<3
#define ADCLK_DIV6          4<<3
#define ADCLK_DIV5          5<<3
#define ADCLK_DIV4          6<<3
#define ADCLK_DIV2          7<<3
#define ADC_LV0             0
#define ADC_LV1             1<<1
#define ADC_LV2             2<<1
#define ADC_LV3             3<<1

// ADM1 ************************************************************************
#define bADTMD1             BIT7
#define bADTMD0             BIT6
#define bADSCM              BIT5
#define ADTRS1              BIT1
#define ADTRS0              BIT0
#define ADC_TRIG_SOFT       0
#define ADC_TRIG_HARD_NO_WAIT BIT7
#define ADC_TRIG_HARD_WAIT  3<<6
#define ADC_TRIG_TAU        0
#define ADC_TRIG_RTC        2
#define ADC_TRIG_IT         3

// ADM2 ************************************************************************
#define bADREFP1            BIT7
#define bADREFP0            BIT6
#define bADREFM             BIT5
#define bADRCK              BIT3
#define bAWC                BIT2
#define bADTYP              BIT0
#define ADC_8BIT            BIT0
#define ADC_SNOOZE_ENABLE   BIT2
#define ADC_REFP_VDD        0
#define ADC_REFP_PIN        BIT6
#define ADC_REFP_INT        BIT7

// ADS *************************************************************************
#define bADISS              BIT7
#define ADC_CH0             0
#define ADC_CH1             1
#define ADC_CH2             2
#define ADC_CH3             3
#define ADC_CH4             4
#define ADC_CH5             5
#define ADC_CH6             6
#define ADC_CH7             7
#define ADC_CH8             8
#define ADC_CH9             9
#define ADC_CH10            10
#define ADC_CH11            11
#define ADC_CH12            12
#define ADC_CH13            13
#define ADC_CH14            14
#define ADC_CH16            16
#define ADC_CH17            17
#define ADC_CH18            18
#define ADC_CH19            19
#define ADC_CH20            20
#define ADC_CH21            21
#define ADC_CH22            22
#define ADC_CH23            23
#define ADC_CH24            24
#define ADC_CH25            25
#define ADC_CH26            26
#define ADC_CH_TEMP         BIT7
#define ADC_CH_REF          BIT7 | BIT0

// ADTES ***********************************************************************
#define bADTES1             BIT1
#define bADTES0             BIT0
#define ADC_TEST_OFF        0
#define ADC_TEST_REFM       2
#define ADC_TEST_REFP       3

// CKC *************************************************************************
#define bCLS                BIT7
#define bCSS                BIT6
#define bMCS                BIT5
#define bMCM0               BIT4

// CKS *************************************************************************
#define bPCLOE              BIT7
#define bCSEL               BIT3

// CMC *************************************************************************
#define OSC_X1_OFF          0
#define OSC_X1_OSC          BIT6
#define OSC_X1_EXT          BIT7 | BIT6
#define OSC_XT1_OFF         0
#define OSC_XT1_OSC         BIT4
#define OSC_XT1_EXT         BIT5 | BIT4
#define OSC_XT1_LP          0
#define OSC_XT1_STD         BIT1
#define OSC_XT1_ULP         BIT2
#define bAMPH               BIT0

// CSC *************************************************************************
#define bMSTOP              BIT7
#define bXTSTOP             BIT6
#define bHIOSTOP            BIT5

// CRC0CTL *********************************************************************
#define bCRC0EN             BIT7

// DMC *************************************************************************
#define bSTG                BIT7
#define bDRS                BIT6
#define bDS                 BIT5
#define bDWAIT              BIT4
#define DMA_TRIG_SOFT       0
#define DMA_TRIG_ADC        1
#define DMA_TRIG_T00        2
#define DMA_TRIG_T10        2
#define DMA_TRIG_T01        3
#define DMA_TRIG_T11        3
#define DMA_TRIG_T02        4
#define DMA_TRIG_T12        4
#define DMA_TRIG_T03        5
#define DMA_TRIG_T13        5
#define DMA_TRIG_TX0        6
#define DMA_TRIG_RX0        7
#define DMA_TRIG_TX3        6
#define DMA_TRIG_RX3        7
#define DMA_TRIG_TX1        8
#define DMA_TRIG_RX1        9
#define DMA_TRIG_TX2        10
#define DMA_TRIG_RX2        11

// DRC *************************************************************************
#define bDEN                BIT7
#define bDST                BIT0

// HOCODIV *********************************************************************
#define bHOCODIV2           BIT2
#define bHOCODIV1           BIT1
#define bHOCODIV0           BIT0
#define HOCO_32MHZ          0
#define HOCO_16MHZ          1
#define HOCO_8MHZ           2
#define HOCO_4MHZ           3
#define HOCO_2MHZ           4
#define HOCO_1MHZ           5
#define HOCO_24MHZ          0
#define HOCO_12MHZ          1
#define HOCO_6MHZ           2
#define HOCO_3MHZ           3

// IAWCTL **********************************************************************
#define bIAWEN              BIT7
#define bGRAM1              BIT5
#define bGRAM0              BIT4
#define bGPORT              BIT2
#define bGINT               BIT1
#define bGCSC               BIT0
#define RAMGUARD_DIS        0
#define RAMGUARD_128        bGRAM0
#define RAMGUARD_256        bGRAM1
#define RAMGUARD_512        bGRAM1 | bGRAM0

// IICCTL0 *********************************************************************
#define bIICE               BIT7
#define bLREL               BIT6
#define bWREL               BIT5
#define bSPIE               BIT4
#define bWTIM               BIT3
#define bACKE               BIT2
#define bSTT                BIT1
#define bSPT                BIT0

// IICCTL1 *********************************************************************
#define bWUP                BIT7
#define bCLD                BIT5
#define bDAD                BIT4
#define bSMC                BIT3
#define bDFC                BIT2
#define bPRS                BIT0

// IICF ************************************************************************
#define bSTCF               BIT7
#define bIICBSY             BIT6
#define bSTCEN              BIT1
#define bIICRSV             BIT0

// IICS ************************************************************************
#define bMSTS               BIT7
#define bALD                BIT6
#define bEXC                BIT5
#define bCOI                BIT4
#define bTRC                BIT3
#define bACKD               BIT2
#define bSTD                BIT1
#define bSPD                BIT0

// ISC *************************************************************************
#define bISC1               BIT1
#define bISC0               BIT0

// ITMC ************************************************************************
#define bRINTE              BIT15

// MDUC ************************************************************************
#define bDIVMODE            BIT7
#define bMACMODE            BIT6
#define bMDSM               BIT3
#define bMACOF              BIT2
#define bMACSF              BIT1
#define bDIVST              BIT0
#define MAC_MUL_UNS         0
#define MAC_MUL_SIG         bMDSM
#define MAC_MAC_UNS         bMACMODE
#define MAC_MAC_SIG         bMACMODE | bMDSM
#define MAC_DIV_INT         bDIVMODE
#define MAC_DIV_NOINT       bDIVMODE | bMACMODE

// NFEN0 ***********************************************************************
#define SNFEN30             BIT6
#define SNFEN20             BIT4
#define SNFEN10             BIT2
#define SNFEN00             BIT0

// NFEN1 ***********************************************************************
#define TNFEN07             BIT7
#define TNFEN06             BIT6
#define TNFEN05             BIT5
#define TNFEN04             BIT4
#define TNFEN03             BIT3
#define TNFEN02             BIT2
#define TNFEN01             BIT1
#define TNFEN00             BIT0

// NFEN2 ***********************************************************************
#define TNFEN17             BIT7
#define TNFEN16             BIT6
#define TNFEN15             BIT5
#define TNFEN14             BIT4
#define TNFEN13             BIT3
#define TNFEN12             BIT2
#define TNFEN11             BIT1
#define TNFEN10             BIT0

// OSMC ************************************************************************
#define bRTCLPC             BIT7
#define bWUTMMCK0           BIT4

// OSTS ************************************************************************
#define OSTS_256            0
#define OSTS_512            1
#define OSTS_1k             2
#define OSTS_2k             3
#define OSTS_8k             4
#define OSTS_32k            5
#define OSTS_131k           6
#define OSTS_262k           7

// PER0 ************************************************************************
#define bRTCEN              BIT7
#define bIICA1EN            BIT6
#define bADCEN              BIT5
#define bIICA0EN            BIT4
#define bSAU1EN             BIT3
#define bSAU0EN             BIT2
#define bTAU1EN             BIT1
#define bTAU0EN             BIT0

// RESF ************************************************************************
#define bTRAP               BIT7
#define bWDTRF              BIT4
#define bRPERF              BIT2
#define bIAWRF              BIT1
#define bLVIRF              BIT0

// RPECTL **********************************************************************
#define bRPERDIS            BIT7
#define bRPEF               BIT0

// RTCC0 ***********************************************************************
#define bRTCE               BIT7
#define bRCLOE1             BIT5
#define bAMPM               BIT3
#define INT_RTC_OFF         0
#define INT_RTC_05S         1
#define INT_RTC_1S          2
#define INT_RTC_1MIN        3
#define INT_RTC_1H          4
#define INT_RTC_1D          5
#define INT_RTC_1M          6

// RTCC1 ***********************************************************************
#define bWALE               BIT7
#define bWALIE              BIT6
#define bWAFG               BIT4
#define bRIFG               BIT3
#define bRWST               BIT1
#define bRWAIT              BIT0

// SCRmn ***********************************************************************
#define SAU_COMM_DISABLE    0 | BIT2
#define SAU_COMM_RX         BIT14 | BIT2
#define SAU_COMM_TX         BIT15 | BIT2
#define SAU_COMM_TXRX       3 << 14  | BIT2
#define SAU_CSI_CLKMODE0    0 | BIT2
#define SAU_CSI_CLKMODE1    1 << 12 | BIT2
#define SAU_CSI_CLKMODE2    2 << 12 | BIT2
#define SAU_CSI_CLKMODE3    3 << 12 | BIT2
#define SAU_INTSRE_ENABLE   BIT10 | BIT2
#define SAU_NO_PARITY       0 | BIT2
#define SAU_PARITY_ZERO     1 << 8 | BIT2
#define SAU_PARITY_EVEN     2 << 8 | BIT2
#define SAU_PARITY_ODD      3 << 8 | BIT2
#define SAU_LSB_FIRST       BIT7 | BIT2
#define SAU_NO_STOP         0 | BIT2
#define SAU_ONE_STOP        1 << 4 | BIT2
#define SAU_TWO_STOP        2 << 4 | BIT2
#define SAU_9BITS           1 | BIT2
#define SAU_7BITS           2 | BIT2
#define SAU_8BITS           3 | BIT2

// SIRmn ***********************************************************************
#define bFECT               BIT2
#define bPECT               BIT1
#define bOVCT               BIT0

// SMRmn ***********************************************************************
#define bSAU_CKS            BIT15 | BIT5
#define SAU_CLK_EXT         BIT14 | BIT5
#define bSAU_STS            BIT8 | BIT5
#define SAU_RX_INV          BIT6 | BIT5
#define SAU_MD_CSI          BIT5
#define SAU_MD_UART         BIT1 | BIT5
#define SAU_MD_I2C          BIT2 | BIT5
#define SAU_INT_BUFFER      BIT0 | BIT5

// SOm *************************************************************************
#define SAU_CKO3            BIT11
#define SAU_CKO2            BIT10
#define SAU_CKO1            BIT9
#define SAU_CKO0            BIT8

// SPSm ************************************************************************
#define SAU_CK1_DIV1        0
#define SAU_CK1_DIV2        1<<4
#define SAU_CK1_DIV4        2<<4
#define SAU_CK1_DIV8        3<<4
#define SAU_CK1_DIV16       4<<4
#define SAU_CK1_DIV32       5<<4
#define SAU_CK1_DIV64       6<<4
#define SAU_CK1_DIV128      7<<4
#define SAU_CK1_DIV256      8<<4
#define SAU_CK1_DIV512      9<<4
#define SAU_CK1_DIV1024     10<<4
#define SAU_CK1_DIV2048     11<<4
#define SAU_CK1_DIV4096     12<<4
#define SAU_CK1_DIV8192     13<<4
#define SAU_CK1_DIV16384    14<<4
#define SAU_CK1_DIV32768    15<<4
#define SAU_CK0_DIV1        0
#define SAU_CK0_DIV2        1
#define SAU_CK0_DIV4        2
#define SAU_CK0_DIV8        3
#define SAU_CK0_DIV16       4
#define SAU_CK0_DIV32       5
#define SAU_CK0_DIV64       6
#define SAU_CK0_DIV128      7
#define SAU_CK0_DIV256      8
#define SAU_CK0_DIV512      9
#define SAU_CK0_DIV1024     10
#define SAU_CK0_DIV2048     11
#define SAU_CK0_DIV4096     12
#define SAU_CK0_DIV8192     13
#define SAU_CK0_DIV16384    14
#define SAU_CK0_DIV32768    15

// SSm *************************************************************************
#define SAU_CH3             BIT3
#define SAU_CH2             BIT2
#define SAU_CH1             BIT1
#define SAU_CH0             BIT0

// SSCm ************************************************************************
#define bSSEC               BIT1
#define bSWC                BIT0

// SSRmn ***********************************************************************
#define bTSF                BIT6
#define bBFF                BIT5
#define bFEF                BIT2
#define bPEF                BIT1
#define bOVF                BIT0


// TMRmn ***********************************************************************
#define TAU_CK0             0
#define TAU_CK2             BIT14
#define TAU_CK1             BIT15
#define TAU_CK3             BIT15 | BIT14
#define bCCS                BIT12
#define bTAU_MASTER         BIT11
#define bTAU_8BIT           BIT11
#define TAU_TRIG_SOFT       0
#define TAU_TRIG_VALID_EDGE BIT8
#define TAU_TRIG_BOTH_EDGE  BIT9
#define TAU_TRIG_MASTER     BIT10
#define TAU_EDGE_FALLING    0
#define TAU_EDGE_RISING     BIT6
#define TAU_EDGE_FALL_RISE  BIT7
#define TAU_EDGE_BOTH       BIT7
#define TAU_EDGE_RISE_FALL  BIT7 | BIT6
#define TAU_MD_TIMER              0
#define TAU_MD_TIMER_TRIG_INT     1
#define TAU_MD_CAPTURE            4
#define TAU_MD_CAPTURE_TRIG_INT   5
#define TAU_MD_EVENTCOUNT         6
#define TAU_MD_ONECOUNT           8
#define TAU_MD_ONECOUNT_TRIG      9
#define TAU_MD_CAPTURE_LEVEL      0xC

// TPSm ************************************************************************
#define TAU_CK3_DIV256      0
#define TAU_CK3_DIV1024     BIT12
#define TAU_CK3_DIV4096     BIT13
#define TAU_CK3_DIV16384    BIT13 | BIT12
#define TAU_CK2_DIV2        0
#define TAU_CK2_DIV4        BIT8
#define TAU_CK2_DIV16       BIT9
#define TAU_CK2_DIV64       BIT9 | BIT8
#define TAU_CK1_DIV1        0
#define TAU_CK1_DIV2        1<<4
#define TAU_CK1_DIV4        2<<4
#define TAU_CK1_DIV8        3<<4
#define TAU_CK1_DIV16       4<<4
#define TAU_CK1_DIV32       5<<4
#define TAU_CK1_DIV64       6<<4
#define TAU_CK1_DIV128      7<<4
#define TAU_CK1_DIV256      8<<4
#define TAU_CK1_DIV512      9<<4
#define TAU_CK1_DIV1024     10<<4
#define TAU_CK1_DIV2048     11<<4
#define TAU_CK1_DIV4096     12<<4
#define TAU_CK1_DIV8192     13<<4
#define TAU_CK1_DIV16384    14<<4
#define TAU_CK1_DIV32768    15<<4
#define TAU_CK0_DIV1        0
#define TAU_CK0_DIV2        1
#define TAU_CK0_DIV4        2
#define TAU_CK0_DIV8        3
#define TAU_CK0_DIV16       4
#define TAU_CK0_DIV32       5
#define TAU_CK0_DIV64       6
#define TAU_CK0_DIV128      7
#define TAU_CK0_DIV256      8
#define TAU_CK0_DIV512      9
#define TAU_CK0_DIV1024     10
#define TAU_CK0_DIV2048     11
#define TAU_CK0_DIV4096     12
#define TAU_CK0_DIV8192     13
#define TAU_CK0_DIV16384    14
#define TAU_CK0_DIV32768    15

// TEm, TSm, TTm, **************************************************************
#define TAU_CH0             BIT0
#define TAU_CH1             BIT1
#define TAU_CH2             BIT2
#define TAU_CH3             BIT3
#define TAU_CH4             BIT4
#define TAU_CH5             BIT5
#define TAU_CH6             BIT6
#define TAU_CH7             BIT7
#define bTSH1               BIT9
#define bTSH3               BIT11
