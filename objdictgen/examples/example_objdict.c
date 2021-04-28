
/* File generated by gen_cfile.py. Should not be modified. */

#include "example_objdict.h"

/**************************************************************************/
/* Declaration of mapped variables                                        */
/**************************************************************************/
UNS8 Time_seconds = 0x0;		/* Mapped at index 0x2000, subindex 0x01 */
UNS8 Time_minutes = 0x0;		/* Mapped at index 0x2000, subindex 0x02 */
UNS8 Time_hours = 0x0;		/* Mapped at index 0x2000, subindex 0x03 */
UNS8 Time_days = 0x0;		/* Mapped at index 0x2000, subindex 0x04 */
UNS32 canopenErrNB = 0x0;		/* Mapped at index 0x2001, subindex 0x00 */
UNS32 canopenErrVal = 0x0;		/* Mapped at index 0x2002, subindex 0x00 */
INTEGER8 strTest[10] = "";		/* Mapped at index 0x2003, subindex 0x00 */

/**************************************************************************/
/* Declaration of value range types                                       */
/**************************************************************************/

#define valueRange_EMC 0x9F /* Type for index 0x1003 subindex 0x00 (only set of value 0 is possible) */
UNS32 Linux_slave_valueRangeTest (UNS8 typeValue, void * value)
{
  switch (typeValue) {
    case valueRange_EMC:
      if (*(UNS8*)value != (UNS8)0) return OD_VALUE_RANGE_EXCEEDED;
      break;
  }
  return 0;
}

/**************************************************************************/
/* The node id                                                            */
/**************************************************************************/
/* node_id default value.*/
UNS8 Linux_slave_bDeviceNodeId = 0x00;

/**************************************************************************/
/* Array of message processing information */

const UNS8 Linux_slave_iam_a_slave = 1;

TIMER_HANDLE Linux_slave_heartBeatTimers[1] = {TIMER_NONE};

/*
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

                               OBJECT DICTIONARY

$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/

/* index 0x1000 :   Device Type. */
                    UNS32 Linux_slave_obj1000 = 0x0;	/* 0 */
                    subindex Linux_slave_Index1000[] = 
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1000 }
                     };

/* index 0x1001 :   Error Register. */
                    UNS8 Linux_slave_obj1001 = 0x0;	/* 0 */
                    subindex Linux_slave_Index1001[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1001 }
                     };

/* index 0x1003 :   Pre-defined Error Field */
                    UNS8 Linux_slave_highestSubIndex_obj1003 = 0; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1003[] = 
                    {
                      0x0	/* 0 */
                    };
                    ODCallback_t Linux_slave_Index1003_callbacks[] = 
                     {
                       NULL,
                       NULL,
                     };
                    subindex Linux_slave_Index1003[] = 
                     {
                       { RW, valueRange_EMC, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1003 },
                       { RO, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1003[0] }
                     };

/* index 0x1005 :   SYNC COB ID */
                    UNS32 Linux_slave_obj1005 = 0x0;   /* 0 */

/* index 0x1006 :   Communication / Cycle Period */
                    UNS32 Linux_slave_obj1006 = 0x0;   /* 0 */

/* index 0x1008 :   Manufacturer Device Name. */
                    INTEGER8 Linux_slave_obj1008[16] = "Appli_Slave_HC12";
                    subindex Linux_slave_Index1008[] = 
                     {
                       { RO, visible_string, 16, (void*)&Linux_slave_obj1008 }
                     };

/* index 0x1014 :   Emergency COB ID */
                    UNS32 Linux_slave_obj1014 = 0x80 + 0x00;   /* 128 + NodeID */

/* index 0x1016 :   Consumer Heartbeat Time. */
                    UNS8 Linux_slave_highestSubIndex_obj1016 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1016[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1016[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1016 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1016[0] }
                     };

/* index 0x1017 :   Producer Heartbeat Time */ 
                    UNS16 Linux_slave_obj1017 = 0x0;   /* 0 */

/* index 0x1018 :   Identity. */
                    UNS8 Linux_slave_highestSubIndex_obj1018 = 4; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1018_Vendor_ID = 0x0;	/* 0 */
                    UNS32 Linux_slave_obj1018_Product_Code = 0x0;	/* 0 */
                    UNS32 Linux_slave_obj1018_Revision_Number = 0x0;	/* 0 */
                    UNS32 Linux_slave_obj1018_Serial_Number = 0x0;	/* 0 */
                    subindex Linux_slave_Index1018[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1018 },
                       { RO, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1018_Vendor_ID },
                       { RO, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1018_Product_Code },
                       { RO, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1018_Revision_Number },
                       { RO, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1018_Serial_Number }
                     };

/* index 0x1280 :   Client SDO 1 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1280 = 3; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1280_COB_ID_Client_to_Server_Transmit_SDO = 0x0;	/* 0 */
                    UNS32 Linux_slave_obj1280_COB_ID_Server_to_Client_Receive_SDO = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1280_Node_ID_of_the_SDO_Server = 0x0;	/* 0 */
                    subindex Linux_slave_Index1280[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1280 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1280_COB_ID_Client_to_Server_Transmit_SDO },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1280_COB_ID_Server_to_Client_Receive_SDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1280_Node_ID_of_the_SDO_Server }
                     };

/* index 0x1400 :   Receive PDO 1 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1400 = 5; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1400_COB_ID_used_by_PDO = 0x200;	/* 512 */
                    UNS8 Linux_slave_obj1400_Transmission_Type = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1400_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1400_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1400_Event_Timer = 0x0;	/* 0 */
                    subindex Linux_slave_Index1400[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1400 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1400_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1400_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1400_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1400_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1400_Event_Timer }
                     };

/* index 0x1401 :   Receive PDO 2 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1401 = 5; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1401_COB_ID_used_by_PDO = 0x300;	/* 768 */
                    UNS8 Linux_slave_obj1401_Transmission_Type = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1401_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1401_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1401_Event_Timer = 0x0;	/* 0 */
                    subindex Linux_slave_Index1401[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1401 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1401_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1401_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1401_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1401_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1401_Event_Timer }
                     };

/* index 0x1402 :   Receive PDO 3 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1402 = 5; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1402_COB_ID_used_by_PDO = 0x400;	/* 1024 */
                    UNS8 Linux_slave_obj1402_Transmission_Type = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1402_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1402_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1402_Event_Timer = 0x0;	/* 0 */
                    subindex Linux_slave_Index1402[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1402 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1402_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1402_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1402_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1402_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1402_Event_Timer }
                     };

/* index 0x1403 :   Receive PDO 4 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1403 = 5; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1403_COB_ID_used_by_PDO = 0x500;	/* 1280 */
                    UNS8 Linux_slave_obj1403_Transmission_Type = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1403_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1403_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1403_Event_Timer = 0x0;	/* 0 */
                    subindex Linux_slave_Index1403[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1403 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1403_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1403_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1403_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1403_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1403_Event_Timer }
                     };

/* index 0x1600 :   Receive PDO 1 Mapping. */
                    UNS8 Linux_slave_highestSubIndex_obj1600 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1600[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1600[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1600 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1600[0] }
                     };

/* index 0x1601 :   Receive PDO 2 Mapping. */
                    UNS8 Linux_slave_highestSubIndex_obj1601 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1601[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1601[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1601 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1601[0] }
                     };

/* index 0x1602 :   Receive PDO 3 Mapping. */
                    UNS8 Linux_slave_highestSubIndex_obj1602 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1602[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1602[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1602 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1602[0] }
                     };

/* index 0x1603 :   Receive PDO 4 Mapping. */
                    UNS8 Linux_slave_highestSubIndex_obj1603 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1603[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1603[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1603 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1603[0] }
                     };

/* index 0x1800 :   Transmit PDO 1 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1800 = 5; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1800_COB_ID_used_by_PDO = 0x180;	/* 384 */
                    UNS8 Linux_slave_obj1800_Transmission_Type = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1800_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1800_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1800_Event_Timer = 0x0;	/* 0 */
                    ODCallback_t Linux_slave_Index1800_callbacks[] = 
                     {
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                     };
                    subindex Linux_slave_Index1800[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1800 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1800_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1800_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1800_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1800_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1800_Event_Timer }
                     };

/* index 0x1801 :   Transmit PDO 2 Parameter. */
                    UNS8 Linux_slave_highestSubIndex_obj1801 = 5; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1801_COB_ID_used_by_PDO = 0x280;	/* 640 */
                    UNS8 Linux_slave_obj1801_Transmission_Type = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1801_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Linux_slave_obj1801_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Linux_slave_obj1801_Event_Timer = 0x0;	/* 0 */
                    ODCallback_t Linux_slave_Index1801_callbacks[] = 
                     {
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                     };
                    subindex Linux_slave_Index1801[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1801 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1801_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1801_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1801_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_obj1801_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&Linux_slave_obj1801_Event_Timer }
                     };

/* index 0x1A00 :   Transmit PDO 1 Mapping. */
                    UNS8 Linux_slave_highestSubIndex_obj1A00 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1A00[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1A00[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1A00 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1A00[0] }
                     };

/* index 0x1A01 :   Transmit PDO 2 Mapping. */
                    UNS8 Linux_slave_highestSubIndex_obj1A01 = 1; /* number of subindex - 1*/
                    UNS32 Linux_slave_obj1A01[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex Linux_slave_Index1A01[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj1A01 },
                       { RW, uint32, sizeof (UNS32), (void*)&Linux_slave_obj1A01[0] }
                     };

/* index 0x2000 :   Mapped variable Time */
                    UNS8 Linux_slave_highestSubIndex_obj2000 = 4; /* number of subindex - 1*/
                    subindex Linux_slave_Index2000[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Linux_slave_highestSubIndex_obj2000 },
                       { RW, uint8, sizeof (UNS8), (void*)&Time_seconds },
                       { RW, uint8, sizeof (UNS8), (void*)&Time_minutes },
                       { RW, uint8, sizeof (UNS8), (void*)&Time_hours },
                       { RW, uint8, sizeof (UNS8), (void*)&Time_days }
                     };

/* index 0x2001 :   Mapped variable canopenErrNB */
                    subindex Linux_slave_Index2001[] = 
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&canopenErrNB }
                     };

/* index 0x2002 :   Mapped variable canopenErrVal */
                    subindex Linux_slave_Index2002[] = 
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&canopenErrVal }
                     };

/* index 0x2003 :   Mapped variable strTest */
                    subindex Linux_slave_Index2003[] = 
                     {
                       { RW, visible_string, 10, (void*)&strTest }
                     };

/**************************************************************************/
/* Declaration of pointed variables                                       */
/**************************************************************************/

const indextable Linux_slave_objdict[] = 
{
  { (subindex*)Linux_slave_Index1000,sizeof(Linux_slave_Index1000)/sizeof(Linux_slave_Index1000[0]), 0x1000},
  { (subindex*)Linux_slave_Index1001,sizeof(Linux_slave_Index1001)/sizeof(Linux_slave_Index1001[0]), 0x1001},
  { (subindex*)Linux_slave_Index1008,sizeof(Linux_slave_Index1008)/sizeof(Linux_slave_Index1008[0]), 0x1008},
  { (subindex*)Linux_slave_Index1016,sizeof(Linux_slave_Index1016)/sizeof(Linux_slave_Index1016[0]), 0x1016},
  { (subindex*)Linux_slave_Index1018,sizeof(Linux_slave_Index1018)/sizeof(Linux_slave_Index1018[0]), 0x1018},
  { (subindex*)Linux_slave_Index1280,sizeof(Linux_slave_Index1280)/sizeof(Linux_slave_Index1280[0]), 0x1280},
  { (subindex*)Linux_slave_Index1400,sizeof(Linux_slave_Index1400)/sizeof(Linux_slave_Index1400[0]), 0x1400},
  { (subindex*)Linux_slave_Index1401,sizeof(Linux_slave_Index1401)/sizeof(Linux_slave_Index1401[0]), 0x1401},
  { (subindex*)Linux_slave_Index1402,sizeof(Linux_slave_Index1402)/sizeof(Linux_slave_Index1402[0]), 0x1402},
  { (subindex*)Linux_slave_Index1403,sizeof(Linux_slave_Index1403)/sizeof(Linux_slave_Index1403[0]), 0x1403},
  { (subindex*)Linux_slave_Index1600,sizeof(Linux_slave_Index1600)/sizeof(Linux_slave_Index1600[0]), 0x1600},
  { (subindex*)Linux_slave_Index1601,sizeof(Linux_slave_Index1601)/sizeof(Linux_slave_Index1601[0]), 0x1601},
  { (subindex*)Linux_slave_Index1602,sizeof(Linux_slave_Index1602)/sizeof(Linux_slave_Index1602[0]), 0x1602},
  { (subindex*)Linux_slave_Index1603,sizeof(Linux_slave_Index1603)/sizeof(Linux_slave_Index1603[0]), 0x1603},
  { (subindex*)Linux_slave_Index1800,sizeof(Linux_slave_Index1800)/sizeof(Linux_slave_Index1800[0]), 0x1800},
  { (subindex*)Linux_slave_Index1801,sizeof(Linux_slave_Index1801)/sizeof(Linux_slave_Index1801[0]), 0x1801},
  { (subindex*)Linux_slave_Index1A00,sizeof(Linux_slave_Index1A00)/sizeof(Linux_slave_Index1A00[0]), 0x1A00},
  { (subindex*)Linux_slave_Index1A01,sizeof(Linux_slave_Index1A01)/sizeof(Linux_slave_Index1A01[0]), 0x1A01},
  { (subindex*)Linux_slave_Index2000,sizeof(Linux_slave_Index2000)/sizeof(Linux_slave_Index2000[0]), 0x2000},
  { (subindex*)Linux_slave_Index2001,sizeof(Linux_slave_Index2001)/sizeof(Linux_slave_Index2001[0]), 0x2001},
  { (subindex*)Linux_slave_Index2002,sizeof(Linux_slave_Index2002)/sizeof(Linux_slave_Index2002[0]), 0x2002},
  { (subindex*)Linux_slave_Index2003,sizeof(Linux_slave_Index2003)/sizeof(Linux_slave_Index2003[0]), 0x2003},
};

const indextable * Linux_slave_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks)
{
	int i;
	*callbacks = NULL;
	switch(wIndex){
		case 0x1000: i = 0;break;
		case 0x1001: i = 1;break;
		case 0x1008: i = 2;break;
		case 0x1016: i = 3;break;
		case 0x1018: i = 4;break;
		case 0x1280: i = 5;break;
		case 0x1400: i = 6;break;
		case 0x1401: i = 7;break;
		case 0x1402: i = 8;break;
		case 0x1403: i = 9;break;
		case 0x1600: i = 10;break;
		case 0x1601: i = 11;break;
		case 0x1602: i = 12;break;
		case 0x1603: i = 13;break;
		case 0x1800: i = 14;*callbacks = Linux_slave_Index1800_callbacks; break;
		case 0x1801: i = 15;*callbacks = Linux_slave_Index1801_callbacks; break;
		case 0x1A00: i = 16;break;
		case 0x1A01: i = 17;break;
		case 0x2000: i = 18;break;
		case 0x2001: i = 19;break;
		case 0x2002: i = 20;break;
		case 0x2003: i = 21;break;
		default:
			*errorCode = OD_NO_SUCH_OBJECT;
			return NULL;
	}
	*errorCode = OD_SUCCESSFUL;
	return &Linux_slave_objdict[i];
}

/* 
 * To count at which received SYNC a PDO must be sent.
 * Even if no pdoTransmit are defined, at least one entry is computed
 * for compilations issues.
 */
s_PDO_status Linux_slave_PDO_status[2] = {s_PDO_status_Initializer,s_PDO_status_Initializer};

const quick_index Linux_slave_firstIndex = {
  0, /* SDO_SVR */
  5, /* SDO_CLT */
  6, /* PDO_RCV */
  10, /* PDO_RCV_MAP */
  14, /* PDO_TRS */
  16 /* PDO_TRS_MAP */
};

const quick_index Linux_slave_lastIndex = {
  0, /* SDO_SVR */
  5, /* SDO_CLT */
  9, /* PDO_RCV */
  13, /* PDO_RCV_MAP */
  15, /* PDO_TRS */
  17 /* PDO_TRS_MAP */
};

const UNS16 Linux_slave_ObjdictSize = sizeof(Linux_slave_objdict)/sizeof(Linux_slave_objdict[0]); 

CO_Data Linux_slave_Data = CANOPEN_NODE_DATA_INITIALIZER(Linux_slave);
