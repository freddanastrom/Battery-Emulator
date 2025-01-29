#include "../include.h"
#ifdef SOLAX_CAN
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "SOLAX-CAN.h"

#define NUMBER_OF_MODULES 0
#define BATTERY_TYPE 0x50
// If you are having BattVoltFault issues, configure the above values according to wiki page
// https://github.com/dalathegreat/Battery-Emulator/wiki/Solax-inverters

/* Do not change code below unless you are sure what you are doing */
static int16_t temperature_average = 0;
static uint8_t STATE = BATTERY_ANNOUNCE;
static unsigned long LastFrameTime = 0;
static uint8_t number_of_batteries = 1;
static uint16_t capped_capacity_Wh;
static uint16_t capped_remaining_capacity_Wh;
static uint8_t rotate_1877 = 0;
static uint8_t rotate_187B = 0;
static uint8_t rotate_187C = 1;

//CAN message translations from this amazing repository: https://github.com/rand12345/solax_can_bus

CAN_frame SOLAX_1801 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1801,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_1872 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1872,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_Limits
CAN_frame SOLAX_1873 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1873,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_PackData
CAN_frame SOLAX_1874 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1874,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_CellData
CAN_frame SOLAX_1875 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1875,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_Status
CAN_frame SOLAX_1876 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1876,
                        .data = {0x0, 0x0, 0xE2, 0x0C, 0x0, 0x0, 0xD7, 0x0C}};  //BMS_PackTemps
CAN_frame SOLAX_1877 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1877,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_1878 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1878,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_PackStats
CAN_frame SOLAX_1879 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1879,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_187E = {.FD = false,  //Needed for Ultra
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x187E,
                        .data = {0x60, 0xEA, 0x0, 0x0, 0x64, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_187D = {.FD = false,  //Needed for Ultra
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x187D,
                        .data = {0x8B, 0x01, 0x0, 0x0, 0x8B, 0x1, 0x0, 0x0}};
CAN_frame SOLAX_187C = {.FD = false,  //Needed for Ultra
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x187C,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_187B = {.FD = false,  //Needed for Ultra
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x187B,
                        .data = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_187A = {.FD = false,  //Needed for Ultra
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x187A,
                        .data = {0x01, 0x50, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
CAN_frame SOLAX_1881 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1881,
                        .data = {0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  // E.g.: 0 6 S B M S F A
CAN_frame SOLAX_1882 = {.FD = false,
                        .ext_ID = true,
                        .DLC = 8,
                        .ID = 0x1882,
                        .data = {0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  // E.g.: 0 2 3 A B 0 5 2
CAN_frame SOLAX_100A001 = {.FD = false, .ext_ID = true, .DLC = 0, .ID = 0x100A001, .data = {}};

// __builtin_bswap64 needed to convert to ESP32 little endian format
// Byte[4] defines the requested contactor state: 1 = Closed , 0 = Open
#define Contactor_Open_Payload __builtin_bswap64(0x0200010000000000)
#define Contactor_Close_Payload __builtin_bswap64(0x0200010001000000)

void update_values_can_inverter() {  //This function maps all the values fetched from battery CAN to the correct CAN messages
  // If not receiveing any communication from the inverter, open contactors and return to battery announce state
  if (millis() - LastFrameTime >= SolaxTimeout) {
    datalayer.system.status.inverter_allows_contactor_closing = false;
    STATE = BATTERY_ANNOUNCE;
  }
  //Calculate the required values
  temperature_average =
      ((datalayer.battery.status.temperature_max_dC + datalayer.battery.status.temperature_min_dC) / 2);

  // Batteries might be larger than uint16_t value can take
  if (datalayer.battery.info.total_capacity_Wh > 65000) {
    capped_capacity_Wh = 65000;
  } else {
    capped_capacity_Wh = datalayer.battery.info.total_capacity_Wh;
  }
  // Batteries might be larger than uint16_t value can take
  if (datalayer.battery.status.reported_remaining_capacity_Wh > 65000) {
    capped_remaining_capacity_Wh = 65000;
  } else {
    capped_remaining_capacity_Wh = datalayer.battery.status.reported_remaining_capacity_Wh;
  }

  //Put the values into the CAN messages
  //BMS_Limits
  SOLAX_1872.data.u8[0] = (uint8_t)datalayer.battery.info.max_design_voltage_dV;
  SOLAX_1872.data.u8[1] = (datalayer.battery.info.max_design_voltage_dV >> 8);
  SOLAX_1872.data.u8[2] = (uint8_t)datalayer.battery.info.min_design_voltage_dV;
  SOLAX_1872.data.u8[3] = (datalayer.battery.info.min_design_voltage_dV >> 8);
  SOLAX_1872.data.u8[4] = (uint8_t)datalayer.battery.status.max_charge_current_dA;
  SOLAX_1872.data.u8[5] = (datalayer.battery.status.max_charge_current_dA >> 8);
  SOLAX_1872.data.u8[6] = (uint8_t)datalayer.battery.status.max_discharge_current_dA;
  SOLAX_1872.data.u8[7] = (datalayer.battery.status.max_discharge_current_dA >> 8);

  //BMS_PackData
  SOLAX_1873.data.u8[0] = (uint8_t)datalayer.battery.status.voltage_dV;  // OK
  SOLAX_1873.data.u8[1] = (datalayer.battery.status.voltage_dV >> 8);
  SOLAX_1873.data.u8[2] = (int8_t)datalayer.battery.status.current_dA;  // OK, Signed (Active current in Amps x 10)
  SOLAX_1873.data.u8[3] = (datalayer.battery.status.current_dA >> 8);
  SOLAX_1873.data.u8[4] = (uint8_t)(datalayer.battery.status.reported_soc / 100);  //SOC (100.00%)
  //SOLAX_1873.data.u8[5] = //Seems like this is not required? Or shall we put SOC decimals here?
  SOLAX_1873.data.u8[6] = (uint8_t)(capped_remaining_capacity_Wh / 10);
  SOLAX_1873.data.u8[7] = ((capped_remaining_capacity_Wh / 10) >> 8);

  //BMS_CellData
  SOLAX_1874.data.u8[0] = (int8_t)datalayer.battery.status.temperature_max_dC;
  SOLAX_1874.data.u8[1] = (datalayer.battery.status.temperature_max_dC >> 8);
  SOLAX_1874.data.u8[2] = (int8_t)datalayer.battery.status.temperature_min_dC;
  SOLAX_1874.data.u8[3] = (datalayer.battery.status.temperature_min_dC >> 8);
  SOLAX_1874.data.u8[4] = (uint8_t)(datalayer.battery.info.max_cell_voltage_mV);
  SOLAX_1874.data.u8[5] = (datalayer.battery.info.max_cell_voltage_mV >> 8);
  SOLAX_1874.data.u8[6] = (uint8_t)(datalayer.battery.status.cell_min_voltage_mV);
  SOLAX_1874.data.u8[7] = (datalayer.battery.status.cell_min_voltage_mV >> 8);

  //BMS_Status
  SOLAX_1875.data.u8[0] = (uint8_t)temperature_average;
  SOLAX_1875.data.u8[1] = (temperature_average >> 8);
  SOLAX_1875.data.u8[2] = (uint8_t)NUMBER_OF_MODULES;  // Number of slave batteries
  SOLAX_1875.data.u8[4] = (uint8_t)0;                  // Contactor Status 0=off, 1=on.

  //BMS_PackTemps (strange name, since it has voltages?)
  SOLAX_1876.data.u8[0] = (int8_t)datalayer.battery.status.temperature_max_dC;
  SOLAX_1876.data.u8[1] = (datalayer.battery.status.temperature_max_dC >> 8);
  SOLAX_1876.data.u8[2] = (uint8_t)datalayer.battery.status.cell_max_voltage_mV;
  SOLAX_1876.data.u8[3] = (datalayer.battery.status.cell_max_voltage_mV >> 8);

  SOLAX_1876.data.u8[4] = (int8_t)datalayer.battery.status.temperature_min_dC;
  SOLAX_1876.data.u8[5] = (datalayer.battery.status.temperature_min_dC >> 8);
  SOLAX_1876.data.u8[6] = (uint8_t)datalayer.battery.status.cell_min_voltage_mV;
  SOLAX_1876.data.u8[7] = (datalayer.battery.status.cell_min_voltage_mV >> 8);

  //Unknown
  SOLAX_1877.data.u8[4] = (uint8_t)BATTERY_TYPE;  // Battery type (Default 0x50)
  if (BATTERY_TYPE == 0x5A)
  {
    // 1877 byte 7 is rotatating 01-10-20-30-40
    // On type 0x5A it seems like the master BMS is 01, not 02. Byte 6 = 0x06
    // 10-40 maybe modules. Byte 6 = 0x16
    switch (rotate_1877) {
      case 0:
        //BMS
        SOLAX_1877.data.u8[6] = (uint8_t)0x06;          // Firmware version?
        SOLAX_1877.data.u8[7] = (uint8_t)0x01;

        rotate_1877++;
        break;
      default:
        //Modules
        SOLAX_1877.data.u8[6] = (uint8_t)0x16;          // Firmware version?
        SOLAX_1877.data.u8[7] = rotate_1877 * 10;

        if (rotate_1877 >= NUMBER_OF_MODULES)
          rotate_1877 = 0;  // Reset rotation
        else
          rotate_1877++;
    }
  }
  else
  {
    SOLAX_1877.data.u8[6] = (uint8_t)0x22;          // Firmware version?
    SOLAX_1877.data.u8[7] =
    (uint8_t)0x02;  // The above firmware version applies to:02 = Master BMS, 10 = S1, 20 = S2, 30 = S3, 40 = S4
  }

  //BMS_PackStats
  SOLAX_1878.data.u8[0] = (uint8_t)(datalayer.battery.status.voltage_dV);
  SOLAX_1878.data.u8[1] = ((datalayer.battery.status.voltage_dV) >> 8);
  
  SOLAX_1878.data.u8[4] = (uint8_t)datalayer.battery.info.total_capacity_Wh; // Correct data should be the sum of total charged and discharged Wh from battery, but doesn't seem to matter.
  SOLAX_1878.data.u8[5] = (datalayer.battery.info.total_capacity_Wh >> 8);
  SOLAX_1878.data.u8[6] = (datalayer.battery.info.total_capacity_Wh >> 16);
  SOLAX_1878.data.u8[7] = (datalayer.battery.info.total_capacity_Wh >> 24);

  // BMS_Answer
  SOLAX_1801.data.u8[0] = 2;
  SOLAX_1801.data.u8[2] = 1;
  SOLAX_1801.data.u8[4] = 1;

  //Ultra messages
  if (BATTERY_TYPE == 0x5A)
  {
    // This is values that differs from the standard ones defined in Ultra messages above.
    // To avoid messing up other battery/inverter combinations this is restricted to battery type 0x5A for now
    // Values used here are logged from a S36 battery with 4 modules where frame 1877 byte 4 = 0x5A and Ultra 25K
    //
    // Frames in log that still need some investigation:
    // 0C0D Unknown source. Always same timestamp as 1300. Byte 0 and 7 changes e.g. 15 3F 05 3F 07 00 00 F4
    // 1300 From inverter, unknown. Byte 1-6 changes e.g. 00 10 10 5A 00 16 20 88
    // 1802 Unknown source. Byte 0 changes between 00 and 05, e.g. 05 00 00 00 00 00 00 00
    // 1871 From inverter. More data available in request from inverter that isn´t used in the emulator
    // 1879 Unknown source, byte 0-3 changes 01 0A 03 00 01 02 03 01
    // 1890 Unknown source. Static 10 00 12 00 00 00 00 00. Appears in front of every 187A-E frame, request from inverter? 
    
    // Some unknown fixed values, byte 2 is the same as max charge current * 10.
    SOLAX_187A.data.u8[1] = (uint8_t)0x70;
    SOLAX_187A.data.u8[2] = (uint8_t)datalayer.battery.status.max_charge_current_dA * 10;
    SOLAX_187A.data.u8[4] = (uint8_t)0xE0;
    SOLAX_187A.data.u8[5] = (uint8_t)0x01;
    SOLAX_187A.data.u8[7] = (uint8_t)0x08;

    SOLAX_187B.data.u8[0] = rotate_187B; // Rotating 00-01-02-03-04. Maybe BMS and modules?
    SOLAX_187B.data.u8[3] = (uint8_t)0x10;
    SOLAX_187B.data.u8[6] = (uint8_t)0x48;

    if (rotate_187B >= NUMBER_OF_MODULES)
      rotate_187B = 0;  // Reset rotation
    else
      rotate_187B++;

    SOLAX_187C.data.u8[0] = rotate_187C; // Rotating 01-02-03-04. Maybe modules?
    //SOLAX_187C.data.u8[1] = (uint8_t)0x88; // Values are changing 28-168, unknown for now
    //SOLAX_187C.data.u8[2] = (uint8_t)0x13; // Values are changing 19-20, unknown for now
    //SOLAX_187C.data.u8[3] = (uint8_t)0x00; // Values are changing 87-90, unknown for now. Module SOC?
    if (rotate_187C >= NUMBER_OF_MODULES)
      rotate_187C = 1;  // Reset rotation
    else
      rotate_187C++;

    // Fixed values, but not the same as defined in Ultra messages above.
    SOLAX_187D.data.u8[0] = (uint8_t)0x80;
    SOLAX_187D.data.u8[1] = (uint8_t)0x0C;
    SOLAX_187D.data.u8[4] = (uint8_t)0x55;
    SOLAX_187D.data.u8[5] = (uint8_t)0x0C;

    SOLAX_187E.data.u8[4] = (uint8_t)(datalayer.battery.status.soh_pptt / 100);
  }

  SOLAX_187E.data.u8[0] = (uint8_t)datalayer.battery.status.reported_remaining_capacity_Wh;
  SOLAX_187E.data.u8[1] = (datalayer.battery.status.reported_remaining_capacity_Wh >> 8);
  SOLAX_187E.data.u8[2] = (datalayer.battery.status.reported_remaining_capacity_Wh >> 16);
  SOLAX_187E.data.u8[3] = (datalayer.battery.status.reported_remaining_capacity_Wh >> 24);
  SOLAX_187E.data.u8[5] = (uint8_t)(datalayer.battery.status.reported_soc / 100);
}

void transmit_can_inverter() {
  // No periodic sending used on this protocol, we react only on incoming CAN messages!
}

void map_can_frame_to_variable_inverter(CAN_frame rx_frame) {

  if (rx_frame.ID == 0x1871) {
    datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
  }

  if (rx_frame.ID == 0x1871 && rx_frame.data.u8[0] == (0x01) ||
      rx_frame.ID == 0x1871 && rx_frame.data.u8[0] == (0x02)) {
    LastFrameTime = millis();
    switch (STATE) {
      case (BATTERY_ANNOUNCE):
#ifdef DEBUG_LOG
        logging.println("Solax Battery State: Announce");
#endif
        datalayer.system.status.inverter_allows_contactor_closing = false;
        SOLAX_1875.data.u8[4] = (0x00);  // Inform Inverter: Contactor 0=off, 1=on.
        for (uint8_t i = 0; i <= number_of_batteries; i++) {
          transmit_can_frame(&SOLAX_187E, can_config.inverter);
          if (BATTERY_TYPE == 0x5A)
          {
            transmit_can_frame(&SOLAX_187D, can_config.inverter);
            transmit_can_frame(&SOLAX_187C, can_config.inverter);
            transmit_can_frame(&SOLAX_187B, can_config.inverter);
          }
          transmit_can_frame(&SOLAX_187A, can_config.inverter);
          transmit_can_frame(&SOLAX_1872, can_config.inverter);
          transmit_can_frame(&SOLAX_1873, can_config.inverter);
          transmit_can_frame(&SOLAX_1874, can_config.inverter);
          transmit_can_frame(&SOLAX_1875, can_config.inverter);
          transmit_can_frame(&SOLAX_1876, can_config.inverter);
          transmit_can_frame(&SOLAX_1877, can_config.inverter);
          transmit_can_frame(&SOLAX_1878, can_config.inverter);
        }
        transmit_can_frame(&SOLAX_100A001, can_config.inverter);  //BMS Announce
        // Message from the inverter to proceed to contactor closing
        // Byte 4 changes from 0 to 1
        if (rx_frame.data.u64 == Contactor_Close_Payload)
          STATE = WAITING_FOR_CONTACTOR;
        break;

      case (WAITING_FOR_CONTACTOR):
        SOLAX_1875.data.u8[4] = (0x00);  // Inform Inverter: Contactor 0=off, 1=on.
        transmit_can_frame(&SOLAX_187E, can_config.inverter);
        if (BATTERY_TYPE == 0x5A)
          {
            transmit_can_frame(&SOLAX_187D, can_config.inverter);
            transmit_can_frame(&SOLAX_187C, can_config.inverter);
            transmit_can_frame(&SOLAX_187B, can_config.inverter);
          }
        transmit_can_frame(&SOLAX_187A, can_config.inverter);
        transmit_can_frame(&SOLAX_1872, can_config.inverter);
        transmit_can_frame(&SOLAX_1873, can_config.inverter);
        transmit_can_frame(&SOLAX_1874, can_config.inverter);
        transmit_can_frame(&SOLAX_1875, can_config.inverter);
        transmit_can_frame(&SOLAX_1876, can_config.inverter);
        transmit_can_frame(&SOLAX_1877, can_config.inverter);
        transmit_can_frame(&SOLAX_1878, can_config.inverter);
        if (BATTERY_TYPE != 0x5A) // Cant see this at all in the logs, trying to omit.
          transmit_can_frame(&SOLAX_1801, can_config.inverter);  // Announce that the battery will be connected
        STATE = CONTACTOR_CLOSED;                              // Jump to Contactor Closed State
#ifdef DEBUG_LOG
        logging.println("Solax Battery State: Contactor Closed");
#endif
        break;

      case (CONTACTOR_CLOSED):
        datalayer.system.status.inverter_allows_contactor_closing = true;
        SOLAX_1875.data.u8[4] = (0x01);  // Inform Inverter: Contactor 0=off, 1=on.
        transmit_can_frame(&SOLAX_187E, can_config.inverter);
        if (BATTERY_TYPE == 0x5A)
          {
            transmit_can_frame(&SOLAX_187D, can_config.inverter);
            transmit_can_frame(&SOLAX_187C, can_config.inverter);
            transmit_can_frame(&SOLAX_187B, can_config.inverter);
          }
        transmit_can_frame(&SOLAX_187A, can_config.inverter);
        transmit_can_frame(&SOLAX_1872, can_config.inverter);
        transmit_can_frame(&SOLAX_1873, can_config.inverter);
        transmit_can_frame(&SOLAX_1874, can_config.inverter);
        transmit_can_frame(&SOLAX_1875, can_config.inverter);
        transmit_can_frame(&SOLAX_1876, can_config.inverter);
        transmit_can_frame(&SOLAX_1877, can_config.inverter);
        transmit_can_frame(&SOLAX_1878, can_config.inverter);
        // Message from the inverter to open contactor
        // Byte 4 changes from 1 to 0
        if (rx_frame.data.u64 == Contactor_Open_Payload) {
          set_event(EVENT_INVERTER_OPEN_CONTACTOR, 0);
          STATE = BATTERY_ANNOUNCE;
        }
        break;
    }
  }

  if (rx_frame.ID == 0x1871 && rx_frame.data.u64 == __builtin_bswap64(0x0500010000000000)) {
    if (BATTERY_TYPE == 0x5A)
    {
      // BMS serial number
      SOLAX_1881.data = {0x0, 0x48, 0x53, 0x32, 0x35, 0x41, 0x4A, 0x41}; // HS25AJA
      SOLAX_1882.data = {0x0, 0x31, 0x32, 0x41, 0x42, 0x33, 0x34, 0x35}; // 12AB345

      transmit_can_frame(&SOLAX_1881, can_config.inverter);
      transmit_can_frame(&SOLAX_1882, can_config.inverter);

      // Battery modules serial numbers
      SOLAX_1881.data = {0x0, 0x48, 0x53, 0x33, 0x36, 0x42, 0x4A, 0x42}; // HS36BJB
      SOLAX_1882.data = {0x0, 0x31, 0x32, 0x41, 0x42, 0x33, 0x34, 0x35}; // 12AB345

      for (int i = 1; i < NUMBER_OF_MODULES+1; i++)
      {
        SOLAX_1881.data.u8[0] = (uint8_t)i;
        SOLAX_1882.data.u8[0] = (uint8_t)i;

        transmit_can_frame(&SOLAX_1881, can_config.inverter);
        transmit_can_frame(&SOLAX_1882, can_config.inverter);
      }
    }
    else
    {
      transmit_can_frame(&SOLAX_1881, can_config.inverter);
      transmit_can_frame(&SOLAX_1882, can_config.inverter);
    }
#ifdef DEBUG_LOG
    logging.println("1871 05-frame received from inverter");
#endif
  }
  if (rx_frame.ID == 0x1871 && rx_frame.data.u8[0] == (0x03)) {
#ifdef DEBUG_LOG
    logging.println("1871 03-frame received from inverter");
#endif
  }
}
void setup_inverter(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.inverter_protocol, "SolaX Triple Power LFP over CAN bus", 63);
  datalayer.system.info.inverter_protocol[63] = '\0';
  datalayer.system.status.inverter_allows_contactor_closing = false;  // The inverter needs to allow first
}
#endif
