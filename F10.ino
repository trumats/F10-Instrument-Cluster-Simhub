#include <mcp_can.h>
#include <SPI.h>

#define CAN_CS 9
MCP_CAN CAN(CAN_CS);

#define POLYNOMIAL 0x1D
#define INIT_CRC 0xFF
const uint8_t ledPins[] = {0, 1, 2, 3, 4, 5, 6}; // Pins with LEDs

uint8_t crcTable[256];
uint8_t counter4Bit = 0;
uint8_t count = 0;
uint32_t lastDashboardUpdateTime = 0;
uint32_t lastDashboardUpdateTime1000ms = 0;
uint32_t dashboardUpdateTime100 = 100;
uint32_t dashboardUpdateTime1000 = 1000;
uint16_t distanceTravelledCounter = 0;
uint8_t accCounter = 0;
uint32_t timestamp100ms = 0;
uint32_t timestamp200ms = 0;
// ✅ Launch control status
int launchctrl = 0;
void initCRC8Table() {
  for (int i = 0; i < 256; ++i) {
    uint8_t rem = i;
    for (uint8_t b = 0; b < 8; ++b)
      rem = (rem & 0x80) ? (rem << 1) ^ POLYNOMIAL : (rem << 1);
    crcTable[i] = rem;
  }
}

uint8_t getCRC8(const uint8_t* data, uint8_t len, uint8_t finalXor = 0x00) {
  uint8_t crc = INIT_CRC;
  for (uint8_t i = 0; i < len; ++i)
    crc = crcTable[data[i] ^ crc];
  return crc ^ finalXor;
}

void sendIgnitionStatus(bool ignition) {
  // uint8_t ignitionStatus = ignition ? 0x8A : 0x08;
  // uint8_t frame[7] = { 0x80 | counter4Bit, ignitionStatus, 0xDD, 0xF1, 0x01, 0x30, 0x06 };
  // uint8_t message[8] = { getCRC8(frame, 7, 0x44) };
  // memcpy(&message[1], frame, 7);
  // CAN.sendMsgBuf(0x12F, 0, 8, message);

  uint8_t ign1[8] = { 0x6E, 0x77, 0xFA, 0xDD, 0xFF, 0xFF, 0xFF, 0x00};
  uint8_t ign2[8] = { 0x88, 0x7A, 0xFA, 0xDD, 0xFF, 0xFF, 0xFF, 0x00};
  uint8_t ign3[8] = { 0xD5, 0x7B, 0xFA, 0xDD, 0xFF, 0xFF, 0xFF, 0x00};
  uint8_t ign4[8] = { 0xD9, 0x50, 0x8A, 0xDD, 0xF4, 0x01, 0x30, 0x01};
  uint8_t ign5[8] = {0x8C, 0x50, 0x8A, 0xDD, 0xF4, 0x05, 0x30, 0x06};

  CAN.sendMsgBuf(0x12F, 0, 8, ign1);
  CAN.sendMsgBuf(0x12F, 0, 8, ign2);
  CAN.sendMsgBuf(0x12F, 0, 8, ign3);
  CAN.sendMsgBuf(0x12F, 0, 8, ign4);
  CAN.sendMsgBuf(0x12F, 0, 8, ign5);
}
void sendBackLightsStatus(bool status){
  uint8_t backLightStatus = status ? 0xFD : 0x00;
  uint8_t backLight[2] = {backLightStatus, 0xFF};
  CAN.sendMsgBuf(0x202, 0, 2, backLight);
}

void sendDriveMode(uint8_t driveMode){
  uint8_t drive[8] = {0x00, 0x00, 0x00, 0x00, driveMode, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(0x3A7, 0, 8, drive); 
}

void sendOilTemp(uint8_t oilTempValue){
  uint8_t oilTemp[8] = {0x02, 0x00, 0x00, 0x00, 0x00, oilTempValue, 0x00, 0x00};
  CAN.sendMsgBuf(0x3F9, 0, 8, oilTemp);
}


void sendSpeed(int kph) {
  uint16_t scaled = (uint16_t)(kph * 64.01);
  uint8_t frame[4] = { static_cast<uint8_t>(0xC0 | counter4Bit), lowByte(scaled), highByte(scaled), kph == 0 ? 0x81 : 0x91 };
  uint8_t message[5] = { getCRC8(frame, 4, 0xA9) };
  memcpy(&message[1], frame, 4);
  CAN.sendMsgBuf(0x1A1, 0, 5, message);
}

void sendRPM(int rpm, int gear) {
  int mappedGear = 0;
  if (gear >= 1 && gear <= 9) mappedGear = gear + 4;
  else if (gear == 11) mappedGear = 2;
  else if (gear == 12) mappedGear = 1;

  int rpmVal = map(rpm, 0, 6900, 0x00, 0x2B);
  uint8_t frame[7] = { static_cast<uint8_t>(0x60 | counter4Bit), static_cast<uint8_t>(rpmVal), 0xC0, 0xF0, static_cast<uint8_t>(mappedGear), 0xFF, 0xFF };
  uint8_t message[8] = { getCRC8(frame, 7, 0x7A) };
  memcpy(&message[1], frame, 7);
  CAN.sendMsgBuf(0x0F3, 0, 8, message);
}

void sendFuel(int fuelQuantity) {
  uint8_t inFuelRange[3] = {0, 50, 100};
  uint8_t outFuelRange[3] = {37, 18, 4};
  uint8_t fuelQuantityLiters = map(fuelQuantity, inFuelRange[0], inFuelRange[2], outFuelRange[0], outFuelRange[2]);
  unsigned char fuelWithoutCRC[] = { highByte(fuelQuantityLiters), lowByte(fuelQuantityLiters), highByte(fuelQuantityLiters), lowByte(fuelQuantityLiters), 0x00 };
  CAN.sendMsgBuf(0x349, 0, 5, fuelWithoutCRC);
}



void kombiHappy(){
    uint8_t stmp1[8] = {0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t stmp2[8] = {0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t stmp3[8] = {0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t stmp4[8] = {0x12, 0xDD, 0x8A, 0xDD, 0xF1, 0x15, 0x30, 0x02};
    uint8_t stmp5[8] = {0x02, 0xDD, 0xDD, 0x00, 0x00, 0xDD, 0xDD, 0xDD};
    uint8_t stmp6[8] = {0x80, 0x00, 0x8A, 0xDD, 0xF1, 0x01, 0x30, 0x06};
    CAN.sendMsgBuf(0x7C3, 8, stmp1);
    //CAN.sendMsgBuf(0x36A, 8, stmp2);
    CAN.sendMsgBuf(0x3D8, 8, stmp3);
    CAN.sendMsgBuf(0x3F9, 8, stmp4);
    CAN.sendMsgBuf(0x12F, 8, stmp5);
    CAN.sendMsgBuf(0x12F, 8, stmp6);
}


void sendAirbagOff() {
  uint8_t message[2] = {count, 0xFF};
  CAN.sendMsgBuf(0x0D7, 0, 2, message);
}

void sendSeatbelt(bool status) {
  if(status){  
    uint8_t belt[8] = {0x40, 0x4D, 0x00, 0x00, 0x28, 0xFF, 0xFF, 0xFF}; //ON
    CAN.sendMsgBuf(0x581, 0, 8, belt);
  }
  else{
    uint8_t belt2[8] = {0x40, 0x4D, 0x00, 0x00, 0x29, 0xFF, 0xFF, 0xFF}; //OFF
    CAN.sendMsgBuf(0x394, 0, 8, belt2);
  }

}

void setup() {
  Serial.begin(115200);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("CAN init failed");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  initCRC8Table();
  Serial.println("CAN ready");
    timestamp100ms = millis();
    timestamp200ms = millis();


}



void loop() {
  sendIgnitionStatus(true);

  if (millis() - timestamp100ms > 99) {
    sendBackLightsStatus(true);
    sendDriveMode(5);
    kombiHappy();

    sendFuel(50);
    sendOilTemp(130);

    int rpm = 3000;   // Simulēta vērtība
    int speed = 5;    // Simulēta vērtība

    sendRPM(rpm, 3);
    sendSpeed(speed);

    // ✅ Launch Control loģika
    if (speed <= 10 && rpm >= 2500) {
      uint8_t launchctrlMsg[] = { 0x40, 0xBE, 0x01, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, launchctrlMsg);
      launchctrl = 1;
    } else {
      uint8_t launchctrlMsg[] = { 0x40, 0xBE, 0x01, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, launchctrlMsg);
      launchctrl = 0;
    }

    timestamp100ms = millis();
  }

  if (millis() - timestamp200ms > 199) {
    sendAirbagOff();
    sendSeatbelt(true);
    count++;
    timestamp200ms = millis();
  }

  counter4Bit = (counter4Bit + 1) % 15;
  delay(100);
}