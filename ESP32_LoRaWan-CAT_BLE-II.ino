#ifdef __cplusplus
  extern "C" {
 #endif
 
  uint8_t temprature_sens_read();
 
#ifdef __cplusplus
}
#endif
 
uint8_t temprature_sens_read();

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <U8x8lib.h>

#include "BLEDevice.h"
#define FLORA_ADDR "c4:7c:8d:66:d6:a0"

BLEClient* pClient;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// The characteristic of the remote service we are interested in.
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");
static BLEAddress floraAddress(FLORA_ADDR);
static BLERemoteCharacteristic* pRemoteCharacteristic;
float temp;
int moisture;
int light;
int conductivity;
int freeHeapstart;
int freeHeapstop;

//For Heltec Wifi LoRa 32, TTGO LoRa and TTGO LoRa32 V1 use:
U8X8_SSD1306_128X64_NONAME_HW_I2C display(/*rst*/ 16, /*scl*/ 15, /*sda*/ 4);

#define SEALEVELPRESSURE_HPA (1013.25) //1013.25

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

char buf1[]="";

float temperature = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;

uint16_t tempC_int = 0;
uint16_t tempC1_int = 0;
uint8_t hum_int = 0;
uint16_t lux_int = 0;
uint16_t pressure_int = 0;

bool next = false;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define txInterval_1  5        /* Time ESP32 will go to sleep (in seconds) */

// Lorawan CAT 
// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { 0x28, 0xAE, 0xD2, 0x2B, 0x7E, 0x15, 0x16, 0xA6, 0x09, 0xCF, 0xAB, 0xF7, 0x15, 0x88, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { 0x16, 0x28, 0xAE, 0x2B, 0x7E, 0x15, 0xD2, 0xA6, 0xAB, 0xF7, 0xCF, 0x4F, 0x3C, 0x15, 0x88, 0x09 };

// LoRaWAN end-device address (DevAddr), ie 0x91B375AC  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x260411B4;

static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x25, 0xF2, 0xCA, 0x65, 0xDF, 0x1D, 0x8C };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

int channel = 0;
int txInterval = 60;

unsigned long sampletime_ms = 30000;  // 30 Seconds

#define RATE        DR_SF12

struct {
  uint8_t Header1[2] = {0x01,0x67};
  char temp[2];
  uint8_t Header2[2] = {0x02,0x67};
  char temp1[2];  
  uint8_t Header3[2] = {0x01,0x68};
  char humid[1];
  uint8_t Header4[2] = {0x01,0x02};
  char vbat[2];  
  uint8_t Header5[2] = {0x01,0x65};
  char lux[2];   
  uint8_t Header6[2] = {0x01,0x73};
  char baro[2];  
  uint8_t Header7[2] = {0x01,0x02};
  char mois[2];
  uint8_t Header8[2] = {0x01,0x02};
  char cond[2];    
}mydata;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ LMIC_UNUSED_PIN}
};

bool TX_done = false;
bool joined = false;

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            display.drawString(0, 2, "Deep sleep 60s");
            esp_deep_sleep_start();
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
  next = true; // Always send after any event, to recover from a dead link
}

void do_send(osjob_t* j) {

display.drawString(0, 2, "WakeUp!!!!      ");
  Serial.println(F("Enter do_send"));

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    getSensorData();
    readData();
    // Prepare upstream data transmission at the next possible time.
    //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  Serial.println(F("Leave do_send"));

  TX_done = false;

}

int vbat_int;

void getSensorData() {
  //freeHeapstart = ESP.getFreeHeap();
 //Serial.print("FREEHEAP START : ");
  //Serial.println(freeHeapstart);
  getSensorData1(floraAddress);
}

void getSensorData1(BLEAddress pAddress){
  btStart();
  //delay(500);
  //Serial.println("============================ Forming a connection to Flora device ===============================");
  //Serial.println(pAddress.toString().c_str());
  // Connect to the remove BLE Server.
  if (!pClient->connect(pAddress)){
  pClient->disconnect();
  return; 
  } else {
  Serial.println(" - Connected to Flora");
  }
  delay(500);
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
  } else {
    Serial.println(" - Found our service");
  }
 
  pRemoteCharacteristic = pRemoteService->getCharacteristic(uuid_write_mode);
  uint8_t buf[2] = {0xA0, 0x1F};
  pRemoteCharacteristic->writeValue(buf, 2, true);
  
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(uuid_sensor_data);
  //Serial.println(pRemoteService->toString().c_str());
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(uuid_sensor_data.toString().c_str());
  } else {
      Serial.println(" - Found our characteristic");
    }
  delay(500);  
  // Read the value of the characteristic.
  std::string value = pRemoteCharacteristic->readValue();
  Serial.print("The characteristic value was: ");
  const char *val = value.c_str();

  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++) {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  temp = (val[0] + val[1] * 256) / ((float)10.0);
  moisture = val[7];
  light = val[3] + val[4] * 256;
  conductivity = val[8] + val[9] * 256;
  
  char buffer[64];
  
  Serial.print("Temperature: ");
  Serial.println(temp);

  Serial.print("Moisture: ");
  Serial.println(moisture);
  
  Serial.print("Light: ");
  Serial.println(light);
  
  Serial.print("Conductivity: ");
  Serial.println(conductivity);
  pClient->disconnect();
  delay(500);
  btStop();
 // timer.setTimeout(1000L,sendSensorData);
}

void readData(){
   
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    moisture = moisture*100;
    conductivity = conductivity*10;
        
    tempC_int = temperature*10;
    tempC1_int = temp*10;
    hum_int = humidity*2;
    pressure_int = pressure*10;
    //light = light;
 
    mydata.temp[0] = tempC_int >> 8;
    mydata.temp[1] = tempC_int;
    mydata.temp1[0] = tempC1_int >> 8;
    mydata.temp1[1] = tempC1_int;    
    mydata.humid[0] =  hum_int ;
    mydata.vbat[0] = vbat_int >> 8;
    mydata.vbat[1] = vbat_int;    
    mydata.lux[0] = light >> 8;
    mydata.lux[1] = light;    
    mydata.baro[0] = pressure_int >> 8;
    mydata.baro[1] = pressure_int;     
    mydata.mois[0] = moisture >> 8;
    mydata.mois[1] = moisture;
    mydata.cond[0] = conductivity >> 8;
    mydata.cond[1] = conductivity;        

    sprintf(buf1,"T%2.1f RH%2.1f",temperature,humidity);
    display.drawString(0, 3, buf1);    
    sprintf(buf1,"%2.0fhPa %2.0fm",pressure,altitude);
    display.drawString(0, 4, buf1);     
    sprintf(buf1,"mois:%d uS:%d ",moisture/100,conductivity/10);
    display.drawString(0, 5, buf1);      
    sprintf(buf1,"T:%2.1f lux:%2.0f",temp,light);
    display.drawString(0, 6, buf1);    
}

void forceTxSingleChannelDr() {
    for(int i=0; i<9; i++) { // For EU; for US use i<71
        if(i != channel) {
            LMIC_disableChannel(i);
        }
    }
    // Set data rate (SF) and transmit power for uplink
    LMIC_setDrTxpow(RATE, 14);
}

void setup() {
  
  Serial.begin(115200);
  SPI.begin(5,19,27);
  display.begin();
  display.setFont(u8x8_font_victoriamedium8_r); 

  BLEDevice::init("");
  pClient  = BLEDevice::createClient();
  
  esp_sleep_enable_timer_wakeup(txInterval * uS_TO_S_FACTOR);
   
  bool status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  
  LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
  LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
  LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
  LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band          
  LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    

  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  //LMIC.dn2Dr = DR_SF9;

  forceTxSingleChannelDr(); 
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(RATE, 14);
  
  Serial.println(F("Leave setup"));

  display.drawString(0, 0, "ESP32 CAT03 ");
  display.drawString(0, 1, "SF12");

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}




