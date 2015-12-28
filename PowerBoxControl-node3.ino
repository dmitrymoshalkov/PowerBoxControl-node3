#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SimpleTimer.h>
#include <DigitalIO.h>
#include <avr/wdt.h>


 //#define NDEBUG                        // enable local debugging information

#define NODE_ID 196 //На даче поменять на 102

/******************************************************************************************************/
/*                               				Сенсоры												  */
/******************************************************************************************************/

#define CHILD_ID_RELAY1				  30  //Реле 1
#define CHILD_ID_RELAY2				  31  //Реле 2
#define CHILD_ID_RELAY3				  32  //Реле 3
#define CHILD_ID_RELAY4				  33  //Реле 4

#define CHILD_ID_RELAY1_STATUS 		  20
#define CHILD_ID_RELAY2_STATUS 		  21
#define CHILD_ID_RELAY3_STATUS 		  22
#define CHILD_ID_RELAY4_STATUS 		  23

#define CHILD_ID_RELAYPOWER_STATUS 	  24



#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101 


/******************************************************************************************************/
/*                               				IO												                                    */
/******************************************************************************************************/

#define ONE_WIRE_BUS              3      // Pin where dallase sensor is connected 
#define RELAY1_PIN		      	  4
#define RELAY2_PIN		      	  5
#define RELAY3_PIN		      	  6
#define RELAY4_PIN		      	  7

#define RELAY1_STATE_PIN		       A0
#define RELAY2_STATE_PIN		       A1
#define RELAY3_STATE_PIN		       A2
#define RELAY4_STATE_PIN		       A3

#define RELAYPOWER_STATE_PIN           A4


/*****************************************************************************************************/
/*                               				Common settings									      */
/******************************************************************************************************/

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define COMPARE_TEMP              1      // Send temperature only if changed? 1 = Yes 0 = No
#define MAX_ATTACHED_DS18B20      16
#define EEPROM_DEVICE_ADDR_START  64     // start byte in eeprom for remembering our sensors
#define EEPROM_DEVICE_ADDR_END    EEPROM_DEVICE_ADDR_START+MAX_ATTACHED_DS18B20*2

#define RADIO_RESET_DELAY_TIME 20 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения

#define NUM_OF_PRESENTED_TEMP_SENSORS 2

OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass the oneWire reference to Dallas Temperature. 

float lastTemperature[MAX_ATTACHED_DS18B20];
uint8_t numSensors = 0;
uint8_t currentTsensor = 0;
DeviceAddress dsaddr[MAX_ATTACHED_DS18B20];
bool ts_spot[MAX_ATTACHED_DS18B20]; // used spot array

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

boolean boolRecheckSensorValues = false;
boolean boolRecheckTempValues = false;

int lastRelayPowerStatusLightLevel = -1;             // Holds last light level
int lastRelay1StatusLightLevel = -1;             // Holds last light level
int lastRelay2StatusLightLevel = -1;             // Holds last light level
int lastRelay3StatusLightLevel = -1;             // Holds last light level
int lastRelay4StatusLightLevel = -1;             // Holds last light level

// Initialize temperature message
MyMessage TempMsg(0, V_TEMP);  //CHILD_ID_TEMPERATURE

MyMessage Relay1StateMsg(CHILD_ID_RELAY1_STATUS ,V_STATUS);
MyMessage Relay2StateMsg(CHILD_ID_RELAY2_STATUS ,V_STATUS);
MyMessage Relay3StateMsg(CHILD_ID_RELAY3_STATUS ,V_STATUS);
MyMessage Relay4StateMsg(CHILD_ID_RELAY4_STATUS ,V_STATUS);

MyMessage RelayPowerStateMsg(CHILD_ID_RELAYPOWER_STATUS ,V_STATUS);



boolean receivedConfig = false;
boolean metric = true; 

SimpleTimer checkTempTimer;
SimpleTimer checkRelayStatus;
SimpleTimer checkRelayPower;

MySensor gw;


void setup()  
{ 

  //init relay ports	
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  //init relay status ports
  pinMode(RELAY1_STATE_PIN, INPUT);
  pinMode(RELAY2_STATE_PIN, INPUT);
  pinMode(RELAY3_STATE_PIN, INPUT);
  pinMode(RELAY4_STATE_PIN, INPUT);
  pinMode(RELAYPOWER_STATE_PIN, INPUT);

 // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin(incomingMessage, NODE_ID, false);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Power box relays sensor", "1.0");


  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

        #ifdef NDEBUG
          Serial.print(F("Number of sensors # "));
          Serial.println(numSensors);
        #endif

  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     // reset used spot array
     ts_spot[i] = false;
  }
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     sensors.getAddress(dsaddr[i],i);
     // check if we know this sensor
     int8_t sidx = getSensorIndex(dsaddr[i]);
     if (sidx >= 0) {
        // we know this sensor
        ts_spot[sidx] = true;
     }
  }
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     int8_t sidx = getSensorIndex(dsaddr[i]);
     if (sidx < 0) {
        // we have unknown sensor (not present in EEPROM)
        // let's find a first free spot and put the addr hash there
        uint8_t spot = 0;
        while (ts_spot[spot] && spot < MAX_ATTACHED_DS18B20) {
          spot++;
        }
        ts_spot[spot] = true;
        storeSensorAddr(dsaddr[i],spot);
        sidx = spot;
        #ifdef NDEBUG
          Serial.print(F("Added new sensor to spot # "));
          Serial.println(spot);
        #endif
     }
     
     gw.present(sidx, S_TEMP);
     #ifdef NDEBUG
       Serial.println();
       Serial.print(i);
       Serial.print(F(" index: "));
       Serial.print(sidx);
       Serial.print(F(" address: "));
       printAddress(dsaddr[i]);
       Serial.println();
     #endif
  }


    //reboot sensor command
    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(REBOOT_CHILD_ID, S_BINARY); //, "Reboot node sensor", true); 

    //reget sensor values
    gw.wait(RADIO_RESET_DELAY_TIME);
  	gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 


    //relays
    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY1, S_LIGHT);   

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY2, S_LIGHT);   

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY3, S_LIGHT);   

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY4, S_LIGHT);           

  	  // Relays status
    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY1_STATUS, S_BINARY);   

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY2_STATUS, S_BINARY);  

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY3_STATUS, S_BINARY);  

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAY4_STATUS, S_BINARY);  

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_RELAYPOWER_STATUS, S_BINARY);  


  	checkTempTimer.setInterval(10000, checkTemperature);
  	checkRelayStatus.setInterval(1000, checkRelaysStatus);
  	checkRelayPower.setInterval(120000, checkRelayPowerStatus);


    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.request(CHILD_ID_RELAY1, V_LIGHT);

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.request(CHILD_ID_RELAY2, V_LIGHT);

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.request(CHILD_ID_RELAY3, V_LIGHT);

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.request(CHILD_ID_RELAY4, V_LIGHT);        

  	  //Enable watchdog timer
  wdt_enable(WDTO_8S);



        #ifdef NDEBUG
          Serial.print(F("End setup "));
        #endif
  
}


void loop() {

  checkTempTimer.run();
  checkRelayStatus.run();
  checkRelayPower.run();

  gw.process();


if (boolRecheckSensorValues)
{
  	checkRelaysStatus();
  	checkRelayPowerStatus();
  boolRecheckSensorValues = false;
}


    //reset watchdog timer
    wdt_reset();    
}


void incomingMessage(const MyMessage &message) {

  if (message.isAck())
  {
    gotAck = true;
    return;
  }

    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }
     
     if (message.type==V_LIGHT && strlen(message.getString())>0 ) 
     {

     	switch(message.sensor)
     	{
     		case CHILD_ID_RELAY1:

     		   digitalWrite(RELAY1_PIN, message.getBool()?RELAY_ON:RELAY_OFF);

     			break;
     		case CHILD_ID_RELAY2:

     		   digitalWrite(RELAY2_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
     		   	
     			break;
     		case CHILD_ID_RELAY3:

     		   digitalWrite(RELAY3_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
     		   	
     			break;
     		case CHILD_ID_RELAY4:

     		   digitalWrite(RELAY4_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
     		   	
     			break;     		     		
     	}
     
     }

    if ( message.sensor == RECHECK_SENSOR_VALUES && strlen(message.getString())>0) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
            boolRecheckTempValues = true;

         }

     }

        return;      
} 



// a simple hash function for 1wire address to reduce id to two bytes
uint16_t simpleAddrHash(DeviceAddress a){
  return ((a[1] ^ a[2] ^ a[3] ^ a[4] ^ a[5] ^ a[6]) << 8) + a[7];
}

// search for device address hash in eeprom
// return -1 if not found
int8_t getSensorIndex(DeviceAddress a) {
  uint16_t hash = simpleAddrHash(a);
  int8_t idx = -1;
  uint8_t aidx = 0;
  uint8_t ptr = EEPROM_DEVICE_ADDR_START;
  while (ptr < EEPROM_DEVICE_ADDR_END && idx == -1) {
    uint8_t hash1 = gw.loadState(ptr);
    uint8_t hash2 = gw.loadState(ptr+1);
    if ( hash1 == (uint8_t)(hash >> 8) && hash2 == (uint8_t)(hash & 0x00FF)) {
      // found device index
      idx = aidx;
    }
    aidx++;
    ptr+=2;
  }
  return idx;
}

// save address hash in EEPROM under index
void storeSensorAddr(DeviceAddress a, uint8_t index) {
    uint16_t hash = simpleAddrHash(a);
    uint8_t ptr = EEPROM_DEVICE_ADDR_START + index * 2;
    if (ptr < EEPROM_DEVICE_ADDR_END) {
      gw.saveState(ptr,   hash >> 8);
      gw.saveState(ptr+1, hash & 0x00FF);
    }
    #ifdef NDEBUG
      Serial.print(F("storeSensorAddr under index: "));
      Serial.println(index);
    #endif
}



void checkTemperature(){
  // Check temperature
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  
  if (numSensors > 0) {
    currentTsensor = 0;
    checkTempTimer.setTimeout(conversionTime, readTemperature);
  }
}



void readTemperature(){

  #ifdef NDEBUG
  unsigned long start = millis();
  #endif

  // Fetch and round temperature to one decimal
  //float temperature = static_cast<float>(static_cast<int>(sensors.getTempC(dsaddr[currentTsensor]) * 10.)) / 10.;
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempC(dsaddr[currentTsensor]):sensors.getTempF(dsaddr[currentTsensor])) * 10.)) / 10.;


  // Only send data if temperature has changed and no error
  #if COMPARE_TEMP == 1
  if ( ((lastTemperature[currentTsensor] != temperature) || boolRecheckTempValues ) && temperature != -127.00 && temperature != 85.00) {
  #else
  if (temperature != -127.00 && temperature != 85.00) {
  #endif

      if ( temperature > 70  )
      {
        //TODO: отослать номер сенсора
        //Выключить электричество, если какой-то из автоматов нагрелся более чем на 59 градусов

        	switch(currentTsensor)
        	{
        		case 0:
     		   		digitalWrite(RELAY1_PIN, RELAY_OFF);
     		   		digitalWrite(RELAY2_PIN, RELAY_OFF);     		   	
     		   		break;
        		case 1:
     		   		digitalWrite(RELAY3_PIN, RELAY_OFF);
     		   		digitalWrite(RELAY4_PIN, RELAY_OFF);     		   	
     		   		break;
        	}
      }

        if ( (lastTemperature[currentTsensor] != temperature )  || boolRecheckTempValues )
        {
            //Отсылаем состояние сенсора с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	             gw.send(TempMsg.setSensor(getSensorIndex(dsaddr[currentTsensor])).set(temperature,1), true);
                    gw.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;
          }
    // Save new temperatures for next compare
    lastTemperature[currentTsensor] = temperature;

  }
 
 
  #ifdef NDEBUG
  Serial.print(F("Temperature "));
  Serial.print(currentTsensor,DEC);
  Serial.print(F(" index: "));
  Serial.print(getSensorIndex(dsaddr[currentTsensor]));
  Serial.print(F(" -> "));
  Serial.print(temperature);
  unsigned long etime = millis() - start;
  Serial.print(F(" time elapsed: "));
  Serial.println(etime);
  #endif
  

  currentTsensor++;
  if (currentTsensor < numSensors && currentTsensor < MAX_ATTACHED_DS18B20) {
    // postpone next sensor reading
    checkTempTimer.setTimeout(25, readTemperature);    
  } else
  {
      if (boolRecheckTempValues)
      {
        boolRecheckTempValues=false;
      }
  }

}




void checkRelayPowerStatus()
{
  //lastRelay1StatusLightLevel

  int  lightLevel=0;

  

     lightLevel = (1023-analogRead(RELAYPOWER_STATE_PIN))/10.23; 

        #ifdef NDEBUG
          Serial.print(F("Relay power light level # "));
          Serial.println(lightLevel);
        #endif
  if (lightLevel >= 0)
  {    

     lightLevel = (lightLevel > 90) ? 1 : 0; 

        #ifdef NDEBUG
          Serial.print(F("Relay power status # "));
          Serial.println(lightLevel);
        #endif

          if ( (lightLevel != lastRelayPowerStatusLightLevel) || boolRecheckSensorValues) {


      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(RelayPowerStateMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastRelayPowerStatusLightLevel = lightLevel;
          }
    
    
          lightLevel=0;

  }  
  
}

void checkRelaysStatus()
{
 

  int  lightLevel=0;

  

     lightLevel = (1023-analogRead(RELAY1_STATE_PIN))/10.23;  //relay 1

        #ifdef NDEBUG
          Serial.print(F("Relay 1 light level # "));
          Serial.println(lightLevel);
        #endif
  if (lightLevel >= 0)
  {    

     lightLevel = (lightLevel > 90) ? 1 : 0; 

        #ifdef NDEBUG
          Serial.print(F("Relay 1 status # "));
          Serial.println(lightLevel);
        #endif

          if ( (lightLevel != lastRelay1StatusLightLevel) || boolRecheckSensorValues) {


      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(Relay1StateMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastRelay1StatusLightLevel = lightLevel;
          }
    
    
          lightLevel=-1;

  }  


     lightLevel = (1023-analogRead(RELAY2_STATE_PIN))/10.23;  //relay 2

        #ifdef NDEBUG
          Serial.print(F("Relay 2 light level # "));
          Serial.println(lightLevel);
        #endif
  if (lightLevel >= 0)
  {    

     lightLevel = (lightLevel > 90) ? 1 : 0; 

        #ifdef NDEBUG
          Serial.print(F("Relay 2 status # "));
          Serial.println(lightLevel);
        #endif

          if ( (lightLevel != lastRelay2StatusLightLevel) || boolRecheckSensorValues) {



      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(Relay2StateMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastRelay2StatusLightLevel = lightLevel;
          }
    
    
          lightLevel=-1;

  }  


     lightLevel = (1023-analogRead(RELAY3_STATE_PIN))/10.23;  //relay 3

        #ifdef NDEBUG
          Serial.print(F("Relay 3 light level # "));
          Serial.println(lightLevel);
        #endif
  if (lightLevel >= 0)
  {    

     lightLevel = (lightLevel > 90) ? 1 : 0; 

        #ifdef NDEBUG
          Serial.print(F("Relay 3 status # "));
          Serial.println(lightLevel);
        #endif

          if ( (lightLevel != lastRelay3StatusLightLevel) || boolRecheckSensorValues) {



      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(Relay3StateMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastRelay3StatusLightLevel = lightLevel;
          }
    
    
          lightLevel=-1;

  }  


     lightLevel = (1023-analogRead(RELAY4_STATE_PIN))/10.23;  //relay 4

        #ifdef NDEBUG
          Serial.print(F("Relay 4 light level # "));
          Serial.println(lightLevel);
        #endif
  if (lightLevel >= 0)
  {    

     lightLevel = (lightLevel > 90) ? 1 : 0; 

        #ifdef NDEBUG
          Serial.print(F("Relay 4 status # "));
          Serial.println(lightLevel);
        #endif

          if ( (lightLevel != lastRelay4StatusLightLevel) || boolRecheckSensorValues) {



      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(Relay4StateMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastRelay4StatusLightLevel = lightLevel;
          }
    
    
          lightLevel=-1;

  }  


}

#ifdef NDEBUG
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
#endif

