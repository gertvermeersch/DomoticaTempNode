#include <EEPROM.h>
#include <NRF905.h>
#include <domotica.h>
#include <DHT.h>
#include <SPI.h>
#include <avr/wdt.h>
#define DHTPIN 6 
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

  bool debug = true;
  bool red = true;
  bool green = false;
  volatile boolean measure_flag = false;
  volatile boolean message_flag = false;
  Domotica controller;
  char testmsg[MSG_LEN];
  float hysteresis[10];
  uint8_t counter = 0;
  long sent = 0;
  float currentTemperature = 20;
  float avgTemperature = 20;
  float currentHumidity = 40;
  bool heatingState = false;
  bool requestRunningState = false;
  bool requestTemp = false;
  bool requestHumidity = false;
  bool requestTargetTemp = false;
  uint8_t targetTemperature = 0;
  char from_address[4];
  char serialBuffer[32];
  char response[32];
  char tempstring[50];

void setup() {
  Serial.begin(115200); //max baud rate
  //nrf = NRF905();
  controller = Domotica();
  controller.setDebug(debug);
  controller.init(1); 
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(3, INPUT);
  attachInterrupt(1,receivedMessage,RISING);
  targetTemperature = EEPROM.read(0);
  dht.begin();
  if(debug) {
    Serial.print("The temperature from EEPROM is ");
    Serial.println(targetTemperature);  
  }
  
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 31249;// 2 second interval = MINIMUM for the temperature sensor // 1 sec interval for reading, can't read in interrupt because of use of delay == bad
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
  wdt_enable(WDTO_8S);
}

ISR(TIMER1_COMPA_vect) {
 measure_flag = true;
 //switch leds: heart beat
 red = heatingState;
 green = !green;
 digitalWrite(A2, red);
 digitalWrite(A1, green);
}

void measure() {
   hysteresis[counter++] = currentTemperature;
   
   if(counter == 10)
     counter = 0;
  
   if(hysteresis[0] == hysteresis[1] &&
   hysteresis[0] == hysteresis[2] &&
   hysteresis[0] == hysteresis[3] &&
   hysteresis[0] == hysteresis[4] &&
   hysteresis[0] == hysteresis[5] &&
   hysteresis[0] == hysteresis[6] &&
   hysteresis[0] == hysteresis[7] &&
   hysteresis[0] == hysteresis[8] &&
   hysteresis[0] == hysteresis[9] )
     avgTemperature = currentTemperature;
     if(debug) {
     Serial.print("Target temperature: ");
     Serial.println(targetTemperature);
     Serial.print("Current temperature: ");
     Serial.println(avgTemperature);
     }
   
   if(avgTemperature >= (float)targetTemperature - 0.4 && heatingState == true) {
     digitalWrite(A0, LOW);
     heatingState = false;
   }else if (avgTemperature < (float)targetTemperature - 0.6 && heatingState == false) { //allow the temperature to drop with 0.8 degree below the target temperature
     digitalWrite(A0, HIGH); 
     heatingState = true;
   }
   //check current temperature
 
   currentHumidity = dht.readHumidity();
    currentTemperature = dht.readTemperature();
    
    if(debug) {
      Serial.print("Measured temperature: ");
      Serial.println(currentTemperature);
      Serial.print("Measured humidity: ");
      Serial.println(currentHumidity);
    }
    
   
   measure_flag = false;
}

void loop() {
  if(measure_flag) {
    measure();
  }
  if(message_flag) {
    readMessages();
    sendResponses();
  }
  wdt_reset();  
  
 
}



void sendResponses() {
   if(requestTemp) {
     char response[28] = "STATTEMP0000000000000000000";
     char temp[5];
     //itoa(avgTemperature, temp ,10);
     //sprintf(temp, "%f", avgTemperature);
     dtostrf(avgTemperature, 4, 2, temp);
     response[8] = temp[0];
     response[9] = temp[1];
     response[10] = temp[2];
     response[11] = temp[3];
     response[12] = temp[4];
     
     Serial.print("sending temperature: ");
     Serial.println(temp);
     controller.sendToAddress((char*)from_address, response);
     requestTemp = false;
   }
   if(requestHumidity) {
     char response[28] = "STATHUMY0000000000000000000";
     char temp[5];
     dtostrf(currentHumidity, 4, 2, temp);
     response[8] = temp[0];
     response[9] = temp[1];
     response[10] = temp[2];
     response[11] = temp[3];
     response[12] = temp[4];
     //itoa(currentHumidity, temp ,10);
     //response[8] = temp[0];
     //response[9] = temp[1];
     Serial.print("sending humidity: ");
     Serial.println(temp);
     controller.sendToAddress((char*)from_address, response);
     requestHumidity = false;
   }
   if(requestTargetTemp) {
     char response[28] = "STATTTMP0000000000000000000";
     char temp[2];
     itoa(targetTemperature, temp ,10);
     if(targetTemperature < 10) {
       response[8] = '0';
       response[9] = temp[0];
     }
     else {
       response[8] = temp[0];
       response[9] = temp[1];
     }
     Serial.print("sending target temperature: ");
     Serial.println(temp);
     controller.sendToAddress((char*)from_address, response);
     requestTargetTemp = false;
   }
   if(requestRunningState) {
     char response[28] = "STATHEAT0000000000000000000";
     response[8] = heatingState?'1':'0';
     Serial.print("Sending state: ");
     Serial.println(response[8]);
     controller.sendToAddress((char*)from_address, response);
     requestRunningState = false;
   }
}

void readMessages() {
  if(controller.checkNewMsg()) {
     cli();
     char* ptrbuffer;
     ptrbuffer = controller.getMsg();
     char message[28];
     if(debug)Serial.print("\nSource address = ");
     for(int i = 0; i < 4; i++) {
       from_address[i] = ptrbuffer[i];  
       if(debug)Serial.print(from_address[i]);
     }
     if(debug)Serial.println();
     if(debug)Serial.print("Message = ");
     for(int i = 0; i < 28; i++) {
       message[i] = ptrbuffer[i+4];  
       if(debug)Serial.print(message[i]);
     }
     if(debug)Serial.println();
     if(message[0] == 'R' &&
       message[1] == 'E' &&
       message[2] == 'Q' &&
       message[3] == 'T' &&
       message[4] == 'T' &&
       message[5] == 'E' &&
       message[6] == 'M' &&
       message[7] == 'P' ) {
       if(debug)Serial.println("temperature requested");    
        requestTemp = true;
     } else if(message[0] == 'R' &&
       message[1] == 'E' &&
       message[2] == 'Q' &&
       message[3] == 'T' &&
       message[4] == 'T' &&
       message[5] == 'T' &&
       message[6] == 'M' &&
       message[7] == 'P' ) {
       if(debug)Serial.println("target temperature requested");    
        requestTargetTemp = true;
     } else if(message[0] == 'R' &&
       message[1] == 'E' &&
       message[2] == 'Q' &&
       message[3] == 'T' &&
       message[4] == 'H' &&
       message[5] == 'E' &&
       message[6] == 'A' &&
       message[7] == 'T' ) {
       if(debug)Serial.println("state requested");    
        requestRunningState = true;
     } else if(message[0] == 'R' &&
       message[1] == 'E' &&
       message[2] == 'Q' &&
       message[3] == 'T' &&
       message[4] == 'H' &&
       message[5] == 'U' &&
       message[6] == 'M' &&
       message[7] == 'Y' ) {
       if(debug)Serial.println("Humidity requested");    
        requestHumidity = true;
     }
      else if(message[0] == 'S' &&
       message[1] == 'E' &&
       message[2] == 'T' &&
       message[3] == 'V' &&
       message[4] == 'T' &&
       message[5] == 'T' &&
       message[6] == 'M' &&
       message[7] == 'P' ) {
       if(debug)Serial.println("target temp set");    
       char temperature[2];
       temperature[0] = message[8];
       temperature[1] = message[9];
       targetTemperature = atoi(temperature);
       EEPROM.write(0, targetTemperature);
    }
      
      
    sei();
  }
  message_flag = false;
  
}

void receivedMessage() {
  message_flag = true;
  
  
   
 }
   
 
 
 
 

