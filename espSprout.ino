
/*------------
Modified version of https://github.com/electricityforprogress/BiodataSonificationBreadboardKit/blob/master/BiodataSonification_026_kit.ino:
"Accepts pulse inputs from a Galvanic Conductance sensor 
consisting of a 555 timer set as an astablemultivibrator and two electrodes. 
Through sampling pulse widths and identifying fluctuations, MIDI note and control messages 
are generated.  Features include Threshold, Scaling, Control Number, and Control Voltage 
using PWM through an RC Low Pass filter."

This version:
Runs on ESP8266 Feather Huzzah
Sends MIDI notes via Websockets to remote server
-------------*/

// espSprout
#include <EEPROM.h> //store and read variables to nonvolitle memory

// socket client 
#include <Arduino.h>
#include "settings.h"

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <WebSocketsClient.h>

#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

#define USE_SERIAL Serial


//******************************
//set scaled values, sorted array, first element scale length
const int scaleCount = 5;
const int scaleLen = 13; //maximum scale length plus 1 for 'used length'
int currScale = 1; //current scale, default Chrom
int scale[scaleCount][scaleLen] = {
  {12,1,2,3,4,5,6,7,8,9,10,11,12}, //Chromatic
  {7,1, 3, 5, 6, 8, 10, 12}, //Major
  {7,1, 3, 4, 6, 8, 9, 11}, //DiaMinor
  {7,1, 2, 2, 5, 6, 9, 11}, //Indian
  {7,1, 3, 4, 6, 8, 9, 11} //Minor
};

int root = 0; //initialize for root, pitch shifting
//*******************************

const byte interruptPin = 5; //galvanometer input
//const byte knobPin = A0; //knob analog input
//Bounce button = Bounce(); //debounce button using Bounce2
//const byte buttonPin = A1; //tact button input
int menus = 5; //number of main menus
int mode = 0; //0 = Threshold, 1 = Scale, 2 = Brightness
int currMenu = 0;
int pulseRate = 350; //base pulse rate
int currentValue = 0; // base Midi value
int currentSequence[6]; // base MidiSequence of size 6
int currentSequencePos = 0; // sets the starting position for adding midi notes to array
char midiToSend[23]; // base for array to string conversion of midi sequence

const byte samplesize = 10; //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array

const byte polyphony = 1; //above 8 notes may run out of ram
int channel = 1;  //setting channel to 11 or 12 often helps simply computer midi routing setups
int noteMin = 36; //C2  - keyboard note minimum
int noteMax = 96; //C7  - keyboard note maximum
byte QY8= 0;  //sends each note out chan 1-4, for use with General MIDI like Yamaha QY8 sequencer
byte controlNumber = 80; //set to mappable control, low values may interfere with other soft synth controls!!
byte controlVoltage = 1; //output PWM CV on controlLED, pin 17, PB3, digital 11 *lowpass filter
long batteryLimit = 3000; //voltage check minimum, 3.0~2.7V under load; causes lightshow to turn off (save power)
byte checkBat = 1;

byte timeout = 0;
int value = 0;
int prevValue = 0;

volatile unsigned long microseconds; //sampling timer
volatile byte indexN = 0;
volatile unsigned long samples[samplesize];

float threshold = 1.7;   //2.3;  //change threshold multiplier
float threshMin = 1.61; //scaling threshold min
float threshMax = 3.71; //scaling threshold max
float knobMin = 1;
float knobMax = 1024;

unsigned long previousMillis = 0;
unsigned long currentMillis = 1;
unsigned long batteryCheck = 0; //battery check delay timer
unsigned long menuTimeout = 5000; //5 seconds timeout in menu mode

typedef struct _MIDImessage { //build structure for Note and Control MIDImessages
  unsigned int type;
  int value;
  int velocity;
  long duration;
  long period;
  int channel;
} 
MIDImessage;
MIDImessage noteArray[polyphony]; //manage MIDImessage data as an array with size polyphony
int noteindexN = 0;
MIDImessage controlMessage; //manage MIDImessage data for Control Message (CV out)

// put interrupt code into IRAM (added)
void ICACHE_RAM_ATTR sample ();

void setup() {
  // setup sockets 
  // USE_SERIAL.begin(921600);
  USE_SERIAL.begin(115200);

  //Serial.setDebugOutput(true);
  USE_SERIAL.setDebugOutput(true);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for(uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  WiFiMulti.addAP(ssid, password);

  //WiFi.disconnect();
  while(WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
  }

  // server address, port and URL
  webSocket.begin(host, 3344, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  // webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);
  
  // start heartbeat (optional)
  // ping server every 15000 ms
  // expect pong from server within 3000 ms
  // consider connection disconnected if pong is not received 2 times
  webSocket.enableHeartbeat(15000, 3000, 2);

  // setup espSprout
  randomSeed(analogRead(0)); //seed for QY8 4 channel mode
  //Serial.begin(31250);  //initialize at MIDI rate
  //Serial.begin(9600); //for debugging 
  
  controlMessage.value = 0;  //begin CV at 0
  attachInterrupt(interruptPin, sample, RISING);  //begin sampling from interrupt
  
}

void loop()
{
  currentMillis = millis();   //manage time
  if(indexN >= samplesize)  { analyzeSample(); }  //if samples array full, also checked in analyzeSample(), call sample analysis   
  checkNote();  //turn off expired notes 
  checkControl();  //update control value
}




void setNote(int value, int velocity, long duration, int notechannel)
{
  //find available note in array (velocity = 0);
  for(int i=0;i<polyphony;i++){
    if(!noteArray[i].velocity){
      //if velocity is 0, replace note in array
      noteArray[i].type = 0;
      noteArray[i].value = value;
      noteArray[i].velocity = velocity;
      noteArray[i].duration = currentMillis + duration;
      noteArray[i].channel = notechannel;
     
      
        if(QY8) { midiSerial(144, notechannel, value, velocity); } 
        else { midiSerial(144, channel, value, velocity); }

      break;
    }
  }

  // adds new MIDI note to array, sends it off ro server when array is full
  if (currentValue != noteArray[0].value) {
    Serial.println(noteArray[0].value);
    currentValue = noteArray[0].value;
    currentSequence[currentSequencePos] = currentValue;
    if (currentSequencePos == 5) {
      //convert to string and send it off to server
      sprintf(midiToSend, "%2d, %2d, %2d, %2d, %2d, %2d", currentSequence[0], currentSequence[1], currentSequence[2], currentSequence[3], currentSequence[4], currentSequence[5]);
      Serial.println(midiToSend);
      webSocket.loop();
      webSocket.sendTXT(midiToSend);
      // reset counter, flash array
      currentSequencePos = 0;
      memset(currentSequence, 0, 6);
      memset(midiToSend, 0, 23);
    } else {
      currentSequencePos += 1;
    }
  }
  
}

void setControl(int type, int value, int velocity, long duration)
{
  controlMessage.type = type;
  controlMessage.value = value;
  controlMessage.velocity = velocity;
  controlMessage.period = duration;
  controlMessage.duration = currentMillis + duration; //schedule for update cycle
}


void checkControl()
{
  signed int distance =  controlMessage.velocity - controlMessage.value; 
  //if still sliding
  if(distance != 0) {
    //check timing
    if(currentMillis>controlMessage.duration) { //and duration expired
        controlMessage.duration = currentMillis + controlMessage.period; //extend duration
        //update value
       if(distance > 0) { controlMessage.value += 1; } else { controlMessage.value -=1; }
       
       //send MIDI control message after ramp duration expires, on each increment
       midiSerial(176, channel, controlMessage.type, controlMessage.value); 
    }
  }
}

void checkNote()
{
  for (int i = 0;i<polyphony;i++) {
    if(noteArray[i].velocity) {
      if (noteArray[i].duration <= currentMillis) {
        //send noteOff for all notes with expired duration    
          if(QY8) { midiSerial(144, noteArray[i].channel, noteArray[i].value, 0); }
          else { midiSerial(144, channel, noteArray[i].value, 0); }
        noteArray[i].velocity = 0;
      }
    }
  }

}

// only for real MIDI out
void midiSerial(int type, int channel, int data1, int data2) {

  cli(); //kill interrupts, probably unnessisary
    //  Note type = 144
    //  Control type = 176  
    // remove MSBs on data
    data1 &= 0x7F;  //number
    data2 &= 0x7F;  //velocity
    
    byte statusbyte = (type | ((channel-1) & 0x0F));
    
//    Serial.write(statusbyte);
//    Serial.write(data1);
//    Serial.write(data2);
    
  sei(); //enable interrupts
}


//provide float map function
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//debug SRAM memory size
int freeRAM() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
} // print free RAM at any point

//interrupt timing sample array
void sample()
{
  if(indexN < samplesize) {
    samples[indexN] = micros() - microseconds;
    microseconds = samples[indexN] + microseconds; //rebuild micros() value w/o recalling
    //micros() is very slow
    //try a higher precision counter
    //samples[indexN] = ((timer0_overflow_count << 8) + TCNT0) - microseconds;
    indexN += 1;
  }
}



void analyzeSample()
{
  //eating up memory, one long at a time!
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;

  if (indexN == samplesize) { //array is full
    unsigned long sampanalysis[analysize];
    for (byte i=0; i<analysize; i++){ 
      //skip first element in the array
      sampanalysis[i] = samples[i+1];  //load analysis table (due to volitle)
      //manual calculation
      if(sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if(sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];  //prep stdevi
    }

    //manual calculation
    averg = averg/analysize;
    stdevi = sqrt(stdevi / analysize - averg * averg); //calculate stdevu
    if (stdevi < 1) { stdevi = 1.0; } //min stdevi of 1
    delta = maxim - minim; 
    
    //**********perform change detection 
    if (delta > (stdevi * threshold)){
      change = 1;
    }
    //*********
    
    if(change){// set note and control vector
       int dur = 150+(map(delta%127,1,127,100,2500)); //length of note
       int ramp = 3 + (dur%100) ; //control slide rate, min 25 (or 3 ;)
       int notechannel = random(1,5); //gather a random channel for QY8 mode
       
       //set scaling, root key, note
       int setnote = map(averg%127,1,127,noteMin,noteMax);  //derive note, min and max note
       setnote = scaleNote(setnote, scale[1], root);  //scale the note
       // setnote = setnote + root; // (apply root?)
       if(QY8) { setNote(setnote, 100, dur, notechannel); } //set for QY8 mode
       else { setNote(setnote, 100, dur, channel); }
  
       //derive control parameters and set    
       setControl(controlNumber, controlMessage.value, delta%127, ramp); //set the ramp rate for the control
     }
     //reset array for next sample
    indexN = 0;
  }
}


int scaleSearch(int note, int scale[], int scalesize) {
 for(byte i=1;i<scalesize;i++) {
  if(note == scale[i]) { return note; }
  else { if(note < scale[i]) { return scale[i]; } } //highest scale value less than or equal to note
  //otherwise continue search
 }
 //didn't find note and didn't pass note value, uh oh!
 return 6;//give arbitrary value rather than fail
}


int scaleNote(int note, int scale[], int root) {
  //input note mod 12 for scaling, note/12 octave
  //search array for nearest note, return scaled*octave
  int scaled = note%12;
  int octave = note/12;
  int scalesize = (scale[0]);
  //search entire array and return closest scaled note
  scaled = scaleSearch(scaled, scale, scalesize);
  scaled = (scaled + (12 * octave)) + root; //apply octave and root
  return scaled;
}
