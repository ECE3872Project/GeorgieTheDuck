#include <Servo.h>
#include <stdio.h>
#include <math.h>
#include <TimedAction.h>
#include <Wire.h>
//#include "Adafruit_TPA2016.h"
#include "PinChangeInt.h" // PinChange Interrupt library
#include <Thread.h>

#define TempoCal 512
#define TempoPotMax 1023
#define OctaveHigh 800
#define OctaveLow 500
#define OctavePotMax 1023
#define PwmMax 255
//mic min and max
#define mic_threshold 700


#define rest 0

#define Octive 4

//Music Notes based on Octive--
#define C 16.3516*pow(2,Octive)
#define D 18.35405*pow(2,Octive)
#define E 20.60172*pow(2,Octive)
#define F 21.82676*pow(2,Octive)
#define G 24.49971*pow(2,Octive)
#define A 27.5*pow(2,Octive)
#define B 30.86771*pow(2,Octive)
#define high_C 32.70320*pow(2,Octive)
#define rest 0

//pins and motors
#define speakerPIN 5
#define sycnPIN 3
#define stop_btn 2
#define play_btn 6
#define leftwingPIN 10
#define rightwingPIN 11
//stop light
#define red_led 4
//syncing light
#define yellow_led 7 
//running light 
#define green_led 8 
int conductorSignal = A3;



//states
enum state 
{   ST_IDLE = 0, 
    ST_STOP = 1, 
    ST_PLAY = 2,
   ST_SYNC = 3,
    ST_OCTAVE_SYNC = 4
};


bool stop = false;
bool fly = false;
int timerCounter = 0;
bool sync_time = true; //Varible will keep the sync thread from constant updats
int sync_count = 0; // count down till sync fails
int time_ups = 50; // after 2 ful plays just go to sync mode

//Row Row Row Your Boat
int songLength = 54;  
int notes[] = {C, rest, C, rest, C, rest, D, rest, E, rest, E, rest, D, rest, E, rest, F, rest, G, rest, high_C, rest, high_C, rest, high_C, rest, G, rest, G, rest, G, rest, E, rest, E, rest, E, rest, C, rest, C, rest, C, rest, G, rest, F, rest, E, rest, D, rest, C, rest};
int beats[] = {2,1,2,1,2,1,1,1,2,1,2,1,1,1,2,1,1,1,6,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,2,1,1,1,5,1};
int song_tempo = 250;
int pos;

//setup the servo output
Servo left_wing;
Servo right_wing;

////// setting the begining state
int pastState = ST_PLAY;
//int currentState = ST_PLAY;
//int currentState = ST_SYNC;
int currentState = ST_IDLE;

//threads
//Functio looks at mic value and see if it hears the song
void sync_check(){
  if(sync_time){// if sync time not over
    sync_count++;
    int mic = analogRead(A0);
    if (mic >= mic_threshold){
      currentState = ST_SYNC;
    }
    sync_time = false;
  }
  //if over test time
  if (sync_count >= time_ups && sync_time){
    currentState = ST_SYNC;
    sync_time = false;
  }
}
//This thread checks the mic every 400 millsec for a sound
TimedAction sycThread = TimedAction(400,sync_check);



void setup()
{
  //sensors and potentiometer
  pinMode(A0, INPUT); //tempo senor
  pinMode(A1, INPUT);// box sensor
  pinMode(A3, INPUT); // sync mode, mic activated
  pinMode(A4, INPUT); // octave
  
  
  //set up outputs
  pinMode(speakerPIN, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  
  pinMode(sycnPIN, INPUT_PULLUP);
  pinMode(stop_btn, INPUT_PULLUP);
  pinMode(play_btn, INPUT);
  
  
  //attachInterrupt(A3, syncing, RISING);
  attachInterrupt(digitalPinToInterrupt(sycnPIN), syncing, RISING);
  attachInterrupt(digitalPinToInterrupt(stop_btn), stopping, RISING);
  //attachPinChangeInterrupt(play_btn, playing, RISING); // interrupt connected to pin 11}
  

  left_wing.attach(leftwingPIN);
  right_wing.attach(rightwingPIN);

  //debug only
  Serial.begin(9600);
  Serial.print("Setup Complete\n");
}



//stop button pressed so change state
void stopping() {
  stop = !stop;
  if(stop){
    pastState = currentState;
    currentState = ST_STOP;
  }else{
    currentState = pastState;
  }
  
    
} 
//move wings back and forth 40 degrees
void flapping() {
  //smove motor 40 degree
  if(fly){
    for (int l = 60, r = 120; l <= 120; l++, r--) {
      // tell servo to go to position in variable 'pos'
      left_wing.write(l);
      right_wing.write(r);
      // wait 15 ms for servo to reach the position
      delay(10);
    }
    for (int l = 120, r = 60; l >= 60; l--, r++) {
      // tell servo to go to position in variable 'pos'
      left_wing.write(l);
      right_wing.write(r);
      // wait 15 ms for servo to reach the position
      delay(10);
    }
  }   
}

//syncing with tempo
void syncing(){
  currentState = ST_SYNC;
    sync_time = false; 
}

//play button
void playing(){
  currentState = ST_PLAY;
}


int i_note_index;
void loop()
{
  int duration;                  
  int tempo;
  int tempo_pot;
  int octave;
  int octave_pot;
  int motor_speed;
  static const uint8_t analog_pins[] = {A2,A3,A4,A5};
  sycThread.check();
  
  Serial.print("looping\n");
  switch (currentState) {
   case ST_IDLE:
       // do something in the idle state
       Serial.print("idle\n");
       
       fly = false; //stop motor
      
       break;
    case ST_STOP:
       // do something in the stop state
        Serial.print("stop\n");
        digitalWrite(red_led, HIGH); //turn off the red LED
        digitalWrite(yellow_led, LOW);  //turn off the yellow LED
        digitalWrite(green_led, LOW); //turn off the green LED
        fly = false; //stop motor
        i_note_index = 0;
       break;
    case ST_PLAY:
       // do something in the play state
        Serial.print("play\n");
        digitalWrite(red_led, LOW); //turn off the red LED
        digitalWrite(yellow_led, LOW);  //turn off the yellow LED
        digitalWrite(green_led, HIGH); //turn on the green LED
       //move wings
        fly = true;
        // flapping();
        //song setup
        tempo_pot = analogRead(A0);//read the tempo pot
        tempo = song_tempo*float(tempo_pot)/TempoCal; //read the tempo POT
        octave_pot = analogRead(A4);//read the octave pot
      if(octave_pot >= OctaveHigh)
           octave = pow(2,Octive); // raise 1 hz
        else if(octave_pot <= OctaveLow)
           octave = (1/pow(2,Octive)); // lower 1 hz
        else
          octave = 1; // no pith change 
    
        //play the song
        if (i_note_index % 4 == 0) {
          left_wing.write(60);
          right_wing.write(120);
        } else {
          left_wing.write(120);
          right_wing.write(60);
        }
        duration = beats[i_note_index] * tempo;
        tone(speakerPIN, notes[i_note_index]*octave, duration);
        delay(duration);


//        for (int l = 60, r = 120; l <= 120; l++, r--) {
//          // tell servo to go to position in variable 'pos'
//          left_wing.write(l);
//          right_wing.write(r);
//
//          if (l % 10 == 0) {
//            duration = beats[i_note_index] * tempo;
//            tone(speakerPIN, notes[i_note_index++]*octave, duration);
//            delay(duration);
//          }
//        }
//        for (int l = 120, r = 60; l >= 60; l--, r++) {
//          // tell servo to go to position in variable 'pos'
//          left_wing.write(l);
//          right_wing.write(r);
//          
//          if (l % 10 == 0) {
//            duration = beats[i_note_index] * tempo;
//            tone(speakerPIN, notes[i_note_index++]*octave, duration);
//            delay(duration);
//          }
//        }

        
    
        //increment the note counter
        ++i_note_index;
        Serial.print(i_note_index);
        if(i_note_index >= songLength) {
          i_note_index = 0;
        }
      
       break;
    case ST_SYNC:
      Serial.print("sync\n");
      digitalWrite(red_led, LOW); //turn off the red LED
        digitalWrite(yellow_led, HIGH);  //turn on the yellow LED
        digitalWrite(green_led, LOW); //turn off the green LED
        fly = false; //stop motor
        i_note_index = 0;
    
        // do whatever whe do for syncing
      delay(2000);
      currentState = ST_PLAY;
        
       break;
    // etc...
}
}
