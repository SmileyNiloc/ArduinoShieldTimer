//Colin Aten 
//CPEG298 Arduino Uno Shield IoT device
//May 22 2024

#include "Arduino.h"
#include <SoftwareSerial.h>        //Allows us to use two GPIO pins for a second UART
#include "Arduino_SensorKit.h"    //Library for controlling the OLED

//Library for using timers on Arduino Uno
//Timer setup
#define TIMER_INTERRUPT_DEBUG         1
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_1     true

#include "TimerInterrupt.h"  //Library for defining the timer interrupt

#include "ISR_Timer.h" //Library for setting up the ISR Timer
#define USE_TIMER_1     true

//Headers for OLED
#include <Arduino.h>

// Setup for ESP
SoftwareSerial espSerial(10, 11);  //Create software UART to talk to the ESP8266
String IO_USERNAME = "caten";
String IO_KEY = ""; //redacted for security
String WIFI_SSID = "UD Devices";  //Only need to change if using other network, eduroam won't work with ESP8266
String WIFI_PASS = "";            //Blank for open network


//Pin Assignments
#define BUTTON_PIN 3
#define BUZZER_PIN 6
#define POT_PIN A2
#define RST_PIN 8

//Function declarations
void display_time(unsigned int);
void button_interrupt();
void button_press();
void buzzer(byte);
void display_done();
void button_up();
void clear_disp();

//Global Variables
unsigned int current_time = 6000; // Variable to keep track of the time in seconds
unsigned int next_time=0;         // variable to keep hold the set time from potentiometer
unsigned int old_time=0;          // variable to check if there is a change in the time
char state = 0;                   // keeps track of our state for what to do in loop()
bool timer_running_flag = 0;
unsigned int pot_value = 0;       // Value for holding the value of the potentiometer
byte timer_flip=1;                // Flag to flip every time timer is run

bool debug = 0;                   // Variable to enable debug options or not
bool new_state_flag = 1;          // Variable to make new state debugging cleaner
bool display_debug = 0;           // To enable or disable debug in display

//Timer 1 definition
#define TIMER1_INTERVAL_MS 1000 //Timer interrupt runs every 1 second
void TimerHandler1() {
  current_time += -1;
  if(debug){
  Serial.print("Timer1 running");
  }
}

void setup() {
  // Set up Timer 1:
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1);
  ITimer1.pauseTimer();//start with timer paused

  // Set up OLED not flipped and with a font
  Oled.begin();
  Oled.setFlipMode(0);
  Oled.setFont(u8x8_font_chroma48medium8_r);

  // Begin Serial for debugging
  Serial.begin(115200);

  // Configure pins for the inputs and outputs
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POT_PIN,INPUT);
  pinMode(RST_PIN, OUTPUT);

  // Setup the button interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),button_interrupt,FALLING);

  // ESP Borad setup
  digitalWrite(RST_PIN,LOW);
  delay(500);
  digitalWrite(RST_PIN,HIGH);
  espSerial.begin(9600);  // set up software UART to ESP8266 @ 9600 baud rate
  Serial.println("setting up");
  String resp = espData("get_macaddr", 2000, true);      //get MAC address of 8266
  resp = espData("wifi_ssid=" + WIFI_SSID, 2000, true);  //send Wi-Fi SSID to connect to
  resp = espData("wifi_pass=" + WIFI_PASS, 2000, true);  //send password for Wi-Fi network
  resp = espData("io_user=" + IO_USERNAME, 2000, true);  //send Adafruit IO info
  resp = espData("io_key=" + IO_KEY, 2000, true);
  resp = espData("setup_io", 30000, true);  //setup the IoT connection
  if (resp.indexOf("connected") < 0) {
    Serial.println("\nAdafruit IO Connection Failed");
    while (1)
      ;
  }
  resp = espData("setup_feed=1,current_time2", 2000, false);  //start the data feed
  resp = espData("setup_feed=2, current_time", 2000, false);  //start the second data feed
  Serial.println("------ Setup Complete ----------");
}

void loop() {
  if(current_time<=1){              // Special Statement to check if time ran out
    if(timer_running_flag){       // Pause timer if neccessary
        ITimer1.pauseTimer();
        timer_running_flag = 0;
    }
    display_done();               // Calls function to show timer is done
  }
  else{
    switch (state){
      case 0: //Default state, for waiting for input
        new_state_flag =1;
        break;
      case 1: // State for checking potentiometer to set value of timer
        next_time = (analogRead(POT_PIN)*5.8);   // Get potentiometer value
        if(old_time != next_time){
          display_time(next_time);
          old_time = next_time;
        }
        if(new_state_flag){                    // Send to Serial for debugging
          Serial.println("state 1");
          Serial.print("Next_time =");
          Serial.println(next_time);
          new_state_flag = 0;
        }
        
        break;
      case 2: // Send value of next_time to current_time and activate timer, 
              //run once then go to watch state
        Serial.println("state 2");
        current_time = next_time;
        if(old_time != current_time){ // Only show on OLED if new
          old_time = current_time;
          display_time(current_time);
        }
        ITimer1.resumeTimer();
        timer_running_flag = 1;
        state++;
        new_state_flag = 1;
        break;
      case 3: // Watch state, timer and display are running and waiting for button
        if(new_state_flag){
          Serial.println("state 3");
          new_state_flag = 0;
        }
        if(old_time != current_time){ // Only update display if time changes
          old_time = current_time;
          display_time(current_time);
          espData("send_data=2," + String(current_time), 2000, false);  //send feed to cloud
        }
        break;
      case 4: // Paused state, pause or resume timer (flip it), and send back to state 3.
        Serial.println("state 4");      
        if(timer_running_flag){
          ITimer1.pauseTimer();
          timer_running_flag = 0; 
        }else{
          ITimer1.resumeTimer();
          timer_running_flag = 1;
        }
        new_state_flag = 1;
        state = 3;
        
        break;
      case 5: // State for resetting timer on DOUBLE press, sends to state 1
        Serial.println("state 5");
        if(timer_running_flag){
          ITimer1.pauseTimer();
          timer_running_flag = 0;
        }
        current_time = 2;
        state = 1;     //Sets state to starting state
        buzzer(1);     //Beeps buzzer once
        clear_disp();  //Sets the Oled to all spaces
        break;
    }
  }
}

void display_time(unsigned int t) {
  char seperated_time[9] = "";
// Calculate minutes and seconds
  if(debug){
    Serial.print("value passed into disply_time:  ");
    Serial.println(t);
  }
  int minutes = t / 60;
  int seconds = t  % 60;
  
//Convert to chars for the display
  seperated_time[0] = (minutes/10) + 48;
  seperated_time[1] = (minutes % 10) + 48;
  seperated_time[2] = ':';
  seperated_time[3] = (seconds/10) + 48;
  seperated_time[4] = (seconds%10) + 48;
  seperated_time[5] = '\0';

  if(debug){
  Serial.println(seperated_time);
  display_debug = 0;
  }
  //Oled.print("               0");
  Oled.setCursor(0, 0);
  Oled.print(seperated_time);
  Oled.refreshDisplay();
  

}

void button_press(){ 
  // Function to call from button_interrupt in order to limit the logic in the interrupt
  state++;   // increment the state
  if(debug){ // Send to Serial for debug
    Serial.println("Button pressed function");
  }
}

void button_interrupt(){ // Button interrupt function to run every time the button is pressed
  noInterrupts();
  // variables for checking debounce and double press
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  unsigned long time_between = (interrupt_time - last_interrupt_time);
  
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if ((time_between > 200)){
    if(time_between > 1000){
      // If the interrupt comes in twice within one second, 
      button_press();
    }else{
      if(debug){
        Serial.println("long button press");  
      }
      state = 5;
      current_time=1;
    }
  }
  
  last_interrupt_time = interrupt_time;
  interrupts();
  return;
}



void buzzer(byte i){
  while(i){
  tone(BUZZER_PIN, 2000); // Set the voltage to high and makes a noise
  delay(100);             // Waits for 100 milliseconds
  noTone(BUZZER_PIN);     // Sets the voltage to low and makes no noise
  delay(100);             // Waits for 100 milliseconds
  if(debug){
    Serial.print("buzzer beep");
  }
  i--;
  }
  return;
}

void display_done(){      // Function to run when timer is done
  // Display text
  Oled.setCursor(0,0);    
  Oled.print("Timer done! \n Get Moving!");
  Oled.refreshDisplay();
  buzzer(5);            //Beep Buzzer 5 times
  delay(1000);
}

void clear_disp(){      //Sets the Oled to all spaces
  Oled.setCursor(0,0);
  Oled.print("                     \n                 ");
  Oled.refreshDisplay();
  
}

String espData(String command, const int timeout, boolean debug) {
  String response = "";
  espSerial.println(command);  //send data to ESP8266 using serial UART
  long int time = millis();
  while ((time + timeout) > millis()) {  //wait the timeout period sent with the command
    while (espSerial.available()) {      //look for response from ESP8266
      char c = espSerial.read();
      response += c;
      Serial.print(c);  //print response on serial monitor
    }
  }
  if (debug) {
    Serial.println("Resp: " + response);
  }
  response.trim();
  return response;
}
