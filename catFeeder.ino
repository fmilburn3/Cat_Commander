/* Cat Commander ntpTime_v7
 * Tested on Arduino MKR 1010 with MKRRGB shield
 *  
 * Gets time from NTP server and activates servo by manual button push
 *  
 * Servo Leads:  Red = Power from separate source
 *               Brown = Ground connected to Arduino
 *               Yellow = Servo control, Arduino pin 5
 * Button: Purple = Power from Arduino Vcc to button switch
 *         Brown = Ground connected to Arduino
 *         Blue = Button swich pulled down connected to Arduino pin 6
 *         White = LED on button switch connected thru 680R to Arduino pin 7
 * 
 * by fmilburn Jan 2021
 * 
 * This code is in the public domain
 */

#include "arduino_secrets.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
#include <Servo.h>
#include <ArduinoGraphics.h>      // Arduino_MKRRGB needs ArduinoGraphics
#include <Arduino_MKRRGB.h>     

#define DEBUG false

const int SERVOPIN = 5;           // Servo control pin
const int SPEED =   30;           // Servo speed
const int BUTTONLED = 7;          // LED inside button switch
const int BUTTONPIN = 6;          // Button switch pin, 10k pulldown
volatile bool buttonPushed = false;

char ssid[] = SECRET_SSID;        // network SSID (name)
char pass[] = SECRET_PASS;        // network password
int keyIndex = 0;                 // network key Index number (needed for WEP)
int status = WL_IDLE_STATUS;

const int GMT_OFFSET = -8;        // local offset from GMT
struct Feeding{
  int h;                          // hr (24 hour) to feed cats
  int m;                          // minute of hour to feed cats
}; 
const int FEEDINGTIMES = 2;
struct Feeding feedingTime[FEEDINGTIMES] = {
                                  {18, 30},    // eg 08:00
                                  {18, 31}     // eg 16:30
                                };

int feedingTimeIndex = 0;
volatile bool timeToFeed = false; // turns true when feed time occurs
unsigned int feedRate = 2;        // number of seconds the feed motor runs

WiFiServer server(80);            // server socket
WiFiClient client = server.available();

RTCZero rtc;                      // Real Time Clock instance
Servo myservo;                    // A servo instance to feed the cats

// -----------------------------------------------------------------
// setup
// -----------------------------------------------------------------
void setup() {

  if (DEBUG == true){
    Serial.begin(115200);
    while (!Serial){
      // wait for serial
    }
    Serial.println("Started...");
  }

  initLedMatrix();
  initGpio();
  initWifi();
  initRtc();
  setFeedTime();
  initServer();
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonPush, HIGH);
  rtc.attachInterrupt(rtcAlarm);

}

// -----------------------------------------------------------------
// loop
// -----------------------------------------------------------------
void loop() {

  displayTime();

  client = server.available();

  if (client) {
    printWEB();
  }
  
  if (buttonPushed == true){     
    feedCats();                  // handle button push          
  }

  if (timeToFeed == true){       
    feedCats();                  // feed the cats
  }
}

// -----------------------------------------------------------------
// initialize the server
// -----------------------------------------------------------------
void initServer(){

  server.begin();
  printWifiStatus();
  
}

void printWEB() {

  if (client) {                             // if you get a client,
    // Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        // Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Display title
            client.print("Cat Commander Kibble Dispenser");
            client.print("<br><br>");
            
            // Display feed settings
            client.print("Feed Rate: ");
            client.print(feedRate);
            client.print(" seconds<br><br>");

            //create the feed buttons
            client.print("Click <a href=\"/MORE\">here</a> increase feed rate<br>");
            client.print("Click <a href=\"/LESS\">here</a> decrease feed rate<br><br>");

            client.print("Click <a href=\"/FEED\">here</a> to feed now<br><br>");
                     
            client.println();    
                    
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if (currentLine.endsWith("GET /MORE")) {
          if (feedRate < 10){
            feedRate++;
          }
          feedRate++;        
        }
        if (currentLine.endsWith("GET /LESS")) {
          if (feedRate > 0) {
            feedRate--;
          }   
        }
        if (currentLine.endsWith("GET /FEED")) {
          // Check for last feeding?
          feedCats();       
        }

      }
    }
    // close the connection:
    client.stop();
    // Serial.println("client disconnected");
  }
}

// -----------------------------------------------------------------
// print the WiFi Status
// -----------------------------------------------------------------
void printWifiStatus() {
  
  IPAddress ip = WiFi.localIP(); 
  MATRIX.beginText(0, 0, 127, 127, 127);
  MATRIX.print("  http://");
  MATRIX.print(ip);
  MATRIX.endText(SCROLL_LEFT);
  
  if (DEBUG == true){
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
  
    // print your board's IP address:
    Serial.print("IP Address: ");
    Serial.println(ip);
  
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
  
    Serial.print("To see this page in action, open a browser to http://");
    Serial.println(ip); 
  }
  
}

// -----------------------------------------------------------------
// initialize GPIO
// -----------------------------------------------------------------
void initGpio(){

  pinMode(BUTTONPIN, INPUT);
  pinMode(BUTTONLED, OUTPUT);
  digitalWrite(BUTTONLED, HIGH);
}

// -----------------------------------------------------------------
// initialize and start WiFi
// -----------------------------------------------------------------
void initWifi(){
  
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    while (true){
      MATRIX.println("No WiFi");
      MATRIX.endText(SCROLL_LEFT);
    }
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
}

// -----------------------------------------------------------------
// set feeding time by setting rtc alarm
// -----------------------------------------------------------------
void setFeedTime(){

  // adjust for GMT offset
  int adjTime = feedingTime[feedingTimeIndex].h - GMT_OFFSET;
  if (adjTime < 0){
    adjTime = adjTime + 24;
  }
    if (adjTime > 24){
    adjTime = adjTime - 24;
  }

  // set alarm to feed
  rtc.setAlarmTime(adjTime, feedingTime[feedingTimeIndex].m, 0);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // set index to next feeding time
  feedingTimeIndex++;
  if (feedingTimeIndex >= FEEDINGTIMES){
    feedingTimeIndex = 0;
  }
}

// -----------------------------------------------------------------
// initialize the Real Time Clock
// -----------------------------------------------------------------
void initRtc(){
  
  rtc.begin();
  unsigned long epoch = 0;
  do {
    epoch = WiFi.getTime();
    delay(100);
  } while (epoch == 0);

  rtc.setEpoch(epoch + GMT_OFFSET);  
}

// -----------------------------------------------------------------
// initialize the LED matrix
// -----------------------------------------------------------------
void initLedMatrix(){
  MATRIX.begin();
  MATRIX.brightness(5);
  MATRIX.textScrollSpeed(200);
  MATRIX.beginText(0, 0, 127, 127, 127); // X, Y, then R, G, B
}

// -----------------------------------------------------------------
// display Time
// -----------------------------------------------------------------
void displayTime(){

  MATRIX.beginText(0, 0, 127, 127, 127);
  MATRIX.print("  ");
  int timeH = rtc.getHours() + GMT_OFFSET;
  if (timeH < 0){
    timeH = timeH + 24;
  }
    if (timeH > 24){
    timeH = timeH - 24;
  }
  if (timeH > 12) {
    timeH = timeH - 12;
  }
  MATRIX.print(timeH);
  MATRIX.print(":");
  int timeM = rtc.getMinutes();
  if (timeM < 10){
    MATRIX.print("0");
  }
  MATRIX.println(timeM);
  MATRIX.endText(SCROLL_LEFT);
}

// -----------------------------------------------------------------
// feed the cats!
// -----------------------------------------------------------------
void feedCats(){
  
  if (feedRate > 0){
    // count down
    int i;
    for (i=10; i>0; i--){
      MATRIX.beginText(0, 0, 127, 0, 0);
      if (i < 10){
        MATRIX.print(" ");
      }
      MATRIX.println(i);
      MATRIX.endText();
      delay(1000);
    }
    MATRIX.println("  BLAST OFF! ");
    MATRIX.endText(SCROLL_LEFT);
  
    // servo
    myservo.attach(SERVOPIN);
    myservo.write(SPEED);
    delay(1000 * feedRate);  
    myservo.detach();
  }
  
  // restore button
  pinMode(BUTTONLED, HIGH);
  buttonPushed = false;

  // mark cats as fed
  timeToFeed = false;

  // set the next feeding time
  setFeedTime();
}  

// -----------------------------------------------------------------
// ISR for button
// -----------------------------------------------------------------
void buttonPush(){
  
  // check to see if the button has already been pushed
  if (buttonPushed == false){
    // show button as inactive by turning off LED
    pinMode(BUTTONLED, LOW);
    buttonPushed = true;
  }
}

// -----------------------------------------------------------------
// ISR for rtc alarm
// -----------------------------------------------------------------
void rtcAlarm(){

  // Time to feed the cats!
  timeToFeed = true;
}
