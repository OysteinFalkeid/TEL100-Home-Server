#include "driver/timer.h" //Dedicated driver for the timer peripherals 
#include <stdio.h> // Deddicatet driver for the UART comunication peripheral
#include <WiFi.h> //Premade code for running a number of WiFi services
#include <mDNS.h> //This pack enabels us to create a custum name for the server

//Defines helps uss predefine values used multiple times in the code.
//Defines are onley handeled by the compiler alowing us humens write more understandable code. and to reduse repetition.
#define INTERNAL_LED  2 //The pin adress of the power led indicating the ESP32 hase bootet corectley
#define HOSTNAME      "server" //The name we want oher server to have
#define Timer_StartValue 0x00000000ULL //the hexadesimal value of an adres in the peripheral timer
#define Timer_prescaler 80 //The prescaler determens the speed at witch the timer counts at

#define ssid "ssid"
#define password "password"
#define Timout_time_of_client_request 1000000

#define BUZZER_TIMER_GROOP TIMER_GROUP_0 //Here we rename a define so as to let the reader of the code better understand
#define BUZZER_TIMER TIMER_0

//Anotter way to define something is wit struct. Some programing languages cals theese objects or clases
struct Button { //this is a struct defining the name button for a list of variables
  const uint8_t PIN; //the pin location of the button
  uint32_t numberKeyPresses; // an internal counter to log the amount of times the button has ben pressed
  bool pressed; // This variable can be used in functions to register long presses or while pressed

  Button(uint8_t P, uint32_t numberKeyP, bool press) : PIN(P) , numberKeyPresses(numberKeyP), pressed(press) {} // this line defines a way to create the object later. 
};

//this struct dose not contain a declaration of how to create the object 
//however as we can se later the object can be created identicaly to the button struct
//this is a more basic aproach and can lead to error 
struct LED {
  const uint8_t PIN;
  uint8_t state;
  String HTML_on;
  String HTML_off;
};

struct Encoder {
  int64_t value;
  int64_t minValue;
  int64_t maxValue;

  Encoder(int64_t v, int64_t minV, int64_t maxV) : value(v), minValue(minV), maxValue(maxV) {}

  //we candefine functions within a struct. this way we can have more fredom over the naming of a function and the variables within it without variable names colliding.
  float percentage() const {//Insted of storing a lot of different values of the same paramater we can store the base value. And calculate other values as needed
    return (static_cast<float>(value) / (maxValue - minValue)) * 100; //the function returns a value in this case a prosentage convertion of the encoder value
  }
};

struct Moisture {//A generic moisture sensor
  uint8_t PIN;
  uint16_t value;
  uint16_t minValue;
  uint16_t maxValue;

  Moisture(uint16_t P, uint16_t v, uint16_t minV, uint16_t maxV) : PIN(P), value(v), minValue(minV), maxValue(maxV) {}

  float MoisturePrecentage() {//calculations for precentage moisture levle
    float moisturePrecentage;
    moisturePrecentage = (float)maxValue - (float)analogRead(PIN);
    moisturePrecentage = moisturePrecentage * 100;
    moisturePrecentage = moisturePrecentage / ((float)maxValue-(float)minValue);
    return moisturePrecentage;
  }
};

struct Buzzer {// a generic buser
  uint8_t PIN;
  bool State;

  Buzzer(uint8_t P) : PIN(P) {}//we can define that it is onley nesecary to spesify the pin of the buzzer and not all variables

  void Tone(int Hz,unsigned long time) {//this function plays a tone of frequencey Hz for a duration time
    unsigned long startTime = millis(); //millis is a buildt in function getting the time in milli secounds since start.
    unsigned long currentTime = startTime;
    //Serial.println(currentTime - startTime);
    while ((currentTime - startTime) < time) {//the loop runs until the tone time has compleated
      State = State ^ 1;
      digitalWrite(PIN, State);
      delay(1000 / Hz); //the delay function stops the CPU for a sertan amaount of time. this is not recomended as it ihibits other functions from worcing during this time
      currentTime = millis();
    }
    digitalWrite(PIN, 0);
  }
};

//all the structs and defines cold have bin stored in a seperate file caled ESP32.h and included making for a more readable code.

// a struct is onley a set of rules for to use these predefined rules we have to create objects

Button button1 = {18, 0, false}; // there is a button conected to pin 18 it has not bin presed and is currently not presed
LED LED1 = {INTERNAL_LED, 0, "GET /12/on HTTP/1.1", "GET /12/off HTTP/1.1"}; // the inbuildt LED is connected to PIN 2 we have defined this earlyer making the code more readable
Encoder encoder = {10, 0, 100};// the encoder can be conected to pin 10. Change min and max values acording to preferences or type of encoder
uint64_t timerValue;
Moisture moisture = {34, 0, 2000, 4096}; // the moisture sensor is connected to pin 34. My testing gave 2000 as 0% moisture and 4096 as 100% 
Buzzer buzzer = (23);
volatile int buzzerState = 0;
bool toggleFlag = false;
uint64_t i = 0;
uint64_t iMax = 62;
uint64_t Hz = 4000;
timer_config_t BuzzerTimer;
String buzzer_Hz;
String buzzer_time;


//The WiFi.h file dose not contain a method of interpereting HTML messenges form a web browser.
//the MyHomeServer struct aims to create a function that 
  //includes WiFi.h
  //uses functionality from WiFi.h to start a networc conection
  //uses NetworkClient.h included in WiFi.h to open port 80
  //uses NetworkClient.h included in WiFi.h to listen for clients
  //runs functions to interpret inncoming requests and print the corect web page
struct MyHomeServer {
  WiFiServer Server(80);

  const char* ssid;
  const char* password;
  String header;
  uint64_t currentTime;
  uint64_t previousTime;
  uint64_t timeoutTime;
  String WebPage;
  WiFiClient client;

  // This string in particluar facinates me
  //the string contains an entire image stored as base64 
  //imiges are stored as bytes of innformation however base64 gives us the opertunity to hardcode every "byte" as a char.
  String imageString = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAAFiUAABYlAUlSJPAAAABpSURBVDhPtY0LCsAwCEO9/6UdlUac35axB6LVJCX+SBtARFId5RXmKSS9eFMXEraVuNzvLlQikN31NZmB18l0agZWr67bABACkP4SmV247a5L3xd2XowBKJC9QQjI8Lf/Am4K1N8ewfwAwc4z96U4IzMAAAAASUVORK5CYII=";
  String output12State = "off";
  String output14State = "off";

  //to display innformation about the led the LED1 object hase to be refrenced from inside another struct
  //this is not the best solution but will not cause anny trubble as long as LED1 is defined before MyHomeServer
  const int output12 = LED1.PIN;
  const int output14 = 14;
  String currentLine = "";
  bool isPOST = false;
  String POSTLine = "";
  int POSTLength;
  String POSTvalue;

  //there are a lot of variables used in varius functions. but the struct onley needs 4 variables to run corectley
  MyHomeServer(const char* ss, const char* pas, const long timeTime) : ssid(ss), password(pas), timeoutTime(timeTime), Server(80) {}

  //the main part of the server.
  //this function has to run innside a while loop of some sort indefenetlrey to listen to potential clients.
  void ServerMain() {
    char c;

    client = Server.available();   // Listen for incoming clients
    if (client) {
      timer_set_counter_value(BUZZER_TIMER_GROOP ,BUZZER_TIMER ,Timer_StartValue); //set timer for 0
      timer_get_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER, &currentTime);      // If a new client connects,
      previousTime = currentTime;
      currentLine = "";
      if (client.connected()) {
        //Serial.println("client");
      }
      while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client is connected
        ////Serial.println(currentTime - previousTime);
        timer_get_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER, &currentTime);
        if (client.available()) {             // if there's bytes to read from the client,
          //Serial.println("Available");
          c = client.read();             // read a byte, then
          header += c;
          Serial.print(c);
          if (c == '\n') { // "\n" is a unicode char caled new line meaning the message has bin completed   
            if (currentLine.length() == 0 && isPOST)  { //a client can post innformation to the server this innformation is sendt after the http request header
              //Serial.println("POST !");
              while (POSTLine.length() < POSTLength && currentTime - previousTime <= timeoutTime) { //the client gives the post length in the http header
                timer_get_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER, &currentTime);
                c = client.read();
                POSTLine += c;
                //Serial.print(c);
                //As the ESP32 dose not have eany idea of the innformation of the post message we have to test for the contets.
                if (POSTLine.length() == POSTLength) { 
                  if (POSTLine.indexOf("price") > -1){//if the client is trying to send a price we store this value 
                    POSTvalue = POSTLine.substring(6);//as this is connected to a test form in the home menue we store this value to the POSTvalue
                  }
                  else if (POSTLine.indexOf("buzzer_Hz=") > -1){
                    buzzer_Hz = POSTLine.substring(10,POSTLine.indexOf("buzzer_time="));//however we want to diferensiate different POST messeges later.
                    buzzer_time = POSTLine.substring(POSTLine.indexOf("buzzer_time=") + 12); //Therefor we store every type of POST valu individualy
                  }
                }
              }
              ServerWriteWebPage(); //the POST message is the last part of the http request therefor we write the web page requested by the user and break out
              break;
            }          
            else if (currentLine.length() == 0) {// After the http header and the brfor the POST message there is an emptey line.
              ServerWriteWebPage();//if there was no indication of POST the server returns the web page and breaks out
              break;
            } 
            else {
              if (currentLine.startsWith("POST")) { //defining if comand is post og get
                isPOST = true;
                //Serial.println("isPOST");
              }

              if (WebPage.length() == 0) { //getting web adress
                //Serial.println("getting web adress");
                if (currentLine.indexOf("HTTP/1.1") >= 0) {//the line containing the HTTP adres contains the path to the page the user wants to display 
                  if (currentLine.indexOf("/Form") >= 0) {
                    WebPage = "Form";
                  }
                  else if (currentLine.indexOf("/Buzzer") >= 0) {
                    WebPage = "Buzzer";
                  }
                  else if (currentLine.indexOf("/Home") >= 0) {
                    WebPage = "Home";
                  }
                  else {
                    WebPage = "Home";
                  }
                  //Serial.println(WebPage);
                }
              }
              
              
              if (currentLine.indexOf("Content-Length: ") >= 0){ //gettign length of post line
                POSTLength = currentLine.substring(16).toInt();
                //Serial.println(POSTLength);
              }

              currentLine = "";

            }
          //"\r" the return char tels us to return to the beginning of the line. 
          //We also know a http request ends every line with "\r\n" letting us know the innformation has bin written and a new line is coming
          } else if (c != '\r') { 
            currentLine += c;
          }
        }
      }
      //ServerLedControl();
      
      
      currentLine = "";
      WebPage = "";
      POSTLine = "";
      header = "";
      POSTLength = 0;
      isPOST = false;
      // Close the connection
      client.stop();
    }
    // Clear the header variable
    //Serial.println("");
  }

  //The respons from the server to the client has a standard settup
  //HTML is usualy writen in a .txt file starting with the declaration of the type of language used
  void ServerStyle() {
    // HTTP headers always start with a response code
    client.println("HTTP/1.1 200 OK"); //the respons states that the request was a sucsess. giving the client the code 200
    //other codes can give the user input of wat cinds of trubles there might be with the communication.
    client.println("Content-type:text/html"); //Wi are writing HTML
    client.println("Connection: close");
    client.println();
    // Display the HTML web page
    client.println("<!DOCTYPE html><html>");//The programing language
    client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");//the scaling of the page
    client.println("<link  rel='shortcut icon' type='image/png' href=\"");//The innformation about the icon
    client.println(imageString);
    client.println("\">");
    // CSS to style the buttons
    // CSS is a powerful addition to HTML giving a lot of the processing load to the client.
    client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
    client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
    client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
    client.println(".button2 {background-color: #555555;}</style></head>");
  }

  void ServerHome() {

    client.print("<head><title>Home</title></head>");

    // Web Page Heading
    client.println("<body><h1>Home page</h1>");

    client.println("<form action=\"/Form\" method=\"POST\">");
    client.println("<input type=\"text\" name=\"price\" />");
    client.println("<input type=\"submit\" value=\"Send\" />");
    client.println("</form>");

    client.println("<p>Last POST value was " + POSTvalue + "</p>");

    
    // Display current state, and ON/OFF buttons for GPIO 12  
    client.println("<p>GPIO 12 - State " + output12State + "</p>");
    // If the output12State is off, it displays the ON button      
    if (output12State=="off") {
      client.println("<p><a href=\"/12/on\"><button class=\"button\">Turn ON</button></a></p>");
    } else {
      client.println("<p><a href=\"/12/off\"><button class=\"button button2\">Turn OFF</button></a></p>");
    }
    
    
    client.println("<p>Encoder percentage " + String(encoder.percentage()) + "</p>");
    
    client.println("<p>Moisture level" + String(moisture.MoisturePrecentage()) + "%<p/>");
    client.println("<p><p/>");
    client.println("<p>Buzzer<p/>");
    client.println("<form action=\"/Buzzer\" method=\"POST\">");
    client.println("<input type=\"text\" name=\"buzzer_Hz\" value=\"4000\"/>");
    client.println("<input type=\"text\" name=\"buzzer_time\" value=\"500\"/>");
    client.println("<input type=\"submit\" value=\"Send\" />");
    client.println("</form>");

    client.println("</body></html>");

    // The HTTP response ends with another blank line
    client.println();
  }

  void ServerLedControl() {
    if (header.indexOf(LED1.HTML_on) >= 0) {
      //Serial.println("GPIO 12 on");
      output12State = "on";
      digitalWrite(output12, HIGH);
    } 
    else if (header.indexOf(LED1.HTML_off) >= 0) {
      //Serial.println("GPIO 12 off");
      output12State = "off";
      digitalWrite(output12, LOW);
    }
  }

/*
  void SelectWebPage() {
    if (currentLine.indexOf("/Form") > 0) {
      WebPage = "Form";
      //Serial.println("Page Form Selected");
    }
    else if (currentLine.indexOf("/Buzzer") > 0) {
      WebPage = "Buzzer";
      //Serial.println("Page Buzzer Selected");
    }
  }
*/

  void ServerWriteWebPage() {
    //Serial.println("Printing page" + WebPage);

    ServerStyle();

    if (WebPage == "Form") {
      client.print("<head><title>Form</title></head>");
      client.println("<p><a href=\"/Home\"><button class=\"button\">Home</button></a></p>");
    }
    else if (WebPage == "Buzzer"){
      client.print("<head><title>Buzzer</title></head>");
      client.println("<p><a href=\"/Home\"><button class=\"button\">Home</button></a></p>");
      toggleFlag = true;
    }
    else {
      ServerHome();
    }
  }

};


//an object is created from the MyHomeServer struct
MyHomeServer server = {ssid, password, Timout_time_of_client_request};

//Inn some laguages the callback function is powerfull tool
//an a microcontroler an interupt can be used for the same functionality.
//this interupt is conected to the pin of the onboard button
void IRAM_ATTR Button_Press_isr() {
  timer_get_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER, &timerValue);//reusing the buzzer timer

  if (timerValue > 150000){//debounsing in software to ensure the button is onley registered once per pres
    button1.pressed = true;
    timer_set_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER,0); //set timer for 0
  }
}

void IRAM_ATTR Buzzer_isr (void *param){//this interupt is connected to the timer and makes buzzer go buzzzzz
    timer_group_clr_intr_status_in_isr(BUZZER_TIMER_GROOP,BUZZER_TIMER); //clers the interupt flag form the register in the timer peripheral
    
    i += 1; //counts the number of times the interupt has run
    if (i < iMax) {//the time duration of the buzzing defines iMax based on the frequencey
      //the timer interupt has to be spesifcaly reanabled after every interupt.
      //as observed from the button interupt this is not a standard behavior for every interupt.
      timer_group_enable_alarm_in_isr(BUZZER_TIMER_GROOP, BUZZER_TIMER);
    }
    //toggleFlag = true;
    digitalWrite(23, buzzerState); //writes to the buzzer pin
    buzzerState = buzzerState ^ 1; //alternates the state of the buzzer with minnium computation
    
}


//this is the boot cycle of the ESP32 and is where the CPU starts when running the code
void setup() {
  pinMode(LED1.PIN,OUTPUT);
  digitalWrite(LED1.PIN,1); //Indicates that the controler is starting
  Serial.begin(115200); //Enabeling the UART driver included in stdio.h
  //Serial.println("Serial begin");
  pinMode(button1.PIN, INPUT_PULLUP);
  //Serial.println("Button pin configured");
  pinMode(buzzer.PIN,OUTPUT);
  //Serial.println("Buzzer pin configured");
  pinMode(moisture.PIN, INPUT);
  //Serial.println("Moisture pin configured");
  //this function attaches an interupt to the button pin in hardware removing load from the CPU
  attachInterrupt(button1.PIN, Button_Press_isr, FALLING);
  //Serial.println("Button interupt configured");
  WiFiConfig();
  //I had to create a custum driver for the timer as I cold not find anny drivers maching my neads.
  //The driver is based uppon a struct from driver/timer.h 
  //and has bin configuered using inbuildt functionality in Arduino IDE
  TimerConfig();
  //Serial.println("Tone timer configured");
  digitalWrite(LED1.PIN,0);
}

void loop() {
  server.ServerMain();

  //The buzzer functionality from ServerWriteWebPage() is programd in an if statemnt
  if (toggleFlag) { // Check if the ISR was called
    Serial.println("");
    timer_pause(BUZZER_TIMER_GROOP ,BUZZER_TIMER);
    uint64_t timerAlarmValue;
    i = 0;
    Hz = buzzer_Hz.toInt() * 2; //calculates the frequency of the timer interupt
    timerAlarmValue = 1000000 / Hz; //converts the frequency to a timer value
    iMax = Hz * buzzer_time.toInt(); 
    iMax = iMax / 1000;//calculates the number of times the buzzer has to go buzz lo last for the spesified duration with the spesified frequencey
    timer_set_counter_value(BUZZER_TIMER_GROOP ,BUZZER_TIMER ,Timer_StartValue);
    timer_get_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER, &timerValue);
    //Serial.println(timerValue);
    timer_set_alarm_value(BUZZER_TIMER_GROOP,BUZZER_TIMER,timerAlarmValue);
    timer_set_alarm(BUZZER_TIMER_GROOP,BUZZER_TIMER, TIMER_ALARM_EN);
    timer_start(BUZZER_TIMER_GROOP,BUZZER_TIMER);
    toggleFlag = false; // Reset the flag
  }
  //timer_get_counter_value(BUZZER_TIMER_GROOP,BUZZER_TIMER, &timerValue);
  //Serial.println(timerValue);
}

void TimerConfig() {
  uint64_t timerAlarmValue;
  BuzzerTimer.divider = Timer_prescaler; //Set prescaler for 1 MHz clock
  BuzzerTimer.counter_dir = TIMER_COUNT_UP;
  BuzzerTimer.counter_en = TIMER_PAUSE;
  BuzzerTimer.alarm_en = TIMER_ALARM_EN;
  BuzzerTimer.auto_reload = TIMER_AUTORELOAD_EN; // Reset timer to 0 when end condition is triggered
  BuzzerTimer.intr_type = TIMER_INTR_LEVEL;
  BuzzerTimer.clk_src = TIMER_SRC_CLK_APB;

  timer_init(BUZZER_TIMER_GROOP ,BUZZER_TIMER ,&BuzzerTimer); //start timer 0 at group 0
  timer_set_counter_value(BUZZER_TIMER_GROOP ,BUZZER_TIMER ,Timer_StartValue); //set timer for 0
  //timer_set_auto_reload(BUZZER_TIMER_GROOP ,BUZZER_TIMER ,TIMER_AUTORELOAD_EN);
  timerAlarmValue = 1000000/Hz;
  timer_set_alarm_value(BUZZER_TIMER_GROOP,BUZZER_TIMER,timerAlarmValue);
  timer_enable_intr(BUZZER_TIMER_GROOP,BUZZER_TIMER);//enabels the timer interupt
  timer_isr_register(BUZZER_TIMER_GROOP , BUZZER_TIMER , &Buzzer_isr , (void *)&buzzerState , ESP_INTR_FLAG_IRAM , NULL);

  timer_start(BUZZER_TIMER_GROOP,BUZZER_TIMER);
}

void WiFiConfig() {
  //WiFi.mode(WIFI_STA); //Optional
  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  //WiFi.setHostname("esp32-1cb400");
  WiFi.begin(server.ssid, server.password);
  Serial.println("\nConnecting");

  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }
  
  server.Server.begin();
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP32 HostName: ");
  Serial.println(HOSTNAME);
  Serial.print("RRSI: ");
  //Serial.println(WiFi.RSSI());

  mdns_init(); 
  mdns_hostname_set(HOSTNAME); 
  mdns_instance_name_set(HOSTNAME); 
  Serial.printf("MDNS responder started at http://%s.local\n", HOSTNAME);
}













