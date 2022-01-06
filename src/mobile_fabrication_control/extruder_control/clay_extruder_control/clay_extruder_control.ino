#include <AccelStepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Chrono.h>

// Set the shield settings
//
byte mac[] = { 0xA6, 0x61, 0x0A, 0xAE, 0x19, 0x65 };
IPAddress ip(192, 168, 0, 220);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
int server_port = 50004;

EthernetServer server(server_port);

const int dir_pin = 9;
const int step_pin = 10;

const int enable_pin = 2;
const int m1_pin = 3;
const int m2_pin = 4;
const int m3_pin = 5;

AccelStepper motor(AccelStepper::DRIVER, step_pin, dir_pin);

boolean CONNECTED = false;

Chrono motor_state_print;
Chrono check_client_interval;

// Structured data =================================================================================
struct HEADER {
  long msg_type;
  long msg_len; 
  long msg_response;
} ;

struct HEADER recv_header;

struct MOTORDATA {
  long motor_state = 0;
  float motor_maxspeed = 1600; 
  float motor_speed = 1200;
} motordata;

struct INFO {
  String arduino_ip = "192.168.0.220";
  String arduino_gateway = "192.168.1.1";
  String arduino_subnet = "255.255.255.0";
  long arduino_port = 50004;
} info;

struct DODATA {
  long digital_out_pin = 0;
  long do_pin_state = 0;
} dodata;

int digital_pins[54];
// Digital pins assignment
// 0 = None, 1 = Output, 2 = Input, 3 = AccelStepper

// Message types ===================================================================================
const int MSG_RECEIVED = 0;
const int MSG_EXECUTED = 1;
const int MSG_STOP = 2;
const int MSG_INFO = 3;
const int MSG_DODATA = 4;
const int MSG_MOTORDATA = 5;
const int MSG_MOTORSTATE = 6;
int OUTPUT_STATE = 0;

String msg_names[] = {"MSG_RECEIVED" , "MSG_EXECUTED", "MSG_STOP", "MSG_INFO", "MSG_DODATA", "MSG_MOTORDATA", "MSG_MOTORSTATE"};

// Message properties ==============================================================================
const int RETURN_MSG_LEN = 0;
const int RETURN_MSG_RSP = 0;

// Execute methods =================================================================================
void set_digital_out() {
  if (digital_pins[dodata.digital_out_pin] != 2) {
    setup_digital_pin(dodata.digital_out_pin, 2);
  }
  digitalWrite(dodata.digital_out_pin, dodata.do_pin_state);
  Serial.println("Set digital out");
}

void setup_digital_pin(int pin, int mode) {
  if (mode == 1){
    pinMode(pin, INPUT);
  }
  else if (mode == 2){
    pinMode(pin, OUTPUT);
  }
  else {
    // Serial.print("Pin mode invalid");
  }
  digital_pins[pin] = mode;
}

// SETUP ===========================================================================================
void setup()
{
  // initialize the ethernet device
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  // Open serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
    
  // start listening for clients
  motor_init();
  server.begin();
  Serial.print("Chat server address:");
  Serial.println(Ethernet.localIP());
}

// LOOP ============================================================================================

void loop(){
  check_motor();
  if (check_client_interval.hasPassed(100)) {
    check_for_client();
  }
}
