
/*
 * To use these examples you need to connect the VL53L7CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (GND) of the VL53L7CX satellite connected to GND of the Nucleo board
 * pin 2 (IOVDD) of the VL53L7CX satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (AVDD) of the VL53L7CX satellite connected to 5V pin of the Nucleo board
 * pin 4 (PWREN) of the VL53L7CX satellite connected to pin A5 of the Nucleo board
 * pin 5 (LPn) of the VL53L7CX satellite connected to pin A3 of the Nucleo board
 * pin 6 (SCL) of the VL53L7CX satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 7 (SDA) of the VL53L7CX satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 8 (I2C_RST) of the VL53L7CX satellite connected to pin A1 of the Nucleo board
 * pin 9 (INT) of the VL53L7CX satellite connected to pin A2 of the Nucleo board
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>
#include <unistd.h>
#include <optional>


int counter = 0;
uint16_t Lthreshold = ;
uint16_t Hthreshold = ;

class StateMachine {
public:
    enum class State {
        Init,
        Idle,
        PossibleEntryStarted,
        EntryThroughDoorway,
        PossibleExitStarted,
        ExitThroughDoorway
    };

    enum class Event {
        OutsideO,
        InsideO,
        MiddleA,
        InsideReached,
        OutsideReached,
        Clear
    };

    StateMachine() : currentState(State::Init) {}

    void handleEvent(Event event) {
        switch (currentState) {
            case State::Init:
                transitionTo(State::Waiting);
                break;

            case State::Waiting:
                if (event == Event::TopDetect) {
                    transitionTo(State::Top);
                } 
                else if (event == Event::BotDetect){
                    transitionTo(State::Bot);
                }

                else {
                    transitionTo(State::Waiting);
                }

                break;

            case State::Top:
                if (event == Event::Clear) {
                    transitionTo(State::Waiting);
                }
                else if (event == Event::Leave){
                    transitionTo(State::Minus);
                }
                else {
                    transitionTo(State::Top);
                }

                break;

            case State::Bot:
                if (event == Event::Clear) {
                    transitionTo(State::Waiting);
                }
                else if (event == Event::Enter){
                    transitionTo(State::Plus);
                }
                else {
                    transitionTo(State::Bot);
                }

                break;
            
            case State::Plus:
                transitionTo(State::Waiting);
                break;

            case State::Minus:
                transitionTo(State::Waiting);
                break;
        }
    }

    void update() {
        switch (currentState) {
            case State::Waiting:
                onWaiting();
                break;
            case State::Top:
                onTop();
                break;
            case State::Bot:
                onBot();
                break;
            case State::Init:
                onInit();
                break;
            case State::Minus:
                onMinus();
                break;
            case State::Plus:
                onPlus();
                break;
            
        }
    }

    State getState() const {
        return currentState;
    }

private:
    State currentState;

    void transitionTo(State newState) {
        onExit(currentState);
        currentState = newState;
        onEnter(currentState);
    }

    void onEnter(State state) {
        switch (state) {
            case State::Waiting:
                //std::cout << "Entering Waiting\n";
                break;
            case State::Top:
                //std::cout << "Entering Top\n";
                break;
            case State::Bot:
                //std::cout << "Entering Bot\n";
                break;
            case State::Init:
                //std::cout << "Entering Init\n";
                break;
            case State::Plus:
                //std::cout << "Entering Plus\n";
                break;
            case State::Minus:
                //std::cout << "Entering Minus\n";
                break;
        }
    }

    void onExit(State state) {
        switch (state) {
            case State::Waiting:
                //std::cout << "Exiting Waiting\n";
                break;
            case State::Top:
                //std::cout << "Exiting Top\n";
                break;
            case State::Bot:
                //std::cout << "Exiting Bot\n";
                break;
            case State::Init:
                //std::cout << "Exiting Init\n";
                break;
            case State::Plus:
                //std::cout << "Exiting Plus\n";
                break;
            case State::Minus:
                //std::cout << "Exiting Minus\n";
                break;
        }
    }

    void onWaiting() {
        //std::cout << "waiting\n";
        Serial.println("waiting");
       // std::cout << "counter = ";
        Serial.println("counter=");
       // std::cout << counter;
        Serial.println(counter);
       // std::cout << "\n";
    }

    void onTop() {
        //std::cout << "Top\n";
        Serial.println("Top");
    }

    void onBot() {
       // std::cout << "Bot\n";
        Serial.println("Bot");
    }

    void onInit() {
       // std::cout << "Init\n";
        Serial.println("Init");
    }

    void onPlus() {
        //std::cout << "Plus\n";
        Serial.println("Plus");
        counter++;
    }
    
    void onMinus() {
        //std::cout << "Minus\n";
        Serial.println("Minus");
        if (counter > 0) {
            counter--;
        }

    }
};

//We have this function because we need to pass an event into handleEvent(...) in int main, but want the event we pass in to be determined by a function
std::optional<StateMachine::Event> inputToEvent(bool I, bool O, bool M, StateMachine::State State) {
    if (I = 0 && M = 0 && O = 0) {
        return StateMachine::Event::Clear;
    }
    if (O = 1 && M = 0 && I = 0 && State != StateMachine::State:ExitThroughDoor) {
      return StateMachine::Event::OutsideO;
    }
    if (I = 1 && M = 0 && O = 0 && (State != StateMachine::State:EnterThroughDoor)) {
        return StateMachine::Event::InsideO;
    }
    if (M = 1) 
    {return StateMachine::Event::MiddleA;
    }
    if (I = 1 && State == EntryThroughDoorway) {
        return StateMachine::Event::InsideReached; 
    }
    if (O = 1 && State == PossibleEntryStarted) {
        return StateMachine::Event::OutsideReached;
    }
    return std::nullopt;   // no event
}


    
#define SDA_PIN 21
#define SCL_PIN 22

#define DEV_I2C Wire
#define SerialPort Serial

#define LPN_PIN 25
#define I2C_RST_PIN 32
#define PWREN_PIN 33

void print_result(VL53L7CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);
void toggle_resolution(void);
void toggle_signal_and_ambient(void);

// Components.
VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L7CX_RESOLUTION_4X4;
char report[256];
StateMachine sm;
void setup() {
  Serial.begin(460800);
  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Initialize serial for output.
  SerialPort.begin(460800);

  // Initialize I2C bus.
  DEV_I2C.begin(SDA_PIN, SCL_PIN, 400000);

  // Configure VL53L7CX component.
  sensor_vl53l7cx_top.begin();

  sensor_vl53l7cx_top.init_sensor();

  toggle_resolution();

  sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(15);

  // Start Measurements
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();

}

void loop()
{
  VL53L7CX_ResultsData Results;
  VL53L7CX_ResultsData oldResults;
  uint8_t NewDataReady = 0;
  uint8_t status;
  uint8_t* ptr;
  uint16_t maxVal = 2500;

  uint16_t AvgColumn[8];
  for (uint8_t j = 0; j < 8; j++) {
    uint8_t total = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (Results.nb_target_detected[i+(j*8)]>0) {
       total = total + Results.distance_mm[i+(j*8)];
      }
    }
    AvgColumn[7-j] = total / 8;
  }


  uint16_t binary_occupancy[8];
  for(uint8_t i = 0; i < 8; i++){
    if (AvgColumn[i] < Hthreshold && AvgColumn[i] > Lthreshold) {
      binary_occupancy[i] = 1;
    }
    else {
      binary_occupany[i] = 0;
    }
  }

  bool Inside = 0;
  bool Middle = 0;
  bool Outside = 0;
  if (binary_occupancy[0] == 1 || binary_occupancy[1] == 1 ||binary_occupancy[2] == 1){
    Inside = 1
  }
  else {
    Inside = 0;
  }
  if (binary_occupancy[3] == 1 || binary_occupancy[4] == 1){
    Middle = 1
  }
  else {
    Middle = 0;
  }
  if (binary_occupancy[5] == 1 || binary_occupancy[6] == 1 ||binary_occupancy[7] == 1){
    Outside = 1
  }
  else {
    Outside = 0;
  }

  auto event = inputToEvent(AvgColumn);

  sm.handleEvent(*event);

  sm.update();
  Serial.print(counter);
  

  do {
    status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&Results);
    //print_result(&Results);
  }
  /*
  uint8_t checksum = 0;
  Serial.write(0xAA);
  for (uint8_t z = 0; z < res; z++) {
    if (Results.nb_target_detected[z]>0) {
      ptr = (uint8_t*)&Results.distance_mm[z];
    }
    else {
      ptr = (uint8_t*)&maxVal;
    }

    Serial.write(ptr, 2);
    checksum ^= ptr[0];
    checksum ^= ptr[1];
    }
  Serial.write(checksum);
  oldResults = Results;
*/

//average the result matrix into a vector
  

  /*if (Serial.available()>0)
  {
    handle_cmd(Serial.read());
  }*/
  delay(1);
}

void print_result(VL53L7CX_ResultsData *Result)
{
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  display_commands_banner();

  SerialPort.print("Cell Format :\n\n");
  
  for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++)
  {
    snprintf(report, sizeof(report)," \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    SerialPort.print(report);

    if(EnableAmbient || EnableSignal)
    {
      snprintf(report, sizeof(report)," %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
      SerialPort.print(report);
    }
  }

  SerialPort.print("\n\n");

  for (j = 0; j < number_of_zones; j += zones_per_line)
  {
    for (i = 0; i < zones_per_line; i++) 
      SerialPort.print(" -----------------");
    SerialPort.print("\n");
    
    for (i = 0; i < zones_per_line; i++)
      SerialPort.print("|                 ");
    SerialPort.print("|\n");
  
    for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++)
    {
      // Print distance and status 
      for (k = (zones_per_line - 1); k >= 0; k--)
      {
        if (Result->nb_target_detected[j+k]>0)
        {
          snprintf(report, sizeof(report),"| \033[38;5;10m%5ld\033[0m  :  %5ld ",
              (long)Result->distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l],
              (long)Result->target_status[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l]);
              SerialPort.print(report);
        }
        else
        {
          snprintf(report, sizeof(report),"| %5s  :  %5s ", "X", "X");
          SerialPort.print(report);
        }
      }
      SerialPort.print("|\n");

      if (EnableAmbient || EnableSignal )
      {
        // Print Signal and Ambient 
        for (k = (zones_per_line - 1); k >= 0; k--)
        {
          if (Result->nb_target_detected[j+k]>0)
          {
            if (EnableSignal)
            {
              snprintf(report, sizeof(report),"| %5ld  :  ", (long)Result->signal_per_spad[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l]);
              SerialPort.print(report);
            }
            else
            {
              snprintf(report, sizeof(report),"| %5s  :  ", "X");
              SerialPort.print(report);
            }
            if (EnableAmbient)
            {
              snprintf(report, sizeof(report),"%5ld ", (long)Result->ambient_per_spad[j+k]);
              SerialPort.print(report);
            }
            else
            {
              snprintf(report, sizeof(report),"%5s ", "X");
              SerialPort.print(report);
            }
          }
          else
          {
            snprintf(report, sizeof(report),"| %5s  :  %5s ", "X", "X");
            SerialPort.print(report);
          }
        }
        SerialPort.print("|\n");
      }
    }
  }
  for (i = 0; i < zones_per_line; i++)
   SerialPort.print(" -----------------");
  SerialPort.print("\n");
}

void toggle_resolution(void)
{
  sensor_vl53l7cx_top.vl53l7cx_stop_ranging();

  switch (res)
  {
    case VL53L7CX_RESOLUTION_4X4:
      res = VL53L7CX_RESOLUTION_8X8;
      break;

    case VL53L7CX_RESOLUTION_8X8:
      res = VL53L7CX_RESOLUTION_4X4;
      break;

    default:
      break;
  }
  sensor_vl53l7cx_top.vl53l7cx_set_resolution(res);
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();
}

void toggle_signal_and_ambient(void)
{
  EnableAmbient = (EnableAmbient) ? false : true;
  EnableSignal = (EnableSignal) ? false : true;
}

void clear_screen(void)
{
  snprintf(report, sizeof(report),"%c[2J", 27); /* 27 is ESC command */
  SerialPort.print(report);
}

void display_commands_banner(void)
{
  snprintf(report, sizeof(report),"%c[2H", 27); /* 27 is ESC command */
  SerialPort.print(report);

  Serial.print("53L7A1 Simple Ranging demo application\n");
  Serial.print("--------------------------------------\n\n");

  Serial.print("Use the following keys to control application\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : enable signal and ambient\n");
  Serial.print(" 'c' : clear screen\n");
  Serial.print("\n");
}

void handle_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'r':
      toggle_resolution();
      clear_screen();
      break;

    case 's':
      toggle_signal_and_ambient();
      clear_screen();
      break;

    case 'c':
      clear_screen();
      break;

    default:
      break;
  }
}


