
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
uint16_t Lthreshold = 200;
uint16_t Hthreshold = 1400;

class StateMachine { //enter > Exiting, i>m>o
public:
	enum class State {
		//one person
		Init,
		Idle,
		Clear,
		iE00, //+1
		iX00,
		mX00,
		mE00,
		oX00, //-1
		oE00,
		//Dupplicates
		iEiE, //+2
		iXiX,
		mXmX,
		mEmE,
		oXoX, //-2
		oEoE,
		//Duplicate flipped
		iEiX, //+1
		mEmX,
		oEoX,   //-1
		//inside and middle
		iEmE, //+1
		iEmX,  //+1
		iXmE, //
		iXmX,
		//outisde middle
		mEoE,
		mXoE,
		mEoX, //-1
		mXoX, //-1
		//inside and outside
		iEoE, //+1
		iEoX,
		iXoE,
		iXoX //-1
	};

	enum class Event {
		e000,
		e110,
		e101,
		e011,
		e100,
		e010,
		e001,
		e200,
		e020,
		e002
	};

	StateMachine() : currentState(State::Init) {}

	void handleEvent(Event event) {
		switch (currentState) {
			case State::Init:
				transitionTo(State::Idle);
				break;

			case State::Idle:
				if (event == Event::e100) {
					transitionTo(State::iX00);
				}
				else if (event == Event::e001){
					transitionTo(State::oE00);
				}

				else if (event == Event::e200) {
					transitionTo(State::iXiX);
				}
				else if (event == Event::e002) {
					transitionTo(State::oEoE);
				}
				else if (event == Event::e101) {
					transitionTo(State::iXoE);
				}
				else {
					transitionTo(State::Idle);
				}

				break;

			case State::iX00:
				if (event == Event::e100) {
					transitionTo(State::iX00);
				}
				else if (event == Event::e010){
					transitionTo(State::mX00);
				}

				else if (event == Event::e200) {
					transitionTo(State::iXiX);
				}
				else if (event == Event::e101) {
					transitionTo(State::iXoE);
				}
				else if (event == Event::e110) {
					transitionTo(State::iXmX);
				}
				else {
					transitionTo(State::Idle);
				}

				break;

			case State::oE00:
				if (event == Event::e001) {
					transitionTo(State::oE00);
				}
				else if (event == Event::e010){
					transitionTo(State::mE00);
				}

				else if (event == Event::e002) {
					transitionTo(State::oEoE);
				}
				else if (event == Event::e101) {
					transitionTo(State::iXoE);
				}
				else if (event == Event::e011) {
					transitionTo(State::mEoE);
				}
				else {
					transitionTo(State::Idle);
				}

				break;

			case State::mX00:
				if (event == Event::e010) {
					transitionTo(State::mX00);
				}
				else if (event == Event::e001){
					counterM();
					transitionTo(State::oE00);
				}
				else if (event == Event::e100){
					transitionTo(State::iX00);
				}

				else if (event == Event::e110) {
					transitionTo(State::iXmX);
				}
				else if (event == Event::e011) {
					transitionTo(State::mXoE);
				}
				else if (event == Event::e002) {
					counterM();
					transitionTo(State::oEoE);
				}
				else if (event == Event::e200) {
					transitionTo(State::iXiX);
				}
				else if (event == Event::e101) {
					counterM();
					transitionTo(State::iXoE);
				}
				else if (event == Event::e020) {
					transitionTo(State::mXmX);
				}
				else {
					transitionTo(State::mX00); //RED ALERT (should not happen)
				}

				break;

			case State::mE00:
				if (event == Event::e010) {
					transitionTo(State::mE00);
				}
				else if (event == Event::e001){
					transitionTo(State::oE00);
				}
				else if (event == Event::e100){
					counterP();
					transitionTo(State::iX00);
				}

				else if (event == Event::e110) {
					transitionTo(State::iXmE);
				}
				else if (event == Event::e011) {
					transitionTo(State::mEoE);
				}
				else if (event == Event::e002) {
					transitionTo(State::oEoE);
				}
				else if (event == Event::e200) {
					counterP();
					transitionTo(State::iXiX);
				}
				else if (event == Event::e101) {
					counterP();
					transitionTo(State::iXoE);
				}
				else if (event == Event::e020) {
					transitionTo(State::mEmE);
				}
				else {
					transitionTo(State::mE00); //RED ALERT (should not happen)
				}

				break;

			case State::iXiX:
				if (event == Event::e200) {
					transitionTo(State::iXiX);
				}
				else if (event == Event::e100) {
					transitionTo(State::iX00);
				}
				else if (event == Event::e110){
					transitionTo(State::iXmX);
				}
				else if (event == Event::e020){
					transitionTo(State::mXmX);
				}
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else if (event == Event::e110) {
					counterM();
					transitionTo(State::mXoX);
				}
				else {
					transitionTo(State::iXiX);
				}

				break;

			case State::oEoE:
				if (event == Event::e002) {
					transitionTo(State::oEoE);
				}
				else if (event == Event::e001){
					transitionTo(State::oE00);
				}

				else if (event == Event::e011){
					transitionTo(State::mEoE);
				}
				else if (event == Event::e020){
					transitionTo(State::mEmE);
				}
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else if (event == Event::e110) {
					counterP();
					transitionTo(State::iXmE);
				}
				else {
					transitionTo(State::oEoE);
				}


				break;


			case State::iXoE:
				if (event == Event::e101) {
					transitionTo(State::iXoE);
				}
				else if (event == Event::e001) {
					transitionTo(State::oE00);
				}
				else if (event == Event::e100) {
					transitionTo(State::iX00);
				}

				else if (event == Event::e011){
					transitionTo(State::mXoE);
				}
				else if (event == Event::e110){
					transitionTo(State::iXmE);
				}
				else if (event == Event::e020){
					transitionTo(State::mEmX);
				}
				// ignoring e010!!!
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else {
					transitionTo(State::iXoE);
				}

				break;


			case State::iXmX: // 100 200 011 010 101 020
				if (event == Event::e110) {
					transitionTo(State::iXmX);
				}
				else if (event == Event::e100) {
					transitionTo(State::iX00);
				}
				else if (event == Event::e010) {
					transitionTo(State::mX00);
				}

				else if (event == Event::e200) {
					transitionTo(State::iXiX);
				}
				else if (event == Event::e011) {
					counterM();
					transitionTo(State::mXoE);
				}
				else if (event == Event::e101) {
					counterM();
					transitionTo(State::iXoE);
				}
				else if (event == Event::e020) {
					transitionTo(State::mXmX);
				}
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else {
					transitionTo(State::iXmX);
				}

				break;

			case State::iXmE: // 100 200 011 010 101 020
				if (event == Event::e110) {
					transitionTo(State::iXmE);
				}
				else if (event == Event::e100) {
					counterP();
					transitionTo(State::iX00);
				}
				else if (event == Event::e010) {
					transitionTo(State::mE00);
				}

				else if (event == Event::e200) {
					counterP();
					transitionTo(State::iXiX);
				}
				else if (event == Event::e011) {
					transitionTo(State::mXoE);
				}
				else if (event == Event::e101) {
					transitionTo(State::iXoE);
				}
				else if (event == Event::e020) {
					transitionTo(State::mEmX);
				}
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else {
					transitionTo(State::iXmE);
				}

				break;

			case State::mEoE: // 001 002 110 010 101 020
				if (event == Event::e011) {
					transitionTo(State::mEoE);
				}
				else if (event == Event::e001) {
					transitionTo(State::oE00);
				}
				else if (event == Event::e010) {
					transitionTo(State::mE00);
				}

				else if (event == Event::e002) {
					transitionTo(State::oEoE);
				}
				else if (event == Event::e110) {
					counterP();
					transitionTo(State::iXmE);
				}
				else if (event == Event::e101) {
					counterP();
					transitionTo(State::iXoE);
				}
				else if (event == Event::e020) {
					transitionTo(State::mEmE);
				}
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else {
					transitionTo(State::mEoE);
				}

				break;

			case State::mXoE: // 001 002 110 010 101 020
				if (event == Event::e011) {
					transitionTo(State::mXoE);
				}
				else if (event == Event::e001) {
					counterM();
					transitionTo(State::oE00);
				}
				else if (event == Event::e010) {
					transitionTo(State::mX00);
				}

				else if (event == Event::e002) {
					counterM();
					transitionTo(State::oEoE);
				}
				else if (event == Event::e110) {
					transitionTo(State::iXmE);
				}
				else if (event == Event::e101) {
					transitionTo(State::iXoE);
				}
				else if (event == Event::e020) {
					transitionTo(State::mEmX);
				}
				else if (event == Event::e000) {
					transitionTo(State::Idle);
				}
				else {
					transitionTo(State::mXoE);
				}

				break;



			case State::mXmX: // 101 200 002 110 011
				if (event == Event::e020) {
					transitionTo(State::mXmX);
				}
				else if (event == Event::e101) {
					counterM();
					transitionTo(State::iXoE);
				}
				else if (event == Event::e200) {
					transitionTo(State::iXiX);
				}
				else if (event == Event::e002) {
					counterM();
					counterM();
					transitionTo(State::oEoE);
				}
				else if (event == Event::e110) {
					transitionTo(State::iXmX);
				}
				else if (event == Event::e011) {
					counterM();
					transitionTo(State::mXoE);
				}
				else if (event == Event::e010) {
					transitionTo(State::mX00);
				}
				else {
					transitionTo(State::mXmX);
				}

				break;

			case State::mEmE: // 101 200 002 110 011
				if (event == Event::e020) {
					transitionTo(State::mEmE);
				}
				else if (event == Event::e101) {
					counterP();
					transitionTo(State::iXoE);
				}
				else if (event == Event::e200) {
					counterP();
					counterP();
					transitionTo(State::iXiX);
				}
				else if (event == Event::e002) {
					transitionTo(State::oEoE);
				}
				else if (event == Event::e110) {
					counterP();
					transitionTo(State::iXmE);
				}
				else if (event == Event::e011) {
					transitionTo(State::mEoE);
				}
				else if (event == Event::e010) {
					transitionTo(State::mE00);
				}
				else {
					transitionTo(State::mEmE);
				}

				break;

			case State::mEmX: // 101 200 002 110 011
				if (event == Event::e020) {
					transitionTo(State::mEmX);
				}
				else if (event == Event::e101) { // wacky
					transitionTo(State::iXoE);
				}
				else if (event == Event::e200) {
					counterP();
					transitionTo(State::iXiX);
				}
				else if (event == Event::e002) {
					counterM();
					transitionTo(State::oEoE);
				}
				else if (event == Event::e110) { // NOTE ambiguity
					counterP();
					transitionTo(State::iXmX);
				}
				else if (event == Event::e011) { // NOTE ambiguity
					counterM();
					transitionTo(State::mEoE);
				}
				else {
					transitionTo(State::mEmX);
				}

				break;


			// FAKE STATES (debunked) -> use iX or oE for everything to cut states
			case State::iE00:
				transitionTo(State::Idle);
				break;

			case State::oX00:
				transitionTo(State::Idle);
				break;


		}
	}
	void counterP() {
	  counter++;
	}
	void counterM() {
	  if(counter > 0){
		counter--;
	  }
	}

	void update() {
		switch (currentState) {
			case State::Init:
				onInit();
				break;
			case State::Idle:
				onIdle();
				break;
			case State::Clear:
				onClear();
				break;
			case State::iE00: // debunk
				oniE00();
				break;
			case State::iX00:
				oniX00();
				break;
			case State::mX00:
				onmX00();
				break;
			case State::mE00:
				onmE00();
				break;
			case State::oX00: // debunk
				onoX00();
				break;
			case State::oE00:
				onoE00();
				break;
			case State::iEiE: // debunk
				oniEiE();
				break;
			case State::iXiX:
				oniXiX();
				break;
			case State::mXmX:
				onmXmX();
				break;
			case State::mEmE:
				onmEmE();
				break;
			case State::oXoX: // debunk
				onoXoX();
				break;
			case State::oEoE:
				onoEoE();
				break;
			case State::iEiX: // debunk
				oniEiX();
				break;
			case State::mEmX:
				onmEmX();
				break;
			case State::iEmE: // debunk
				oniEmE();
				break;
			case State::iEmX: // debunk
				onEmX();
				break;
			case State::oEoX:
				onoEoX();
				break;
			case State::iXmE:
				oniXmE();
				break;
			case State::iXmX:
				oniXmX();
				break;
			case State::mEoE:
				onmEoE();
				break;
			case State::mEoX:
				onmEoX();
				break;
			case State::mXoX:
				onmXoX();
				break;
			case State::iEoE: // debunk
				oniEoE();
				break;
			case State::iEoX: // debunk
				oniEoX();
				break;
			case State::mXoE:
				onmXoE();
				break;
			case State::iXoE:
				oniXoE();
				break;
			case State::iXoX:
				oniXoX();
				break;
		}
	}

	State getState() const {
		return currentState;
	}

private:
	State currentState;

	void transitionTo(State newState) {
		//onExit(currentState);
		currentState = newState;
	   // onEnter(currentState);
	}

	void onIdle() {
		//std::cout << "waiting\n";
		Serial.println("Idle");
	   // std::cout << "counter = ";
		Serial.println("counter=");
	   // std::cout << counter;
		Serial.println(counter);
	   // std::cout << "\n";
	}

	void onInit() {
	  Serial.println("Init");
	}

	void onClear() {
	  Serial.println("Clear");
	}

	void oniE00() {
	  Serial.println("iE00");
	}

	void oniX00() {
	  Serial.println("iX00");
  }

	void onmX00() {
	  Serial.println("mX00");
  }

	void onmE00() {
	  Serial.println("mE00");
  }

	void onoX00() {
	  Serial.println("oX00");
  }

	void onoE00() {
	  Serial.println("oE00");
  }

	void oniEiE() {
	  Serial.println("iEiE");
  }

	void oniXiX() {
	  Serial.println("iXiX");
  }

	void onmXmX() {
	  Serial.println("mXmX");
  }

	void onmEmE() {
	  Serial.println("mEmE");
  }

	void onoXoX() {
	  Serial.println("oXoX");
  }

	void onoEoE() {
	  Serial.println("oEoE");
  }

	void oniEiX() {
	  Serial.println("iEiX");
  }

	void onmEmX() {
	  Serial.println("mEmX");
  }

	void oniEmE() {
	  Serial.println("iEmE");
  }

	void onEmX() {
	  Serial.println("iEmX");
  }

	void onoEoX() {
	  Serial.println("oEoX");
  }

	void oniXmE() {
	  Serial.println("iXmE");
  }

	void oniXmX() {
	  Serial.println("iXmX");
  }

	void onmEoE() {
	  Serial.println("mEoE");
  }

	void onmEoX() {
	  Serial.println("mEoX");
  }

	void onmXoX() {
	  Serial.println("mXoX");
  }

	void oniEoE() {
	  Serial.println("iEoE");
  }

	void oniEoX() {
	  Serial.println("iEoX");
  }

	void onmXoE() {
	  Serial.println("mXoE");
  }

	void oniXoE() {
	  Serial.println("iXoE");
  }

	void oniXoX() {
	  Serial.println("iXoX");
  }
};

//We have this function because we need to pass an event into handleEvent(...) in int main, but want the event we pass in to be determined by a function
std::optional<StateMachine::Event> inputToEvent(int I, int O, int M, StateMachine::State State) {
	if (I == 0 && M == 0 && O == 0) {
		return StateMachine::Event::e000;
	}
	if (I == 1 && M == 0 && O == 0) {
		return StateMachine::Event::e100;
	}
	if (I == 0 && M == 1 && O == 0) {
		return StateMachine::Event::e010;
	}
	if (I == 0 && M == 0 && O == 1) {
		return StateMachine::Event::e001;
	}

	if (I == 2 && M == 0 && O == 0) {
		return StateMachine::Event::e200;
	}
	if (I == 1 && M == 1 && O == 0) {
		return StateMachine::Event::e110;
	}
	if (I == 1 && M == 0 && O == 1) {
		return StateMachine::Event::e101;
	}
	if (I == 0 && M == 0 && O == 2) {
		return StateMachine::Event::e002;
	}
	if (I == 0 && M == 1 && O == 1) {
		return StateMachine::Event::e011;
	}
	if (I == 0 && M == 2 && O == 0) {
		return StateMachine::Event::e020;
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
VL53L7CX_ResultsData Nominal;
VL53L7CX_ResultsData Delay1;
VL53L7CX_ResultsData Delay2;
bool firstTime = 1;
int avgNum = 0;

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


// -- MAIN LOOP FUNCTION --
void loop()
{
VL53L7CX_ResultsData Results;

uint8_t NewDataReady = 0;
  uint8_t status;
  uint8_t* ptr;
  uint16_t maxVal = 2500;

	do {
		status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
		status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&Results);
		//print_result(&Results);
  }

  if(firstTime) {
		Nominal = Results;
		Delay1 = Results;
		Delay2 = Results;
		firstTime = !firstTime;
  }
  //print_result(&Nominal);



  // Calculating averages for rows
  float AvgRow[8];
  for (uint8_t i = 0; i < 8; i++) {
		float total = 0;
		for (uint8_t j = 0; j < 8; j++) {
			if (Results.nb_target_detected[i+8*j] > 0) {
				float movingAvg = ( float(Results.distance_mm[i+8*j]) + float(Delay1.distance_mm[i+8*j]) + float(Delay2.distance_mm[i+8*j]) ) / 3.0;
				if (Nominal.nb_target_detected[i+8*j] > 0) {
					
					if((i+8*j > 16 && i+8*j < 23) || (i+8*j > 24 && i+8*j < 31) || (i+8*j > 32 && i+8*j < 39) || (i+8*j > 40 && i+8*j < 47)) {
						total += abs(movingAvg/float(Nominal.distance_mm[i+8*j]) - 1.0) * 0.0;
					}
					else {
						total += abs(movingAvg/float(Nominal.distance_mm[i+8*j]) - 1.0);
					}
				}
				else {
					total += abs(movingAvg/float(maxVal) - 1.0);
				}
			}
			else {
			total += 0;
			}
		}
		// throw each column into an average
		AvgRow[i] = total / 8;

		// Calculate new Moving averages
		Delay2 = Delay1;
		Delay1 = Results;
  }

  /* Calculating weights for Inside, Middle, and Outside
  float AvgInside = (AvgRow[0] + AvgRow[1] + AvgRow[2]);
  float AvgMiddle = (AvgRow[3] + AvgRow[4]);
  float AvgOutside = (AvgRow[5] + AvgRow[6] + AvgRow[7]);
	*/

	float AvgInside = (AvgRow[0] + AvgRow[1]);
  float AvgMiddle = (AvgRow[2] + AvgRow[3] + AvgRow[4] + AvgRow[5]);
  float AvgOutside = (AvgRow[6] + AvgRow[7]);

  // print statements to validate weight
  Serial.println(AvgInside);
  Serial.println(AvgMiddle);
  Serial.println(AvgOutside);

  float thresh1 = .35; // customize LEINweber door, .5-.7
  float thresh2 = 1.1; // customize leinweber 1.1, prev: 1.6
	float thresh_middle1 = 0.5; // leinweber .7-1.1
	float thresh_middle2 = 1.7; // prev 1.9, 1.4 worked w/ braden and I but not brendan, leinweber: 1.15, prev: 1.5
	// 1.8 for brendan
	// ??? for two people side by side
  uint8_t binary_occupancy[3];

  if (AvgInside > thresh1) {
		if (AvgInside > thresh2) {
			binary_occupancy[0] = 2;
		}
		else {
			binary_occupancy[0] = 1;
		}
  }
  else {
		binary_occupancy[0] = 0;
	}
  if(AvgMiddle > thresh_middle1) {
		if (AvgMiddle > thresh_middle2) {
			binary_occupancy[1] = 2;
		}
		else {
			binary_occupancy[1] = 1;
		}
  }
  else {
		binary_occupancy[1] = 0;
	}
  if (AvgOutside > thresh1) {
		if (AvgOutside > thresh2) {
			binary_occupancy[2] = 2;
		}
		else {
			binary_occupancy[2] = 1;
		}
  }
  else {
		binary_occupancy[2] = 0;
	}
	// END BINARY LOGIC
	Serial.println(binary_occupancy[0]);
		Serial.println(binary_occupancy[1]);

	Serial.println(binary_occupancy[2]);



	
  // EVENT HANDLER
  auto event = inputToEvent(binary_occupancy[0], binary_occupancy[2], binary_occupancy[1], sm.getState());

  sm.handleEvent(*event);
  sm.update();

  // needed ? delay
  delay(1);
}
// -- END MAIN LOOP --



// For swapping resolution
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
