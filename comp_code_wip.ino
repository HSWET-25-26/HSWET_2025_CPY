#include <Wire.h>
#include <Adafruit_INA260.h>
#include <Servo.h>
#include <LowPower.h>

// --------- INITIALIZING ----------
Adafruit_INA260 ina260 = Adafruit_INA260();
Servo pitchActuator;

const int ACTUATOR_PIN = 5;
const int LOAD_PWM_PIN = 9;       // Optoisolated PWM out to Load Box

enum SystemState {
  STARTUP,
  MPPT_MODE,
  PITCH_REGULATION,
  ESD_STATE,
  RESTART_WAIT
};
SystemState currentState = STARTUP;

const float RATED_BUS_MV = 12000.0;     // Pitch when bus hits 12V (adjust to bench tests)
const float BUS_HYSTERESIS_MV = 1000.0; // Return to MPPT when bus drops below 11V
const int FEATHER_POS = 1500;           // full feather actuator position  ---> these numbers will need to be changed
const int RUN_POS = 1100;               // 0perational actuator position ---> these numbers will need to be changed
const int CHARGE_DELAY_MS = 5000;       // 5 seconds to guarantee caps are full

// ----- CONTROL VARS ------
unsigned long startupStartTime = 0;

// MPPT (P&O) Variables
unsigned long lastMpptTime = 0;
const int MPPT_INTERVAL = 50;           // Execute P&O every 50ms
float lastPower = 0.0;
int vRefPWM = 127;                      // Start at 50% duty cycle
int pwmStepDirection = 1;               // 1 for increasing load, -1 for decreasing load

// PI Controller Variables
unsigned long lastPiTime = 0;
const int PI_INTERVAL = 50;             // Execute PI loop every 50ms
float Kp = 0.05;                        // START SMALL! (Tuned for mV scale)
float Ki = 0.001;                       // START SMALL! (Tuned for mV scale)
float integralError = 0.0;
int currentPitchPWM = RUN_POS;

// ------- SETUP ------------
void setup() {
  Serial.begin(115200);
  pinMode(LOAD_PWM_PIN, OUTPUT);
  
  pitchActuator.attach(ACTUATOR_PIN);
  pitchActuator.writeMicroseconds(RUN_POS); 
  
  if (!ina260.begin(0x40)) {
    Serial.println("Couldn't find INA260 chip");
    while (1); 
  }
}

// ------- MAIN LOOP -------
void loop() {
  float currentBus_mV = ina260.readBusVoltage(); 
  
  // Safety system global check
  if (checkGridLoss() && currentState != ESD_STATE && currentState != RESTART_WAIT) {
    currentState = ESD_STATE;
  }

  // FSM
  switch (currentState) {
    
    case STARTUP:
      analogWrite(LOAD_PWM_PIN, 0); // Active load off to allow spool-up
      
      // Monitor turbine voltage as proxy for supercapacitor charge
      if (currentBus_mV > 2000.0) { // If generating > 2V, start timer
        if (startupStartTime == 0) startupStartTime = millis();
        
        if (millis() - startupStartTime >= CHARGE_DELAY_MS) {
          currentState = MPPT_MODE;
        }
      } else {
        startupStartTime = 0; // Reset timer if wind drops during startup
      }
      break;

    case MPPT_MODE:
      // Transition Check: Over-voltage (Proxy for Over-speed)
      if (currentBus_mV > RATED_BUS_MV) {
        currentState = PITCH_REGULATION;
        break;
      }

      // Execute P&O Algorithm
      if (millis() - lastMpptTime >= MPPT_INTERVAL) {
        executeMPPT();
        lastMpptTime = millis();
      }
      break;

    case PITCH_REGULATION:
      // Return to MPPT when voltage drops safely
      if (currentBus_mV < (RATED_BUS_MV - BUS_HYSTERESIS_MV)) {
        currentState = MPPT_MODE;
        pitchActuator.writeMicroseconds(RUN_POS); 
        currentPitchPWM = RUN_POS;
        integralError = 0; // Reset integral windup
        break;
      }

      // Execute PI Loop
      if (millis() - lastPiTime >= PI_INTERVAL) {
        executePitchPI(currentBus_mV); 
        lastPiTime = millis();
      }
      
      // Maintain MPPT load optimization during pitching
      if (millis() - lastMpptTime >= MPPT_INTERVAL) {
        executeMPPT();
        lastMpptTime = millis();
      }
      break;

    case ESD_STATE:
      triggerEmergencyShutdown();
      currentState = RESTART_WAIT;
      break;

    case RESTART_WAIT:
      // Wait for grid restoration or manual reset
      if (ina260.readBusVoltage() > 1000) { 
        currentState = STARTUP;
        pitchActuator.attach(ACTUATOR_PIN);
        pitchActuator.writeMicroseconds(RUN_POS);
      }
      break;
  }
}

// ------ HELPERS ----------

void executeMPPT() {
  float currentPower = ina260.readPower(); 
  
  if (currentPower < lastPower) {
    pwmStepDirection = -pwmStepDirection; // Reverse search direction
  }
  
  vRefPWM += pwmStepDirection;
  vRefPWM = constrain(vRefPWM, 0, 255); 
  analogWrite(LOAD_PWM_PIN, vRefPWM);
  
  lastPower = currentPower;
}

void executePitchPI(float currentBus_mV) {
  float error = currentBus_mV - RATED_BUS_MV; 
  integralError += error * (PI_INTERVAL / 1000.0);
  
  float output = (Kp * error) + (Ki * integralError);
  
  currentPitchPWM = RUN_POS + (int)output;
  currentPitchPWM = constrain(currentPitchPWM, RUN_POS, FEATHER_POS);
  
  pitchActuator.writeMicroseconds(currentPitchPWM);
}

bool checkGridLoss() {
  float current_mA = ina260.readCurrent();
  float bus_mV = ina260.readBusVoltage();
  
  // If PCC drops, current falls to ~0 while voltage remains or spikes
  if (current_mA < 5.0 && bus_mV > 500.0) { 
    return true;
  }
  return false;
}

void triggerEmergencyShutdown() {
  // Maximize Load Braking
  analogWrite(LOAD_PWM_PIN, 255); 
  
  // Feather Blades
  // 1600ms / 20ms = 80 iterations
  for (int t = 0; t < 80; t++) {
    pitchActuator.writeMicroseconds(FEATHER_POS);
    LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_ON, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  }

  // Detach to kill servo holding current
  pitchActuator.detach();
  
  // Enter Deep Sleep (Only wake via hardware reset or loop continuation)
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
}
