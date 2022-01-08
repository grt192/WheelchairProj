#include <Encoder.h>
#include <Servo.h>

// hardware
Encoder flywheelEncoder(2, 3);
const int talon_pwm = 10;
const int stopper_pwm = 9;

const int button = 11;
const int led = 12;
const int slider = 5; // analog

// constants
const long calculateVeloInterval = 500;
const long FLYWHEEL_SPEED_THRESHOLD = 75;

const int STOPPER_CLOSE_POS = 180;
const int STOPPER_OPEN_POS = 150;

const long STOPPER_DROP_TIME = 2000;
const long STOPPER_RETURN_TIME = 1500;

const long MIN_SLIDER = 0;
const long MAX_SLIDER = 1023;

const long MIN_SPEED = 750;
const long MAX_SPEED = 1250;

const long LED_FLASH_MS = 750;

/**
 * A PIDF (PID with Feedforward) controller.
 */
class PIDF {
  public:
    PIDF(double P, double I, double D, double FF)
      : P(P), I(I), D(D), FF(FF) {
        reset();
      }
      
    double scale(double target, double error) {
      totalErr += error;

      double errD = (error - lastErr);

      double ret = (target * FF) + (error * P) + (totalErr * I) + (errD * D);

      lastErr = error;

      return ret;
    }

    void reset() {
      totalErr = 0;
      lastErr = 0;
    }
  private:
    const double P, I, D, FF;
    // F is a primitive method lol, https://forum.arduino.cc/t/what-does-the-f-do-exactly/89384/7
    double totalErr, lastErr;
};

PIDF controller(0, 0, 0, 1.0/1000);

// helpers
Servo talon;
Servo stopper;

long lastLoop = -999999999; // lol idk, is there a Long.MIN_VALUE 
long lastPosition;
bool wasButtonPressed = false;
long stopperStartTime = -999999999;
bool wasFlywheelReady = false;

void setup() {
  Serial.begin(9600);
  
  flywheelEncoder.write(0);
  lastPosition = 0;

  talon.attach(talon_pwm);
  setMotorPower(0);

  stopper.attach(stopper_pwm);
  setStopperPosition(false);

  pinMode(button, INPUT_PULLUP);
  
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

const String fwReadyStr = "FW Ready: ";
const String stprReadyStr = "Stopper ready: ";

void loop() {
  // put your main code here, to run repeatedly:
  long currentTime = millis();

  bool flywheelReady = flywheelLogic(currentTime);
//  flywheelReady = true;
  bool stopperReady = stopperLogic(currentTime);

//  Serial.println(fwReadyStr + flywheelReady);
//  Serial.println(stprReadyStr + stopperReady);

  // utility boolean to control all flashing
  bool ledOn = (currentTime % (2 * LED_FLASH_MS)) < LED_FLASH_MS;

  // Flash the button if ready to fire
  if (flywheelReady && stopperReady) {
    digitalWrite(led, ledOn ? HIGH : LOW);
  } else {
    digitalWrite(led, LOW);
  }

  // flash the builtin LED to show the program is alive
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

const String fwSpeedStr = "FW speed: ";
const String targSpeedStr = "Target speed: ";
const String motorPowStr = "Motor pow: ";
    
bool flywheelLogic(long currentTime) {
  if (currentTime - lastLoop >= calculateVeloInterval) {
    double fwSpeed = calculateFlywheelSpeed(currentTime, lastLoop);
    double targetSpeed = calculateTargetSpeed();

    double scale = constrain(controller.scale(targetSpeed, targetSpeed - fwSpeed), -1, 1);
//    scale = 0.8; // for debugging, TODO remove

    setMotorPower(scale);

    Serial.println("---");

    Serial.println(fwSpeedStr + fwSpeed);
    Serial.println(targSpeedStr + targetSpeed);
    Serial.println(motorPowStr + scale);

    lastLoop = currentTime;

    // store whether the flywheel was ready for future loops where we don't recalc the velocity
    wasFlywheelReady = abs(targetSpeed - fwSpeed) <= FLYWHEEL_SPEED_THRESHOLD;
  }

  // whether the flywheel is up to speed + ready to fire
  return wasFlywheelReady;
}

bool stopperLogic(long currentTime) {
  bool stopperReady = currentTime >= stopperStartTime + STOPPER_DROP_TIME + STOPPER_RETURN_TIME;

  bool pressed = isButtonPressed();
  // if we're ready to fire and the button was pressed
  if (pressed &&  !wasButtonPressed && stopperReady) {
    stopperStartTime = currentTime;
    stopperReady = false;
  }
  wasButtonPressed = pressed;

  if (currentTime <= stopperStartTime + STOPPER_DROP_TIME) {
    setStopperPosition(true);
  } else {
    setStopperPosition(false);
  }

  return stopperReady;
}

double calculateFlywheelSpeed(long currentTime, long lastTime) {
  long newPosition = flywheelEncoder.read();

  // velocity
  double diff = ((double) (newPosition - lastPosition)) / (currentTime - lastTime);

  lastPosition = newPosition;

  return diff;
}

String slideStr = "Slider: ";
double calculateTargetSpeed() {
  int slideInput = analogRead(slider);
  
//  Serial.println(slideStr + slideInput);

  double pos = ((double) (slideInput - MIN_SLIDER)) / (MAX_SLIDER - MIN_SLIDER); // from 0 to 1 the slider position

  double spd = MIN_SPEED + (pos * (MAX_SPEED - MIN_SPEED));
  
  return spd;
}

/**
 * Set the power on the talon, from -1 to 1.
 */
void setMotorPower(double power) {
  int converted = round((power + 1.0) * 90.0);
  talon.write(converted); 
}

void setStopperPosition(bool opened) {
  if (opened) {
    stopper.write(STOPPER_OPEN_POS);
  } else {
    stopper.write(STOPPER_CLOSE_POS);
  }
}

bool isButtonPressed() {
  return (digitalRead(button) == LOW);
}
