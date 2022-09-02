#include <Encoder.h>
#include <Servo.h>

// hardware
Encoder flywheelEncoder(2, 3);
const int talon_pwm = 5;
const int stopper_pwm = 6;

const int button = 11;
const int led = 12;
const int slider = 5; // analog

// constants
const long calculateVeloInterval = 750;
const long FLYWHEEL_SPEED_THRESHOLD = 75;

const int STOPPER_CLOSE_POS = 180;
const int STOPPER_OPEN_POS = 130;

const long STOPPER_DROP_TIME = 750;
const long STOPPER_RETURN_TIME = 300;

const long MIN_SLIDER = 0;
const long MAX_SLIDER = 1023;

const float MIN_SPEED = 5.5;
const float MAX_SPEED = 8.5;

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

PIDF controller(0.075, 0.001, 0.0055, 0.800);

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

  Serial.println("POWER ON");
  
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

  // utility boolean to control all flashing
  bool flashOn = (currentTime % (2 * LED_FLASH_MS)) < LED_FLASH_MS;

  bool flywheelReady = flywheelLogic(currentTime);
  flywheelReady = true;
  bool stopperReady = stopperLogic(currentTime, flywheelReady, flashOn);

//  Serial.println(fwReadyStr + flywheelReady);
//  Serial.println(stprReadyStr + stopperReady);

  // flash the builtin LED to show the program is alive
  digitalWrite(LED_BUILTIN, flashOn ? HIGH : LOW);
}

const String fwSpeedStr = "FW speed: ";
const String targSpeedStr = "Target speed: ";
const String motorPowStr = "Motor pow: ";

const float MAX_SLIDER_POW = 0.9;
const float MIN_SLIDER_POW = 0.3;
    
bool flywheelLogic(long currentTime) {
  if (currentTime - lastLoop >= calculateVeloInterval) {
    double fwSpeed = calculateFlywheelSpeed(currentTime, lastLoop);
//    double targetSpeed = calculateTargetSpeed();

//    double scale = constrain(controller.scale(targetSpeed, targetSpeed - fwSpeed), -1, 1);
//    scale = 0; // for debugging, TODO remove
    double scale = (getSliderPosition() * (MAX_SLIDER_POW - MIN_SLIDER_POW)) + MIN_SLIDER_POW;
      
    setMotorPower(scale);

    Serial.println("---");

    Serial.println(fwSpeedStr + fwSpeed);
//    Serial.println(targSpeedStr + targetSpeed);
    Serial.println(motorPowStr + scale);

    lastLoop = currentTime;

    // store whether the flywheel was ready for future loops where we don't recalc the velocity
//    wasFlywheelReady = abs(targetSpeed - fwSpeed) <= FLYWHEEL_SPEED_THRESHOLD;
     wasFlywheelReady = true;
  }

  // whether the flywheel is up to speed + ready to fire
  return wasFlywheelReady;
}

bool stopperLogic(long currentTime, bool flywheelReady, bool flashOn) {
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

  // Flash the button if ready to fire
  if (flywheelReady && stopperReady) {
    digitalWrite(led, flashOn ? HIGH : LOW);
  } else {
    digitalWrite(led, LOW);
  }

  return stopperReady;
}

double calculateFlywheelSpeed(long currentTime, long lastTime) {
  long newPosition = flywheelEncoder.read();

  Serial.println("pos: " + String(newPosition));
  Serial.println("delta: " + String(newPosition - lastPosition));

  // velocity
  double diff = ((double) (newPosition - lastPosition)) / (currentTime - lastTime);

  lastPosition = newPosition;

  return diff;
}

String slideStr = "Slider: ";
double getSliderPosition() {
  int slideInput = analogRead(slider);
  
//  Serial.println(slideStr + String(slideInput));

  double pos = ((double) (slideInput - MIN_SLIDER)) / (MAX_SLIDER - MIN_SLIDER); // from 0 to 1 the slider position
  return pos;
}

double calculateTargetSpeed() {
  double pos = getSliderPosition();

  // for testing: set motor power to slider position
//  setMotorPower(pos);

  double spd = MIN_SPEED + (pos * (MAX_SPEED - MIN_SPEED));
  
  return spd;
}

/**
 * Set the power on the talon, from -1 to 1.
 */
void setMotorPower(double power) {
//  Serial.println(power);

  double constrained = constrain(power, -1., 1.);
  double writeTime = (constrained * 500.) + 1500.; // scale from 1000 to 2000
  talon.writeMicroseconds(round(writeTime));
}

void setStopperPosition(bool opened) {
  if (opened) {
    stopper.write(STOPPER_OPEN_POS);
  } else {
    stopper.write(STOPPER_CLOSE_POS);
  }
}

bool isButtonPressed() {
  return (digitalRead(button) == 1);
}
