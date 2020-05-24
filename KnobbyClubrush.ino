/*  Knobby Clubrush: robotic plant

     (cleft) 2020 Sensilab - Monash University and UFMG (Federal University of Minas Gerais)
     @ author: Marilia Bergamo
     @ hardware: Marilia Bergamo (UFMG), Elliot Wilson (Sensilab)
     # Knobby Clubrush - Black base {
           Agent A1: TCA Port 0 - Servo Arduino Port D9
           Agent A2: TCA Port 1 - Servo Arduino Port D3
           Agent A3: TCA Port 2 - Servo Arduino Port D5
           Agent A4: TCA Port 3 - Servo Arduino Port D6
           Agent A5: Arduino Port A0 - not connected - cable became too short
           Agent A6: Arduino Port A1
           Agent A7: Arduino Port A2
           Agent A8: Arduino Port A3
           Agent A9: Servo Arduino Port D10
      }
*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

//Class Winder Agents A1, A2, A3 and A4
class Winder {
  private:
    int port;
    byte pin;
    bool alive;
    bool bored;
    bool curius;
    bool interested;
    bool onTop;
    float const_airMass = 1.226; // from theory
    int turnTime = 2000;
    Servo myservo;
    int actualWind;
    int windMemory[10];
    int counter;
  public:
    Winder(int port, byte pin) {
      this->port = port;
      this->pin = pin;
      init();
    }

    void init() {
      pinMode(pin, OUTPUT);
      alive = true;
      bored = false;
      curius = false;
      interested = false;
      onTop = false;
      counter = 0;
      off();
    }

    void die() {
      alive = false;
    }

    void on() {
      myservo.attach(pin);
      // Start turning clockwise
      myservo.write(0);
      // Go on turning for the right duration
      delay(turnTime);
      // Stop turning
      myservo.write(90);
      onTop = true;
      delay(turnTime);
      // Start turning conterclockwise
      myservo.write(180);
      onTop = false;
      delay(turnTime);
      // Stop turning
      myservo.write(90);
      myservo.detach();
    }

    void off() {
      // Stop turning
      myservo.attach(pin);
      myservo.write(90);
      myservo.detach();
    }

    void updateMe() {
      int media = 0;
      for (byte i = 0; i < 10; i = i + 1) {
        //Serial.println(windMemory[i]);
        media += windMemory[i];
      }
      media = media / 10;
      //Serial.println(media);
      if (media > 50) {
        interested = true;
        bored = false;
        curius = false;
        Serial.println("interested");
      } else {
        bored = true;
        interested = false;
        curius = false;
        Serial.println("bored");
      }
      if (bored) {
        int chance = int(random(100));
        if (chance < 10) {
          bored = false;
          interested = false;
          curius = true;
          Serial.println("curius");
        }
      }
    }

    bool onTopAgent() {
      if (onTop) {
        return true;
      } else {
        return false;
      }
    }

    bool boredAgent() {
      if (bored) {
        return true;
      } else {
        return false;
      }
    }

    bool curiusAgent() {
      if (curius) {
        return true;
      } else {
        return false;
      }
    }

    bool interestedAgent() {
      if (interested) {
        return true;
      } else {
        return false;
      }
    }

    void windForce(float xD, float yD, float zD) {
      /*  In nature, Wind-Induced force is calculated by the sum of the wind
          forces acting at each point of the stem and crown, and it is calculated
          using drag equation.
      */
      float z = 30.5;
      int A = 9;
      float u = pow(2, xD);
      float Cd = abs((xD) + (yD) + (zD));
      float wForce = Cd / 2 * const_airMass * A * z * u;
      actualWind = abs(wForce);
      //Serial.println(actualWind);
      if (counter < 10) {
        windMemory[counter] = actualWind;
        counter++;
      } else {
        counter = 0;
      }
      for (byte i = 0; i < 10; i = i + 1) {
        //Serial.println(windMemory[i]);
      }
    }
};

//Class Capturer Agents A5, A6, A7 and A8
class Capturer {
  private:
    int mic;
    bool alive;
    bool bored;
    bool curius;
    bool interested;
    // Variables to find the peak-to-peak amplitude of AUD output
    int sampleTime = 100;
    int micOut;
    //previous VU value0
    int preValue = 0;
    int mySoundMemory[10];
    int counter;
  public:
    Capturer(int mic) {
      this->mic = mic;
      init();
    }

    void init() {
      alive = true;
      bored = false;
      interested = false;
      curius = false;
    }

    void die() {
      alive = false;
    }

    bool boredAgent() {
      if (bored) {
        return true;
      } else {
        return false;
      }
    }

    bool curiusAgent() {
      if (curius) {
        return true;
      } else {
        return false;
      }
    }

    bool interestedAgent() {
      if (interested) {
        return true;
      } else {
        return false;
      }
    }

    void updateMe() {
      if (alive) {
        int micOutput = findPTPAmp(mic);
        //Serial.print("microphone:"); Serial.println(micOutput);
        VUMeter(micOutput, mic);
        int summ = 0;
        for (byte i = 0; i < 10; i = i + 1) {
          //Serial.print(mic);Serial.print(":");Serial.println(mySoundMemory[i]);
          summ += mySoundMemory[i];
        }
        //Serial.print(mic); Serial.print(":"); Serial.println(media);
        if (summ > 200) {
          interested = true;
          bored = false;
          curius = false;
          //Serial.print(mic); Serial.print(":"); Serial.println("interested");
        } else {
          bored = true;
          interested = false;
          curius = false;
          //Serial.print(mic); Serial.print(":"); Serial.println("bored");
        }
        if (bored) {
          int chance = int(random(100));
          if (chance < 10) {
            bored = false;
            interested = false;
            curius = true;
            if (!alive) {
              alive = true;
            }
            //Serial.print(mic); Serial.print(":"); Serial.println("curius");
          }
        }
      }
    }


    // Find the Peak-to-Peak Amplitude Function
    int findPTPAmp(int mic) {
      // Time variables to find the peak-to-peak amplitude
      unsigned long startTime = millis(); // Start of sample window
      unsigned int PTPAmp = 0;

      // Signal variables to find the peak-to-peak amplitude
      unsigned int maxAmp = 0;
      unsigned int minAmp = 1023;

      // Find the max and min of the mic output within the 50 ms timeframe
      while (millis() - startTime < sampleTime)
      {
        micOut = analogRead(mic);
        if ( micOut < 1023) //prevent erroneous readings
        {
          if (micOut > maxAmp)
          {
            maxAmp = micOut; //save only the max reading
          }
          else if (micOut < minAmp)
          {
            minAmp = micOut; //save only the min reading
          }
        }
      }

      PTPAmp = maxAmp - minAmp; // (max amp) - (min amp) = peak-to-peak amplitude
      double micOut_Volts = (PTPAmp * 3.3) / 1024; // Convert ADC into voltage

      //Uncomment this line for help debugging (be sure to also comment out the VUMeter function)
      //Serial.println(PTPAmp);

      //Return the PTP amplitude to use in the soundLevel function.
      // You can also return the micOut_Volts if you prefer to use the voltage level.
      return PTPAmp;
    }

    // Volume Unit Meter function: map the PTP amplitude to a volume unit between 0 and 10.
    int VUMeter(int micAmp, int mic) {

      // Map the mic peak-to-peak amplitude to a volume unit between 0 and 10.
      // Amplitude is used instead of voltage to give a larger (and more accurate) range for the map function.
      // This is just one way to do this -- test out different approaches!
      int fill = map(micAmp, 200, 800, 0, 10);
      //Serial.print("Microphone"); Serial.print(mic); Serial.print(":"); Serial.println(micAmp);
      if (counter < 10) {
        mySoundMemory[counter] = micAmp;
        counter++;
      } else {
        counter = 0;
      }
      //Serial.print("Microphone"); Serial.print(mic); Serial.print(":");
      for (byte i = 0; i < 10; i = i + 1) {
        //Serial.println(mySoundMemory[i]);
      }

      // Only print the volume unit value if it changes from previous value
      while (fill != preValue)
      {
        //Serial.print("Microphone"); Serial.print(mic); Serial.print(":"); Serial.println(fill);
        preValue = fill;
      }
    }
};

//Class Turner Agent A9
class Turner {
  private:
    byte pin;
    bool alive;
    bool bored;
    bool curius;
    bool interested;
    int turnTime = 2000;
    Servo myservo;
  public:
    Turner(byte pin) {
      this->pin = pin;
      init();
    }

    void init() {
      pinMode(pin, OUTPUT);
      off();
    }

    void die() {
      alive = false;
    }

    bool boredAgent() {
      if (bored) {
        return true;
      } else {
        return false;
      }
    }

    bool curiusAgent() {
      if (curius) {
        return true;
      } else {
        return false;
      }
    }

    bool interestedAgent() {
      if (interested) {
        return true;
      } else {
        return false;
      }
    }

    void on() {
      myservo.attach(pin);
      // Start turning clockwise
      myservo.write(0);
      // Go on turning for the right duration
      delay(turnTime);
      // Stop turning
      myservo.write(90);
      delay(turnTime);
      myservo.detach();
    }
    void off() {
      // Stop turning
      myservo.attach(pin);
      myservo.write(90);
      myservo.detach();
    }
};


// from Wire library, so we can do bus scanning
#define TCAADDR 0x70
#define MPUADDR 0x68

#define A1Port 0
#define A2Port 1
#define A3Port 2
#define A4Port 3
#define Servo_0_PIN 9
#define Servo_1_PIN 3
#define Servo_2_PIN 5
#define Servo_3_PIN 6
#define Servo_4_PIN 10
#define mic01 A0 // Disconected 
#define mic02 A1
#define mic03 A2
#define mic04 A3

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
Adafruit_MPU6050 mpu3;
Adafruit_MPU6050 mpu4;
Winder myAgent1(A1Port, Servo_0_PIN);
Winder myAgent2(A2Port, Servo_1_PIN);
Winder myAgent3(A3Port, Servo_2_PIN);
Winder myAgent4(A4Port, Servo_3_PIN);
Capturer myAgent5(mic01);
Capturer myAgent6(mic02);
Capturer myAgent7(mic03);
Capturer myAgent8(mic04);
Turner myAgent9(Servo_4_PIN);

bool windersBored;
bool winderInterested;
bool winderCurius;
bool capturesBored;
bool captureInterested;
bool captureCurius;

//=========================
// FOUND IMU SENSORS
//=========================
bool foundIMU[8] = {false};
unsigned long timer = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void describeTCAPorts() {
  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);
    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
        foundIMU[t] = true;
      }
    }
  }
  Serial.println("\ndone");
}

void setup(void)
{
  while (!Serial);
  delay(1000);

  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");
  //describeTCAPorts();
  //runAndStopAllServos();

  pinMode(LED_BUILTIN, OUTPUT);
  /*Capturer agents*/
  myAgent5.die(); // Because is disconected on the hardware

  /* Initialise the 1st sensor */
  tcaselect(0);
  // Try to initialize!
  if (!mpu1.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("#0 -> MPU6050 Found!");

  /* Initialise the 1st sensor */
  tcaselect(1);
  // Try to initialize!
  if (!mpu2.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("#1 -> MPU6050 Found!");

  /* Initialise the 1st sensor */
  tcaselect(2);
  // Try to initialize!
  if (!mpu3.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("#2 -> MPU6050 Found!");

  /* Initialise the 1st sensor */
  tcaselect(3);
  // Try to initialize!
  if (!mpu4.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("#3 -> MPU6050 Found!");
}
void loop(void)
{
  /* Get new sensor events with the readings */
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);
  myAgent1.windForce(g1.gyro.x, g1.gyro.y, g1.gyro.z);

  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);
  myAgent2.windForce(g2.gyro.x, g2.gyro.y, g2.gyro.z);

  sensors_event_t a3, g3, temp3;
  mpu3.getEvent(&a3, &g3, &temp3);
  myAgent3.windForce(g3.gyro.x, g3.gyro.y, g3.gyro.z);

  sensors_event_t a4, g4, temp4;
  mpu4.getEvent(&a4, &g4, &temp4);
  myAgent4.windForce(g4.gyro.x, g4.gyro.y, g4.gyro.z);

  delay(500);

  myAgent1.updateMe();
  myAgent2.updateMe();
  myAgent3.updateMe();
  myAgent4.updateMe();
  myAgent5.updateMe();
  myAgent6.updateMe();
  myAgent7.updateMe();
  myAgent8.updateMe();
  delay(500);

  //enviroment checking
  howIsEnvironment();
  if (windersBored) {
    runAndStopAllServos();
  }
  if (winderCurius) {
    randonServo();
  }
  if (winderInterested) {
    stopAllServos();
  }
  if (capturesBored) {
    runAndStopAllServos();
  }
  if (captureCurius) {
    randonServo();
  }
  if (captureInterested) {
    stopAllServos();
  }

}

void howIsEnvironment() {
  if ((myAgent1.boredAgent() && myAgent2.boredAgent() && myAgent3.boredAgent() && myAgent4.boredAgent())) {
    windersBored = true;
  } else {
    windersBored = false;
  }
  if (myAgent1.interestedAgent() || myAgent2.interestedAgent() || myAgent3.interestedAgent() || myAgent4.interestedAgent()) {
    winderInterested = true;
  } else {
    winderInterested = false;
  }
  if (myAgent1.curiusAgent() || myAgent2.curiusAgent() || myAgent3.curiusAgent() || myAgent4.curiusAgent()) {
    winderCurius = true;
  } else {
    winderCurius = false;
  }
  if ((myAgent5.boredAgent() && myAgent6.boredAgent() && myAgent7.boredAgent() && myAgent8.boredAgent())) {
    capturesBored = true;
  } else {
    capturesBored = false;
  }
  if (myAgent5.interestedAgent() || myAgent6.interestedAgent() || myAgent7.interestedAgent() || myAgent8.interestedAgent()) {
    captureInterested = true;
  } else {
    captureInterested = false;
  }
  if (myAgent5.curiusAgent() || myAgent6.curiusAgent() || myAgent7.curiusAgent() || myAgent8.curiusAgent()) {
    captureCurius = true;
  } else {
    captureCurius = false;
  }
  if ((myAgent1.onTopAgent() && myAgent2.onTopAgent() && myAgent3.onTopAgent() && myAgent4.onTopAgent())) {
    randomDeth();
  }
}

void runAndStopAllServos() {
  myAgent1.on();
  myAgent2.on();
  myAgent3.on();
  myAgent4.on();
  delay(500);
  myAgent1.off();
  myAgent2.off();
  myAgent3.off();
  myAgent4.off();
}

void stopAllServos() {
  myAgent1.off();
  myAgent2.off();
  myAgent3.off();
  myAgent4.off();
}

void randomDeth() {
  int temp = int(random(4));
  switch (temp) {
    case 0:
      myAgent1.die();
      break;
    case 1:
      myAgent1.die();
      break;
    case 2:
      myAgent1.die();
      break;
    case 3:
      myAgent1.die();
      break;
  }
}

void randonServo() {
  int temp = int(random(5));
  switch (temp) {
    case 0:
      myAgent1.on();
      delay(500);
      myAgent1.off();
      break;
    case 1:
      myAgent2.on();
      delay(500);
      myAgent2.off();
      break;
    case 2:
      myAgent3.on();
      delay(500);
      myAgent3.off();
      break;
    case 3:
      myAgent4.on();
      delay(500);
      myAgent4.off();
      break;
    case 4:
      myAgent9.on();
      delay(500);
      myAgent9.off();
    default:
      // statements
      break;
  }

}
