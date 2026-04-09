#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

// Flags for state machine
bool remapping = false;
bool sendingData = false;

// Raw data in Axis vars from Accelerometer
float AccX, AccY, AccZ;
// Relative Axis, so device can be placed in any suitable position
float relAccX, relAccY, relAccZ;
// Filtered values of accelerometer
float filteredAccX = 0, filteredAccY = 0, filteredAccZ = 0;
float accAlpha = 0.1;  // filter weight

// for axis remapping
int axisMapX = 0;
int axisMapY = 1;
int axisMapZ = 2;
int axisSignX = 1;
int axisSignY = 1;
int axisSignZ = 1;

int currentOrientation = -1;
int previousOrientation = -1;

// 0.42, to not update orientation when monitor's pitch is over that value
// approximately can go up to 65° from perpendicularly positioned to user monitor
float orientationThreshold = 0.42;
// Deadzone to not update orientation when device in state between two orientations
float deadZoneRatio = 0.1;

// for determining in remapping function
struct RemappingPoint {
  float x, y, z;
};
RemappingPoint remapping90 = { 0 }, remapping0 = { 0 };

// States of remapping process
enum RemappingState {
  REMAP_NONE,
  REMAP_WAIT_90,
  REMAP_WAIT_0,
  REMAP_DONE
};
RemappingState remapState = REMAP_NONE;

// Remapping accelerometer axis
void remapAccAxis() {
  float val90, val0;
  int dominantAxis90 = getDominantAxis(remapping90, val90);
  int dominantAxis0 = getDominantAxis(remapping0, val0);
  // Axes should not be same
  if (dominantAxis90 == dominantAxis0) {
    Serial.println("[REMAPPING_ERR] Error: Same axis for both orientation!");
    remapState = REMAP_NONE;
    remapping = false;
    return;
  }

  // Remapping axis
  axisMapY = dominantAxis90;
  axisSignY = (val90 > 0) ? 1 : -1;
  axisMapZ = dominantAxis0;
  axisSignZ = (val0 > 0) ? 1 : -1;

  // last axis is X
  int allAxes[3] = { 0, 1, 2 };
  for (int i = 0; i < 3; i++) {
    if (i != axisMapY && i != axisMapZ) {
      axisMapX = i;
      axisSignX = 1;
      break;
    }
  }
  remapState = REMAP_DONE;
}

// get dominant axis to help determining axis remapping
int getDominantAxis(const RemappingPoint& point, float& valueOut) {
  float vals[3] = { point.x, point.y, point.z };
  int maxIndex = 0;
  float maxVal = fabs(vals[0]);

  for (int i = 1; i < 3; i++) {
    if (fabs(vals[i]) > maxVal) {
      maxVal = fabs(vals[i]);
      maxIndex = i;
    }
  }

  valueOut = vals[maxIndex];
  return maxIndex;
}

// Simplified realization of complementary filter
void filterAccelerometer() {
  filteredAccX = (1 - accAlpha) * filteredAccX + accAlpha * relAccX;
  filteredAccY = (1 - accAlpha) * filteredAccY + accAlpha * relAccY;
  filteredAccZ = (1 - accAlpha) * filteredAccZ + accAlpha * relAccZ;
}

// For calculating orientation
struct Orientation {
  float x, y, z;
  int angle;
};


Orientation orientations[] = {
  { 0, 0, 1, 0 },
  { 0, 0, -1, 180 },
  { 0, 1, 0, 90 },
  { 0, -1, 0, 270 },
};



// -------------------------------------------------------
// CHANGED: replaces the MPU-6050 raw I2C read with
//          Adafruit MMA8451 library call.
//          The MMA8451 returns acceleration in m/s².
//          We divide by 9.80665 to convert back to g,
//          matching the ±1g range the orientation math expects.
// -------------------------------------------------------
void acc_signals(void) {
  sensors_event_t event;
  mma.getEvent(&event);

  AccX = event.acceleration.x / 9.80665;
  AccY = event.acceleration.y / 9.80665;
  AccZ = event.acceleration.z / 9.80665;

  float tempAcc[3] = { AccX, AccY, AccZ };
  relAccX = tempAcc[axisMapX] * axisSignX;
  relAccY = tempAcc[axisMapY] * axisSignY;
  relAccZ = tempAcc[axisMapZ] * axisSignZ;
}

// -------------------------------------------------------
// CHANGED: replaces MPU-6050 I2C register init with
//          Adafruit MMA8451 library begin().
//          Range set to 2g to match the sensitivity the
//          original sketch assumed (values near ±1g).
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);

Serial.println("Attempting MMA8451 init...");
if (!mma.begin(0x1C)) {
    Serial.println("[ERROR] MMA8451 not found. Check wiring.");
    while (1);
}
Serial.println("MMA8451 init succeeded.");

  mma.setRange(MMA8451_RANGE_2_G);  // 2g range: best sensitivity for orientation detection

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.println("[READY]");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "REMAP_AXIS") {
      remapping = true;
      remapState = REMAP_WAIT_90;
    } else if (command == "OK" && remapping) {
      if (remapState == REMAP_WAIT_90) {
        remapping90 = { filteredAccX, filteredAccY, filteredAccZ };
        Serial.println("Portrait orientation recorded");
        remapState = REMAP_WAIT_0;
      } else if (remapState == REMAP_WAIT_0) {
        remapping0 = { filteredAccX, filteredAccY, filteredAccZ };
        Serial.println("Landscape orientation recorded");
        remapAccAxis();
      }
    } else if (command == "STOP" && remapping) {
      Serial.println("[REMAPPING_ERR] Cancelled");
      remapState = REMAP_NONE;
      remapping = false;

    } else if (command.startsWith("AUTO_REMAP")) {
      command.remove(0, 11);
      int parts[6] = { 0 };
      int index = 0;
      int lastSlash = -1;

      for (int i = 0; i < command.length(); i++) {
        if (command[i] == '/') {
          String part = command.substring(lastSlash + 1, i);
          parts[index++] = part.toInt();
          lastSlash = i;
          if (index >= 6) break;
        }
      }
      if (index < 6) {
        parts[index] = command.substring(lastSlash + 1).toInt();
      }
      axisMapX = parts[0];
      axisMapY = parts[1];
      axisMapZ = parts[2];
      axisSignX = parts[3];
      axisSignY = parts[4];
      axisSignZ = parts[5];
      Serial.println("[REMAP_LOADED]");
    } else if (command == "SEND_DATA") {
      sendingData = true;
    } else if (command == "STOP" && sendingData) {
      sendingData = false;
      currentOrientation = -1;
      previousOrientation = -1;
      Serial.println("[SENDING_STOPPED]");
    }
  }

  if (remapping) {
    switch (remapState) {
      case REMAP_WAIT_90:
        acc_signals();
        filterAccelerometer();
        Serial.print("[DATA] Acc X [g]= ");
        Serial.print(filteredAccX);
        Serial.print(" Acc Y [g]= ");
        Serial.print(filteredAccY);
        Serial.print(" Acc Z [g]= ");
        Serial.println(filteredAccZ);
        delay(50);
        Serial.println("[Instruction] Turn device to portrait orientation");
        break;
      case REMAP_WAIT_0:
        acc_signals();
        filterAccelerometer();
        Serial.print("[DATA] Acc X [g]= ");
        Serial.print(filteredAccX);
        Serial.print(" Acc Y [g]= ");
        Serial.print(filteredAccY);
        Serial.print(" Acc Z [g]= ");
        Serial.println(filteredAccZ);
        Serial.println("[Instruction] Turn device to landscape orientation");
        delay(50);
        break;
      case REMAP_DONE:
        Serial.print("[REMAPPING_DONE] Remapping Done");
        Serial.print(" /");
        Serial.print(axisMapX);
        Serial.print("/");
        Serial.print(axisMapY);
        Serial.print("/");
        Serial.print(axisMapZ);
        Serial.print("/");
        Serial.print(axisSignX);
        Serial.print("/");
        Serial.print(axisSignY);
        Serial.print("/");
        Serial.println(axisSignZ);
        remapping = false;
        remapState = REMAP_NONE;
        break;
      default:
        break;
    }

  } else if (sendingData) {
    acc_signals();
    filterAccelerometer();
    float bestDot = -1;
    float secondBestDot = -1;
    int bestAngle;

    for (int i = 0; i < 4; i++) {
      float dot = filteredAccX * orientations[i].x + filteredAccY * orientations[i].y + filteredAccZ * orientations[i].z;

      if (dot > bestDot) {
        secondBestDot = bestDot;
        bestDot = dot;
        bestAngle = orientations[i].angle;
      } else if (dot > secondBestDot) {
        secondBestDot = dot;
      }
    }
    if (bestDot > orientationThreshold) {
      if ((bestDot - secondBestDot) < (deadZoneRatio * bestDot)) {
        currentOrientation = previousOrientation;
      } else {
        currentOrientation = bestAngle;
      }
    }
    if (currentOrientation != previousOrientation) {
      previousOrientation = currentOrientation;
      Serial.print("[DATA] ");
      Serial.println(currentOrientation);
    }
  }
  delay(50);
}
