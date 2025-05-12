#include "navigation.h"

int sweepFrontServo(int stepSize) {
    int distanceF;
    int numberOfSteps = 0;
    // sweeps the servo through 29 points
    // and takes an ultraound reading at each point
    for (int angle = minUsUltra; angle <= maxUsUltra; angle += stepSize) {
      servoUltrasound.writeMicroseconds(angle);
      vTaskDelay(pdMS_TO_TICKS(350));

      distanceF = getUltrasoundValue(trigPinF);

      scanData[numberOfSteps] = {angle, distanceF};

      // prints out the angle and distance to the GUI
      SerialBT.println(String(angle) + "," + String(distanceF));
      numberOfSteps++;

      if (!RUN) {
        vTaskSuspend(NULL);
      }
    }

    return numberOfSteps;
}

bool analyseForCan(int numberOfScans) {
    bool canLocated = false;
    const int tolerance = 5;
    const int minSequence = 4;
    const int maxMismatches = 1;      // allowed mismatches in the sequence
    int midIdx = -1;
    bool minima = false;

    // analyse data to find can
    for (int start = 0; start < numberOfScans - minSequence; start++) {
      int refDist = scanData[start].distance;
      int length = 1;
      int mismatches = 0;
      for (int i = start + 1; i < numberOfScans; i++) {
        if (abs(scanData[i].distance - refDist) <= tolerance) {
          length++;
        } else if (mismatches < maxMismatches){
          mismatches++;
        } else {
          break;
        }
      }

      // if the length of the sequence is greater than the min sequence length 
      // check if it is a valid sequence
      if (length >= minSequence) {
        SerialBT.println("CAN: a sequence found");
        int beforeIdx = start - 1;
        int afterIdx = start + length;
        int sumAfter = 0;
        int sumBefore = 0;
        int countA = 0;
        int countB = 0;
        int averageAfter = 0;
        int averageBefore = 0;

        float minValValid = 450;
        // find average distance for comparison
        for (int j = start; j < afterIdx; j++) {
          if (scanData[j].distance < minValValid) {
            minValValid = scanData[j].distance;
          }
        }

        // calculate the average value after and before the sequence
        // made up of 3 values or however many there are (start of the array / end of the array)
        for (int k = afterIdx; k < afterIdx + 3; k++) {
          if (k >= numberOfScans) {
            break;
          }
          sumAfter += scanData[k].distance;
          countA++;
        }

        for (int k = beforeIdx; k > beforeIdx - 3; k--) {
          if (k < 0) {
            break;
          }
          sumBefore += scanData[k].distance;
          countB++;
        }

        averageAfter = (countA > 0) ? (sumAfter / countA) : 0;
        averageBefore = (countB > 0) ? (sumBefore / countB) : 0;
        SerialBT.println("CAN: average after: " + String(averageAfter));
        SerialBT.println("CAN: average before: " + String(averageBefore));

        bool beforeHigher = (beforeIdx >= 0 && averageBefore > refDist + (tolerance));
        bool afterHigher = (afterIdx < numberOfScans && averageAfter > refDist + (tolerance));

        // if it is a valid potentual can (minima)
        if ((beforeHigher && afterHigher) && minValValid < 100) {
          canLocated = true;

          // this idicates that it is a minima not just one sided
          // this will give priority to sequences that are minima
          
          float minVal = 450;
          // find average distance for comparison
          for (int j = start; j < afterIdx; j++) {
            if (scanData[j].distance < minVal) {
              minVal = scanData[j].distance;
            }
          }
          // if less than the last found dip then replace (as more likely to be can)
          if (minima == false) {
            currentCanDistance = minVal;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          } else if (minVal < currentCanDistance) {
            currentCanDistance = minVal;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          }
          minima = true;
        // if only one sided and a true minima hasnt been found
        } else if (((beforeHigher || afterHigher) && minima == false) && minValValid < 100) {
          
          canLocated = true;
          float minVal = 450;
          // find average distance for comparison
          for (int j = start; j < afterIdx; j++) {
            if (scanData[j].distance < minVal) {
              minVal = scanData[j].distance;
            }
          }
          // if less than the last found dip then replace (as more likely to be can)
          if (minVal < currentCanDistance) {
            currentCanDistance = minVal;
            midIdx = start + length / 2;
            canAngle = scanData[midIdx].angle;
          }
        }
        start += length;
      }
    }
    return canLocated;
}