/*
Date: 07.11.2019
*/

#include "Arduino.h"
#include <SoftwareSerial.h>

/* 
Internal holes in the main polygon,
if you want to add a new hole, make sure you add
another define for the added hole.
*/
#define EXTERNAL_POLYGON_SIZE 4 // Polygon's size.
#define INTERNAL_POLYGON_1_SIZE 0
#define INTERNAL_POLYGON_2_SIZE 0
#define SIZE (EXTERNAL_POLYGON_SIZE + INTERNAL_POLYGON_1_SIZE + INTERNAL_POLYGON_2_SIZE)// The total size of the ribs (outer + inner) in the polygon.
#define LAST_POINTS_SIZE 3 // Size of last inputs from GPS device.
#define NUM_OF_POLYGONS 1 // If we have polygon withot holes, put 1. else: 1+numOfHoles.

#define DISTANCE1 2 // First warning's distance from the limit.
#define DISTANCE2 1 // Second warning's distance from the limit.

#define SPEED 30 // Constant speed (in km/h).
#define TIME 500 // Sampeling time(in msec).

#define WAITING_TO_CONNECT 5000 // Time to wait to connection in milliseconds.
#define CALC_DIS_SPEED_TIME ((SPEED) * (TIME) * (0.79585) * (69.172) * (1.6) * (1000)) // Distance that the object over.

typedef struct Point {
  double xx; // Latitude.
  double yy; // Longitude.
  byte hh; // Hours.
  byte mm; // Minutes.
  byte ss; // Seconds.
  char polygonIndex; // 0 - Main polygon, 1 - The first hole, 2 - Second hole..
  byte valid; // Flag that represent us if is a valid input or not.
} Point;

// Globals:

SoftwareSerial SoftSerial(2, 3);
int lenArr[NUM_OF_POLYGONS]; // Array that represents the sizes of the circles.
Point g_Point; // Global point to input, updates every sample of GPS device.
Point pointArr[SIZE]; // Array of points of the polygones.
int quality; // Quality of the input point.
int invalidPointsCounter; // Count the invalid points.
int lastPointsIndex; // Index in array of the last points.
byte isInside; // Flag that remembers us if we were inside the polygon on the last sampling.
double shortestDistanceLastValidPoint; // Short distance from the last valid point.
Point LastPointsArr[LAST_POINTS_SIZE];// We save the last valid points here.
byte lastPointArrValid; // Flag if the last points array is empty or not.
char buf[128];
byte checksum;
Point closestPoint; // The closest point from the current location;
int unreliableCounter;// Number of points with illogical location.

// Declaration functions:

void setArrSizes(int* i_Arr); // Function to set the size of the whole polygons.
Point CreatePoint(double xx, double yy, char index, int hh, int mm, int ss); // Create Point from input.
void printlnAndCheck(char* i_Str);// Print message to serial and enter new line.
void printAndCheck(char* i_Str);// Print message to serial.
Point inputPoint(); // Return the point after get the string from GPS.
byte InPolygon(Point* i_PointArr, int* i_LenArr, int len, Point i_Point); // Check if the Point inside the Polygon, 1 - ok, 0 - not ok.
void SendPoint(Point i_Point); // Send the input Point to the server(extention to the future).
double ShortestFromLimits(Point i_Point); // Shortest distance between point and polygon(Internal and external).
void PrintWarning(); // Prints appropriate warning message.
void getInput(char* buf); // Get the GGA message from the GPS and return it.
byte checkChecksum(char* i_Str, byte i_Checksum); // Check if the checksum is ok, 1 - ok, 0 - not ok.
double stringToDouble(char* str); // Convert from string to double.
double convertorLatiLongi(double num); // Convert to Lati or Longi.
byte InInternalPolygon(Point* i_PointArr, int i_Len, Point i_Point); // Check if point inside specific polygon, 1 - ok, 0 - not ok.
double shortestDistancePointFromPolygon(Point* i_PointsArr, int i_Len, Point i_Point); // Shortest distance between point and specific polygon.
void clearBuf(char* buf); // Clear the array, put Null in the organs.
double shortestDistanceFromPointToLine(Point i_LeftPoint, Point i_RightPoint, Point i_CenterPoint);
double disPointToPointNotUpdate(Point aa, Point bb); // Calculate the distance between 2 points, return in Megameter, without update the global variable.
double disPointToPoint(Point aa, Point bb); // Calculate the distance between 2 points, return in Megameter, and update the global variable.
double disLineToPoint(Point left, Point right, Point point); // Calculate the distance between line and point, return in Megameter, and update the global variable.
void printPoint(Point point); // Print point.
byte checkQuality(int i_Quality); // Check if the quality is ok, 1 - ok, 0 - not ok.
byte pointIsGood(int i_Quality, char* i_Str, byte i_Checksum); // Check if the point is ok by check the quality and checksum, 1 - ok, 0 - not ok.

void setup() {
  SoftSerial.begin(9600);
  Serial.begin(9600);

  setArrSizes(lenArr);

  /*
  External polygon:
  
  Write here the external points of the polygon.
  For Example: 
  pointArr[0] = CreatePoint(32.067236, 34.826889, 0, 0, 0, 0);
  pointArr[1] = CreatePoint(32.072249, 34.826663, 0, 0, 0, 0);
  pointArr[2] = CreatePoint(32.071869, 34.835604, 0, 0, 0, 0);
  Is a triangle with the points ((32.067236, 34.826889) , (32.072249, 34.826663) , (32.071869, 34.835604)).

  */
  pointArr[0] = CreatePoint(32.067236, 34.826889, 0, 0, 0, 0);
  pointArr[1] = CreatePoint(32.072249, 34.826663, 0, 0, 0, 0);
  pointArr[2] = CreatePoint(32.071869, 34.835604, 0, 0, 0, 0);
  pointArr[3] = CreatePoint(32.066713, 34.835140, 0, 0, 0, 0);
  /*
  Internal polygon 1:
  
  Write here the points of internal polygons(holes),
  the 3rd index is the counting index of holes.
  Example:
  If you want to add the 4th hole, add point:
  pointArr[4] = CreatePoint(LATI, LONGI, 4, 0, 0, 0);
  this is the first point of the 4th hole of the main polygon.
  */
  /*
  pointArr[4] = CreatePoint(32.068932, 34.829724, 1, 0, 0, 0);
  pointArr[5] = CreatePoint(32.070522, 34.829722, 1, 0, 0, 0);
  pointArr[6] = CreatePoint(32.070445, 34.832354, 1, 0, 0, 0);
  pointArr[7] = CreatePoint(32.068860, 34.832239, 1, 0, 0, 0);
  //pointArr[9] = CreatePoint(32.033056, 34.758296, 1, 0, 0, 0);

  // Internal polygon 2:
  pointArr[10] = CreatePoint(32.029772, 34.754105, 2, 0, 0, 0);
  pointArr[11] = CreatePoint(32.031148, 34.756608, 2, 0, 0, 0);
  pointArr[12] = CreatePoint(32.030346, 34.759550, 2, 0, 0, 0);
  pointArr[13] = CreatePoint(32.027909, 34.759144, 2, 0, 0, 0);
  pointArr[14] = CreatePoint(32.033056, 34.758296, 2, 0, 0, 0);
  */
  invalidPointsCounter = 0;
  lastPointsIndex = 0;
  quality = 0;
  lastPointArrValid = 0;
  unreliableCounter = 1;
  printlnAndCheck("Begin.");
}

void loop() {
  g_Point = inputPoint(); // Read new point from GPS device.

  if (g_Point.valid == 1) { // If we have got a valid point(good checksum and GPSfix).
    if ((lastPointArrValid == 1) && (disPointToPointNotUpdate(LastPointsArr[lastPointsIndex], g_Point) > (2 * CALC_DIS_SPEED_TIME * unreliableCounter))) { 
      // We check if we have got enough points to start and the point that we got now is in unlogical location.
      // If so, we print the unlogical point and continue to the next point.
      printlnAndCheck("The last point is incorrect, the details of this point are:");
      printPoint(g_Point);
      unreliableCounter++;
    }
    else {
      // The distance is good or we have to wait for enough points to start.
      invalidPointsCounter = 0;
      unreliableCounter = 1;
      LastPointsArr[lastPointsIndex] = g_Point;
      lastPointsIndex++;
      if ((lastPointArrValid == 0) && (lastPointsIndex == LAST_POINTS_SIZE)) {
        lastPointArrValid = 1;
      }
      lastPointsIndex = lastPointsIndex % LAST_POINTS_SIZE;
      SendPoint(g_Point);
      if (lastPointArrValid == 1) {
        isInside = InPolygon(pointArr, lenArr, NUM_OF_POLYGONS, g_Point);
        if (isInside != 1) {
          printlnAndCheck("You out, Get back!");
        }
        else {
          shortestDistanceLastValidPoint = ShortestFromLimits(g_Point);
          PrintWarning();
        }
      }
      else {
        // We have not got enough points to start.
        printAndCheck("Waiting for ");
        Serial.print(LAST_POINTS_SIZE);
        printAndCheck(" points, ");
        Serial.print((LAST_POINTS_SIZE - lastPointsIndex));
        printlnAndCheck(" left.");
      }
    }
  }
  else { // point not good(not valid, GPSfix is not good).
    if (lastPointArrValid != 0) {
      if (invalidPointsCounter == 0) {
        shortestDistanceLastValidPoint = ShortestFromLimits(g_Point);
      }
      invalidPointsCounter++;
      if (isInside != 1) {
        printlnAndCheck("You out, Get back!");
      }
      else {
        PrintWarning();
      }
    }
    else {
      printlnAndCheck("Dont have a valid point yet.");
    }
  }
  quality = 0;
  checksum = 0;
  printlnAndCheck("");
}

// Functions:

void setArrSizes(int* i_Arr) {
  i_Arr[0] = EXTERNAL_POLYGON_SIZE;
  i_Arr[1] = INTERNAL_POLYGON_1_SIZE;
  i_Arr[2] = INTERNAL_POLYGON_2_SIZE;
}

void printlnAndCheck(char* str) {
  Serial.println(str);
}

void printAndCheck(char* str) {
  Serial.print(str);
}

Point inputPoint() {
  Point point;
  point.valid = 0;
  int countComma;
  int ii = 0; // Index of buf;
  int len; // Index of input array;
  char input[20];
  checksum = 0;
  getInput(buf);
  for (countComma = 6, ii = 0; countComma > 0; ii++) { // Jump to thw quality;
    if (buf[ii] == ',') {
      countComma--;
    }
  }
  quality = buf[ii] - '0';
  for (; buf[ii] != '*'; ii++) {

  }
  ii++;
  checksum = ((('0' <= buf[ii]) && (buf[ii] <= '9')) ? (buf[ii] - '0') : (buf[ii] - 'A' + 10)) * 16;
  ii++;
  checksum += ((('0' <= buf[ii]) && (buf[ii] <= '9')) ? (buf[ii] - '0') : (buf[ii] - 'A' + 10));

  if (checkQuality(quality) != 1) {
    printlnAndCheck("Quality not good.");
  }
  else if (checkChecksum(buf, checksum) != 1) {
    printlnAndCheck("Checksum not good.");
  }
  else { // Quality good;
    point.valid = 1;
    for (ii = 7, len = 0; (buf[ii] != '.') && (buf[ii] != ',') && (len < (20 - 1)); ii++, len++) {
      input[len] = buf[ii];
    }
    input[len] = '\0';
    point.hh = (input[0] - '0') * 10 + (input[1] - '0');
    point.mm = (input[2] - '0') * 10 + (input[3] - '0');
    point.ss = (input[4] - '0') * 10 + (input[5] - '0');

    for (; buf[ii] != ','; ii++) { // Jump to the begin of the next part - yy;

    }
    for (len = 0, ii++; (buf[ii] != ',') && (len < (20 - 1)); ii++, len++) {
      input[len] = buf[ii];
    }
    input[len] = '\0';
    point.xx = convertorLatiLongi(stringToDouble(input));
    for (; buf[ii] != ','; ii++) { // Jump to the begin of the next part - direction;

    }
    if (buf[++ii] == 'S') { // If its South so its minus;
      point.xx *= (-1);
    }
    for (; buf[ii] != ','; ii++) { // Jump to the begin of the next part - xx;

    }
    for (len = 0, ii++; (buf[ii] != ',') && (len < (20 - 1)); ii++, len++) {
      input[len] = buf[ii];
    }
    input[len] = '\0';
    point.yy = convertorLatiLongi(stringToDouble(input));
    for (; buf[ii] != ','; ii++) { // Jump to the begin of the next part - direction;

    }
    if (buf[++ii] == 'W') { // If its West so its minus;
      point.yy *= (-1);
    }
  }
  return point;
}

byte InPolygon(Point* i_PointArr, int* i_LenArr, int len, Point i_Point) {
  int nn;
  byte cc = InInternalPolygon(i_PointArr, i_LenArr[0], i_Point);
  if (cc == 1) {
    nn = i_LenArr[0];
    for (int ii = 1; (ii < len); ii++) {
      if (InInternalPolygon((i_PointArr + nn), i_LenArr[ii], i_Point) == 1) {
        printlnAndCheck("Point is in internal hole.");
        cc = 0;
        break;
      }
      nn += i_LenArr[ii];
    }
  }
  return cc;
}

void SendPoint(Point i_Point) {

}

double ShortestFromLimits(Point i_Point) {
  int nn = lenArr[0];
  Point point;
  double temp, shortest = shortestDistancePointFromPolygon(pointArr, lenArr[0], i_Point);
  point = closestPoint;
  for (int ii = 1; ii < NUM_OF_POLYGONS; ii++) {
    temp = shortestDistancePointFromPolygon(pointArr + nn, lenArr[ii], i_Point);
    if (temp < shortest) {
      shortest = temp;
      point = closestPoint;
    }
    nn += lenArr[ii];
  }
  closestPoint = point;
  return shortest;
}

void PrintWarning() {
  printAndCheck("Time: ");
  Serial.print(((g_Point.hh < 10) ? ("0") : ("")));
  Serial.print(g_Point.hh);
  printAndCheck(":");
  Serial.print(((g_Point.mm < 10) ? ("0") : ("")));
  Serial.print(g_Point.mm);
  printAndCheck(":");
  Serial.print(((g_Point.ss < 10) ? ("0") : ("")));
  Serial.print(g_Point.ss);
  printlnAndCheck("");
  printlnAndCheck("GGA message:");
  printAndCheck(buf);
  printlnAndCheck("Current point:");
  printPoint(g_Point);
  if ((shortestDistanceLastValidPoint - (invalidPointsCounter * CALC_DIS_SPEED_TIME)) < DISTANCE2) {
    // OVER DISTANCE2
    printlnAndCheck("Distance 2");
  }
  else if ((shortestDistanceLastValidPoint - (invalidPointsCounter * CALC_DIS_SPEED_TIME)) < DISTANCE1) {
    // OVER DISTANCE1
    printlnAndCheck("Distance 1");
  }
  else {
    // OK
    printlnAndCheck("The closest point: ");
    printPoint(closestPoint);
    Serial.println(shortestDistanceLastValidPoint, 6);
    printlnAndCheck("OK!");
  }
}

void getInput(char* buf) {
  int ii;
  byte flag = 0;
  unsigned long startMillis = millis();
  unsigned long currentMillis;
  do {
    ii = 0;
    clearBuf(buf);
    while (1 == 1) {
      while (SoftSerial.available()) {
        flag = 1;
        buf[ii] = (char)SoftSerial.read(); // writing data into array
        if (buf[ii] == 10) {
          break;
        }
        else {
          ii++;
        }
      }
      if (flag != 1) {
        currentMillis = millis();
        if (startMillis > currentMillis) {
          startMillis = currentMillis;
        }
        else if ((currentMillis - startMillis) > WAITING_TO_CONNECT) {
          printlnAndCheck("No connection!");
          startMillis = currentMillis;
          break;
        }
      }
      else {
        flag = 0;
        if (buf[ii] == 10) {
          break;
        }
      }
    }
  } while ((buf[3] != 'G') || (buf[4] != 'G') || (buf[5] != 'A'));
}

byte checkChecksum(char* i_Str, byte i_Checksum) {
  int ii;
  byte xorr;
  for (ii = 1, xorr = 0; i_Str[ii] != '*'; ii++) {
    xorr ^= i_Str[ii];
  }
  return ((xorr == i_Checksum) ? (1) : (0));
}

double stringToDouble(char* str) {
  double returnIt = 0;
  double per = 0.1;
  for (int ii = 0; (str[ii] != '.') && (str[ii] != '\0'); ii++) {
    per *= 10;
  }
  for (int ii = 0; str[ii] != '\0'; ii++) {
    if (str[ii] != '.') {
      returnIt += (str[ii] - '0') * per;
      per /= 10;
    }
  }
  return returnIt;
}

double convertorLatiLongi(double num) {
  int aa = num / 100;
  double bb = (num - (aa * 100)) / 60;
  return (aa + bb);
}

byte InInternalPolygon(Point* i_PointArr, int i_Len, Point i_Point) {
  byte cc = 0;
  for (int ii = 0, jj = (i_Len - 1); ii < i_Len; jj = ii++) {
    if (((i_PointArr[ii].yy > i_Point.yy) != (i_PointArr[jj].yy > i_Point.yy)) &&
      (i_Point.xx < (((i_PointArr[jj].xx - i_PointArr[ii].xx) * (i_Point.yy - i_PointArr[ii].yy) / (i_PointArr[jj].yy - i_PointArr[ii].yy)) + i_PointArr[ii].xx))) {
      cc = !(cc);
    }
  }
  return cc;
}

double shortestDistancePointFromPolygon(Point* i_PointsArr, int i_Len, Point i_Point) {
  Point point;
  double temp, shortest = shortestDistanceFromPointToLine(i_PointsArr[i_Len - 1], i_PointsArr[0], i_Point);
  point = closestPoint;
  for (int ii = 1; ii < i_Len; ii++) {
    temp = shortestDistanceFromPointToLine(i_PointsArr[ii], i_PointsArr[ii - 1], i_Point);
    if (temp < shortest) {
      point = closestPoint;
      shortest = temp;
    }
  }
  closestPoint = point;
  return shortest;
}

void clearBuf(char* buf) {
  for (int ii = 0; ii < 128; ii++) {
    buf[ii] = NULL;
  }
}

double shortestDistanceFromPointToLine(Point i_LeftPoint, Point i_RightPoint, Point i_Point) {
  double dis = 1;
  float tt = -((((i_LeftPoint.xx - i_Point.xx) * (i_RightPoint.xx - i_LeftPoint.xx)) + ((i_LeftPoint.yy - i_Point.yy) * (i_RightPoint.yy - i_LeftPoint.yy))) /
    (((i_RightPoint.xx - i_LeftPoint.xx) * (i_RightPoint.xx - i_LeftPoint.xx)) + ((i_RightPoint.yy - i_LeftPoint.yy) * (i_RightPoint.yy - i_LeftPoint.yy))));
  dis = ((tt <= 0) ? (disPointToPoint(i_LeftPoint, i_Point)) : ((tt >= 1) ? (disPointToPoint(i_RightPoint, i_Point)) : (disLineToPoint(i_LeftPoint, i_RightPoint, i_Point))));
  return dis * 1000;
}

double disPointToPoint(Point point1, Point point2) { // Point1 = point on polygon, Point2 = current location.
  double dd = disPointToPointNotUpdate(point1, point2);
  closestPoint = point1;
  return dd; // Kilometers;
}

double disPointToPointNotUpdate(Point point1, Point point2) { // Point1 = point on polygon, Point2 = current location.
  double RR = 6378.137; // Radius of earth in KM
  double dLat = point2.xx * PI / 180 - point1.xx * PI / 180;
  double dLon = point2.yy * PI / 180 - point1.yy * PI / 180;
  double aa = sin(dLat / 2) * sin(dLat / 2) +
    cos(point1.xx * PI / 180) * cos(point2.xx * PI / 180) *
    sin(dLon / 2) * sin(dLon / 2);
  double cc = 2 * atan2(sqrt(aa), sqrt(1 - aa));
  double dd = RR * cc;
  return dd; // Kilometers;
}

double disLineToPoint(Point left, Point right, Point point) {
  double mmOriginal;
  double nnOriginal;
  double mmPerpendicular;
  double nnPerpendicular;
  double xxCross;
  double yyCross;
  if (left.xx == right.xx) { // Line parallel to the Y axis.
    xxCross = left.xx;
    yyCross = point.yy;
  }
  else if (left.yy == right.yy) { // Line parallel to the X axis.
    xxCross = point.xx;
    yyCross = left.yy;
  }
  else {
    mmOriginal = ((right.yy - left.yy) / (right.xx - left.xx));
    nnOriginal = (-1) * (mmOriginal * left.xx - left.yy);
    mmPerpendicular = (-1) / mmOriginal;
    nnPerpendicular = (-1) * (mmPerpendicular * point.xx - point.yy);
    xxCross = (nnPerpendicular - nnOriginal) / (mmOriginal - mmPerpendicular);
    yyCross = mmOriginal * xxCross + nnOriginal;
  }
  return disPointToPoint(CreatePoint(xxCross, yyCross, 0, 0, 0, 0), point);
}

Point CreatePoint(double xx, double yy, char index, int hh, int mm, int ss) {
  Point point;
  point.xx = xx;
  point.yy = yy;
  point.hh = hh;
  point.mm = mm;
  point.ss = ss;
  point.polygonIndex = index;
  return point;
}

void printPoint(Point point) {
  printAndCheck("xx: ");
  Serial.print(point.xx, 6);
  printAndCheck(" yy: ");
  Serial.print(point.yy, 6);
  printAndCheck(" Quality: ");
  Serial.print(quality);
  printlnAndCheck("");
}

byte checkQuality(int i_Quality) {
  return (((i_Quality >= 1) && (i_Quality <= 5)) ? (1) : (0));
}

byte pointIsGood(int i_Quality, char* i_Str, byte i_Checksum) {
  return ((checkQuality(quality) && (checkChecksum(buf, checksum) == 1)) ? (1) : (0));
}
