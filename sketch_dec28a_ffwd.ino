/*
   HEADER NOTES ON THE SKETCH STUCTURE AS A REMINDER ....
   (source: http://sewelectric.org/diy-projects/3-programming-your-lilypad/arduino-program-structure/)

   Each Arduino program has three main parts:
   1. variable declaration section
   2. setup section
   3. loop section

   These sections are analogous to the ingredient list (variable declaration),
   preparation steps (setup), and cooking steps (loop) in a recipe.

   When your program runs,
   it will first define your variables (the ingredients that you need),
   then execute the setup section once (set everything up to begin cooking),
   and then execute the loop section over and over (actually do the cooking).
*/




/* from source files for code below...:
    sketch_oct13a_EGR_V_PID1_2_auto-range_real-bias_plus_pot
    and
    sketch_multi_TC_oct_15a_5channels

*/
/***************************************************************
  AL WELCH, TECOGEN INC., (OCT. 13, 2017)
  THIS CODE CONTROLS 2 PID LOOPS IN PARALLEL
  IT IS BASED UPON THE SINGLE PID LOOP CODE BY LISTED IN ARDUINO (AND GITHUB) BY BRETT BEAUREGARD
  IT ALSO HAS AUTO-RANGE CODE, BRUTALLY SIMPLE, TO FIND THE MIN AND MAX POSTIONS ON START-UP BEFORE THE PID CODE FEATURE EXECUTE

* * Arduino PID Library - Version 1.2.1
  by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

  The original single PID Library is licensed under the MIT License
***************************************************************
  - For an ultra-detailed explanation of why the single PID loop code is the way it is, please visit:
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
  ********************************************************/


/********************************************************
  Other Brett Beauregard comments about the original code:
  PID Adaptive Tuning Example
  One of the benefits of the PID library is that you can
  change the tuning parameters at any time.  this can be
  helpful if we want the controller to be aggressive at some
  times, and conservative at others.   in this routine extracted /modified from Adafruit
  we set the controller to use Conservative Tuning Parameters
  when we're near setpoint and more aggressive Tuning
  Parameters when we're farther away.
********************************************************/
/* To Do items...  AUTORANGE FEATURES need bench testing - largely completed as of Oct.13, 2017
   Forklift EGR Valve PID subroutine tested on bench and forklift
  This PID loop (dual) and tuning values works well with 2 Isuzu (automotive)
  EGR Valves  - Wed. Sept 29, 2017 Al Welch
  Uses dual PID routines separately called for EGR Valve A & B to retain PID calcs independently
  channel and variable names follow similar numeric pattern
  Valves A and B uses PID (original) and PID2, repectively
  To Do make a nice diagram of pins and wiring for future troubleshooting
  Valve A -ve side of coil uses Arduino digital pin 3
  Valve A +ve side of coil uses Arduino digital pin 11
  Valve A position signal, pin C, uses Arduino analog A0
  Valve B -ve side of coil uses Arduino digital pin 9
  Valve B +ve side of coil uses Arduino digital pin 10
  Valve B position signal, pin C, uses Arduino analog A1
  Valve A will be attached to the hot exhaust pipe flow
  Valve B will be attached to the cooled exhaust pipe flow
  Ultimately this code will interface with the thermocouple code and try to control the exhaust temperature (mixed gas from valve A and B to about 190 C / 375 F).
  2017-Oct-18 - Added Potentiometer for hot_Bias multiplier
*/

#include <math.h>    // needed for the isnan ( ) function below
#include <PID_v1.h>  // used for first or A EGR valve - this is the core header from Brett B
#include <PID2_v1.h>  // used for second or B EGR valve  - modified header (by Al Welch) based on original PID code
#include <PID3_v1.h>  // added PID3_v1.h for T4 control - it was created by Al Welch as well, and is in the library, as of Nov. 2017 //
/*
   We could have multiple instances of the Morse class, each on their own pin stored in the _pin private variable of that instance.
   By calling a function on a particular instance, we specify which instance's variables should be used during that call to a
   function. That is, if we had both:
   Morse morse(13);
   Morse morse2(12);
   then inside a call to morse2.dot(), _pin would be 12.
   source: https://www.arduino.cc/en/Hacking/LibraryTutorial
*/

#define A_MOTOR_1 3    // connect inj 1 on DRV 8871 board to Arduino digital Digital 3 out
#define A_MOTOR_2 11   // connect inj 2 on DRV 8871 board to Arduino digital Digital 11 out
#define A_actualPosition A0  // position sensor from EGR Valve A

#define B_MOTOR_1 9    // connect inj 1 on DRV 8871 board to Arduino digital Digital 9 out
#define B_MOTOR_2 10   // connect inj 2 on DRV 8871 board to Arduino digital Digital 10 out
#define B_actualPosition A1 // position sensor from EGR Valve B

#define pot A5 // this signal will be used for modifying the hotBias effect via potFactor -- it is just for manual mode

// from left to right looking at connector as wires go in:
// C= signal, B= GND, A =5V
// Motor 1, empty, Motor 2
// view is from back pf connector, sensor at top of Isuzu or GM EGR Valve
//
//
//      *********************CLIP******************
//      *********************TOP*******************
//      **** C=signal ***** B= GND ***** A = +5V **
//      *******************************************
//      **** Motor 1 ****** EMPTY ****** Motor 2 **
//      ********************Bottom*****************

//Define Variables we'll be connecting to
//note that these are globval variables at this level (asopposed to declaring them in setup ( ) or loop ( )  )
double  inputA, outputA, mapA;
double inputB, outputB, mapB;
//double mapTotal = 1000; // crude fixed are concept where 1000 = 100%, slightly finer resolution than  0-255 position capability

// **************************
double fixedHotBias = 1.00 ; // fraction hot valve is open, 1.00 =100%, 0.000 = 0%, used for manually fixing valves
static double hotBias = 0.500;  // intial setting on hotBias
//static double outputHotBias = 0.500;   // this is the value updated every temperature cycle by PID3 to vary the valve A & B target poitions
static double hotBiasCalc1 = 1.000; //base Hot Bias derived from T1 + T2 weighting model prediction (hotBiasCalc1) and lookup table.
static double hotBiasCalc2 = 1.000;  // corrected hot bias based on model data versus Actual HotBias factor used in cloed loop control tests at 220 C.  See Arduino thermal traces (v2).xlsx file.
static double hotBiasCalc3 = 1.000;  // same as hotBiasCalc2 but limited below to 1.000 in min fcn

static double outputHotBiasAdjust  = 0.0001;  // initial setting to be altered by myPID3 based on (targetT4 - T4) error, +ve error means this ajustment is positive.  Still to be tuned as of Dec 28th, 2017.



// **************************

static double setpointA = 300.0;   // initialize value but to be changed in main loop
static double setpointB = 600.0;  // initialize value but to be changed in main loop
double maxSF = 0.95;  // reduce this if the valve rests when dwelling fo a long time in a single position (
// -->maxSF (maximum safey factor) on on physical range of valves 0.95 means we only use 95% of the effective range (i.e. maxA-minA), as an expample.

//Define the aggressive and conservative valve Exhaust Valve PID 234Tuning Parameters, works for 2 valves using PID and PID2 respectively
double aggKp = 0.2, aggKi = 1.0, aggKd = 0.0005;   // bench tuned
double consKp = 0.1, consKi = 0.5, consKd = 0.0001; // bench tuned

//Define temperture loop Variables we'll be connecting to
double targetT4 = 220.0; // 190 C is normal target temperature for catalyst inlet.  To be used in PID3_v1.h loop

static double T5 = 50.00;  // this may get replaced by T3  (it uses tcDevice3)
static double T5LastGoodValue = 50.00; // updated and used in ifnan () routine below at about line 670, near bottom
static double T4 = 50.00; // starting value for inlet of oxi-cat to put reasonable value in myPID3, assumes 1/2 warm engine
static double T4LastGoodValue = 50.00; // updated and used in ifnan () routine below at about line 670, near bottom
static double T2 = 50.00; // uses tcDevice2
static double T2LastGoodValue = 50.00; // updated and used in ifnan () routine below at about line 670, near bottom
static double T1 = 50.00; // uses tcDevice1
static double T1LastGoodValue = 50.00; // updated and used in ifnan () routine below at about line 670, near bottom
static double T0 = 50.00; // uses tcDevice0
static double T0LastGoodValue = 50.00; // updated and used in ifnan () routine below at about line 670, near bottom

// updated 2017-Dec-29 for  ifnan ( ) protection on T5, T4, T2, T1, T0 


//Define the aggressive and conservative Tuning Parameters . -- maybe modify this for temperature control
double T4aggKp = 0.01, T4aggKi = 0.001, T4aggKd = 0.001;   // forklift tuned
double T4consKp = 0.01, T4consKi = 0.001, T4consKd = 0.001; // forklift tuned



//Specify the links and initial tuning parameters
PID myPID(&inputA, &outputA, &setpointA, consKp, consKi, consKd, DIRECT);  //last KEYWORDS.txt must ontain all definitions as it displaces previous KEYWORDS.txt
PID2 myPID2(&inputB, &outputB, &setpointB, consKp, consKi, consKd, DIRECT); //in other folders....understand why PID is black while PID2 is colour text
PID3  myPID3(&T4, &outputHotBiasAdjust, &targetT4, T4consKp, T4consKi, T4consKd, DIRECT);

/*  &tempC_tcDevice4, uses address for thermocouple 4 whichis T4 oxi-cat inlet
    &hotBias value modified slower the setpointA and setpointB to modify pre oxi-cat T4
    &targetT4, base temperature target
    T4consKp,
    T4consKi,
    T4consKd,
    DIRECT ... I belive this if forward control ... if higher temperature is needed then hot Bias is increased
*/

/*
   From Arduino library: Each line has the name of the keyword, followed by a tab (not spaces), followed by the kind of keyword.
   Classes should be KEYWORD1 and are colored orange; functions should be KEYWORD2 and will be brown.
   You'll have to restart the Arduino environment to get it to recognize the new keywords.
*/

/* Notes on Address operator used above for arguments 1, 2, 3 ...
    from
   https://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html
   1.2  Declaring Pointers
   Pointers must be declared before they can be used, just like a normal variable.
   The syntax of declaring a pointer is to place a * in front of the name.
   A pointer is associated with a type (such as int and double) too.
   type *ptr    or  type* ptr or type * ptr all work ....
   Address-Of Operator (&)
   When you declare a pointer variable, its content is not initialized.
   In other words, it contains an address of "somewhere", which is of course not a valid location.
   This is dangerous! You need to initialize a pointer by assigning it a valid address.
   This is normally done via the address-of operator (&).
   The address-of operator (&) operates on a variable, and returns the address of the variable.
   For example, if number is an int variable, &number returns the address of the variable number.
   You can use the address-of operator to get the address of a variable, and assign the address to a pointer variable.

*/

// ********************
// *****code boundary**
// ********************

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress tcDevice0, tcDevice1, tcDevice2, tcDevice3, tcDevice4;

// TO DO:  Simplify the temperature section. it may be overly complicated for what we need.








void setup() {
  // put your setup code here, to run once:
  // The setup() function is called when a sketch starts.
  //  Use it to initialize variables, pin modes, start using libraries, etc.
  // The setup function will only run once, after each powerup or reset of the Arduino board.

  // start serial port
  Serial.begin(9600);
  // Serial.println("Control loop");
  pinMode(A_MOTOR_1, OUTPUT);   // Motor 1 on 1st DRV8871 board
  pinMode(A_MOTOR_2, OUTPUT);   // Motor 2 on 1st DRV8871 board; this voltage swings between zero and +ve,  spring return used
  pinMode(A_actualPosition, INPUT);
  digitalWrite (A_MOTOR_1, LOW);  // Motor 1 on DRV8871 board

  pinMode(B_MOTOR_1, OUTPUT);   // Motor 1 on 1st DRV8871 board
  pinMode(B_MOTOR_2, OUTPUT);   // Motor 2 on 1st DRV8871 board; this voltage swings between zero and +ve,  spring return used
  pinMode(B_actualPosition, INPUT);
  digitalWrite (B_MOTOR_1, LOW);  // Motor 1 on DRV8871 board

  pinMode(pot, INPUT);  // pot pin is an input

  //initialize the variables we're linked to
  inputA = analogRead(A_actualPosition);
  inputB = analogRead(B_actualPosition);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);

  //added PID limits, optional function inside PID*.cpp (see library code for detail).
  //This was from the original author.  standard limits are 0,255 by the way
  myPID.SetOutputLimits(0, 250);  // sets PWM limits for Valve A
  myPID2.SetOutputLimits(0, 250); // sets PWM limits for Valve B
  myPID3.SetOutputLimits(0.00, 1.00); // sets hotBias limits for tempreature control agorithm
  //Serial.println("line 231 completed");

  //********************
  //*****code boundary**
  //********************


  //Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

  // locate devices on the bus
  //Serial.print("Locating devices...");
  //Serial.print("Found ");
  //Serial.print(sensors.getDeviceCount(), DEC);
  //Serial.println(" devices.");
  //Serial.println(" note: VERIFY SINGLE WIRE SPI JUMPER CONNECTIONS IF # DOES NOT MATCH TARGET i.e. 5 expected");

  // report parasite power requirements
  //Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // assign address manually.  the addresses below will beed to be changed
  // to valid device addresses on your bus.  device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  //outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  if (!sensors.getAddress(tcDevice0, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(tcDevice1, 1)) Serial.println("Unable to find address for Device 1");
  if (!sensors.getAddress(tcDevice2, 2)) Serial.println("Unable to find address for Device 2");
  if (!sensors.getAddress(tcDevice3, 3)) Serial.println("Unable to find address for Device 3");
  if (!sensors.getAddress(tcDevice4, 4)) Serial.println("Unable to find address for Device 4");




  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them.  It might be a good idea to
  // check the CRC to make sure you didn't get garbage.  The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
  // assigns the seconds address found to outsideThermometer
  //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(tcDevice0);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(tcDevice1);
  Serial.println();

  Serial.print("Device 2 Address: ");
  printAddress(tcDevice2);
  Serial.println();

  Serial.print("Device 3 Address: ");
  printAddress(tcDevice3);
  Serial.println();

  Serial.print("Device 4 Address: ");
  printAddress(tcDevice4);
  Serial.println();

  // set the resolution to 9 bit
  sensors.setResolution(tcDevice0, TEMPERATURE_PRECISION);
  sensors.setResolution(tcDevice1, TEMPERATURE_PRECISION);
  sensors.setResolution(tcDevice2, TEMPERATURE_PRECISION);
  sensors.setResolution(tcDevice3, TEMPERATURE_PRECISION);
  sensors.setResolution(tcDevice4, TEMPERATURE_PRECISION);

  //Serial.print("Device 0 Resolution: ");
  //Serial.print(sensors.getResolution(tcDevice0), DEC);
  //Serial.println();

  //Serial.print("Device 1 Resolution: ");
  //Serial.print(sensors.getResolution(tcDevice1), DEC);
  //Serial.println();

  //Serial.print("Device 2 Resolution: ");
  //Serial.print(sensors.getResolution(tcDevice2), DEC);
  //Serial.println();

  //Serial.print("Device 3 Resolution: ");
  //Serial.print(sensors.getResolution(tcDevice3), DEC);
  //Serial.println();

  //Serial.print("Device 4 Resolution: ");
  //Serial.print(sensors.getResolution(tcDevice4), DEC);
  //Serial.println();

}


// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    delay (50);
  }

}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);   // useful code line replicated below to return temperature for PID control
  Serial.print("Temp C: ");
  Serial.print(tempC);
  // globalTempC = int(tempC+0.5);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}




void loop()
{
  // put your main code here, to run repeatedly:

  // Serial.println ("debug- line 391");

  static int minA = 400;  // per Simon Monk (p.54) initial value is only intialized first time this loop( ) runs.  This is a very useful function.
  static int minB = 400;
  static int maxA = 600;
  static int maxB = 600;

  minA = min(inputA, minA);  // compares latest valve A position to last min and assigns to new min
  minB = min(inputB, minB);  // compares latest valve B position to last min and assigns to new min
  maxA = max(inputA, maxA);  // compares latest valve A position to last max and assigns to new max, useful in case key-on learning loop did not open valve, however, may catch noise
  maxB = max(inputB, maxB);  // compares latest valve B position to last max and assigns to new max, useful in case key-on learning loop did not open valve, however, may catch noise
  mapA = map(inputA, minA, maxA, 0, 1000);  // scaled to 1000 for easy reference
  mapB = map (inputB, minB, maxB, 0, 1000);  // scaled top 1000 for easy reference

  delay(100);  // gives time to stabilize input channels ...


  inputA = analogRead(A_actualPosition);
  inputB = analogRead(B_actualPosition);
  //float potValue = analogRead(pot);
  float potFactor = analogRead(pot) / 1023.0 ; // I tried to use INT but this is tied to all the PID math with requites setpointA and setpointB as double (float in Arduino compiler)

  static unsigned int millisStartPoint = millis ( );  //compensates for slower code so the autorange timers adjust to time when available

  // ******* AUTO RANGE LOOP HERE ***** executes in 1.5 sec - using mills ( ) timer.
  // this is simple / brute force method with de-energized DC motors, 0.0 and 0.5 seconds, spring assisted closure for zero position
  // and near full PWM for max position
  // auto-range continues in core PID loop afterward too.  This will tend to make limits wide if any noise pops up.

  // Zero position testing ...

  while   (millis( ) < (1000 + millisStartPoint))
  {

    Serial.println(" *** Min Position Auto Range Executing for 1.0 sec  ");
    Serial.print("  millis( ) = " );
    Serial.println(millis( ));

    analogWrite(A_MOTOR_2, 0); // zero pwm, get a new reading on valve position again
    analogWrite(B_MOTOR_2, 0); // zero pwm, get a new reading on valve position again
    delay (100);  // allows brief settling for first reading, but allows loop to possibly get up to 10 readings
    inputA = analogRead(A_actualPosition);  // makes sure code checks latest value
    inputB = analogRead(B_actualPosition);  // makes sure code checks latest value



    //  Serial.print("    gapA =  ");
    //  Serial.print(gapA);
    //   Serial.print(",  inputA =  ");
    //  Serial.print(inputA);

    //  Serial.print(" ******    minA =  ");
    //  Serial.print(minA);
    //  Serial.print(",  minB =  ");
    //   Serial.print(minB);

    // Serial.print(" maxA =  ");
    //  Serial.print(maxA);
    //  Serial.print(",  maxB =  ");
    //  Serial.print(maxB);

    minA = min(inputA, minA);  // compares latest valve A position to last min and assigns to new min
    minB = min(inputB, minB);  // compares latest valve B position to last min and assigns to new min
    //  maxA = max(inputA, maxA);  // compares latest valve A position to last max and assigns to new max
    // maxB = max(inputB, maxB);  // compares latest valve B position to last max and assigns to new max

    //  Serial.println ("debug- line 463 approx");

  }

  while   ((1000 + millisStartPoint <= millis ( )) && (millis( ) < 2000 + millisStartPoint))
  {
    // full open valve between 1.0 and 4.0 seconds, after offset allowance millisStartPoint
    // Serial.println(" *** Max Position Auto Range - Executing for 1.0 sec  ");
    Serial.print("  millis( ) = " );
    Serial.println(millis( ));

    analogWrite(A_MOTOR_2, 240); //  high pwm, but not position control to open it fully.  To Do: recheck on forklift with battery feed rather thane the 12 VDC/5A desk supply
    analogWrite(B_MOTOR_2, 240); //  240 seems ok (on bench) to quickly get a reading  - creates minor slam & whining.  on bench, 200 still opens valve fully.  150 is only about 10% open.
    delay (100);  // keeps loop quick for multiple readings, allows settling
    inputA = analogRead(A_actualPosition);  // makes sure code checks latest value, will not update otherwise
    inputB = analogRead(B_actualPosition);  // makes sure code checks latest value, will not update otherwise

    //minA = min(inputA, minA);  // compares latest valve A position to last min and assigns to new min
    //minB = min(inputB, minB);  // compares latest valve B position to last min and assigns to new min
    maxA = max(inputA, maxA);  // compares latest valve A position to last max and assigns to new max
    maxB = max(inputB, maxB);  // compares latest valve B position to last max and assigns to new max

    //   Serial.print("    gapA =  ");
    //   Serial.print(gapA);
    //    Serial.print(",  inputA =  ");
    //    Serial.print(inputA);
    //    Serial.print(",  inputB =  ");
    //    Serial.print(inputB);


    Serial.print(" ******    minA =  ");
    Serial.print(minA);
    Serial.print(",  minB =  ");
    Serial.print(minB);

    Serial.print(" maxA =  ");
    Serial.print(maxA);
    Serial.print(",  maxB =  ");
    Serial.print(maxB);


    // Serial.print(millis( ));
    // Serial.print(" ******    minA =  ");
    //  Serial.print(minA);
    //  Serial.print(",  minB =  ");
    //  Serial.print(minB);

    //  Serial.print(" maxA =  ");
    //   Serial.print(maxA);
    //  Serial.print(",  maxB =  ");
    //  Serial.println(maxB);

  }


  // MAIN CODE SECTION - t//


  //  CRITICAL SECTION TO MANUALLY MODIFY
  // *******************************************
  // Choose (1) or (2) or (3) only one method and turn others off with // ...
  //  (1) fixed ratio method
  //  setpointA =  ((maxA - minA) * (fixedHotBias) ) + minA; //
  //  setpointB = ((maxB - minB) * (1.000 - fixedHotBias) )  + minB; //

  //  (2) potentiometer method
  //setpointA =  ((maxA - minA) * (potFactor)) + minA; //  attempted to keep this as nice integer math...
  //setpointB = ((maxB - minB) * (1.0 - potFactor))  + minB; //

  //  (3) automatic temperature feedback method
  setpointA =  (maxSF * ((maxA - minA) * (hotBias))) + minA; // hotBias is driven by PID3 and T4 (oxi-cat inlet), usually.  Later hotBias = outputHotBiasAdjust + hotBiasCalc3
  //We could use T5 (post oxi-cat) too.
  setpointB = (maxSF * ((maxB - minB) * (1.000 - hotBias)))  + minB; //
  // *******************************************

  //Serial.print(" ******    potFactor =  ");
  //Serial.println(potFactor);

  //Serial.print(",  setpointA =  ");
  //Serial.print(setpointA);

  //Serial.print(",  setpointB =  ");
  //Serial.print(setpointB);

  //Serial.print(" inputA =  ");
  //Serial.print(inputA);
  //Serial.print(" inputB =  ");
  //Serial.print(inputB);

  double gapA = (setpointA - inputA); //+ve distance between setpoint from actual
  double gapB = (setpointB - inputB); //+ve distance between setpoint from actual


  Serial.print("  *** millis( ) = " );
  Serial.print(millis( ));

  // Serial.print ("  gapA  = ") ;
  // Serial.print (gapA) ;
  // Serial.print ("  gapB  = ") ;
  // Serial.print (gapB) ;


  //Serial.print(" ******    fixedHotBias =  ");
  //Serial.print(fixedHotBias);

  Serial.print(" ******    hotBias =  ");
  Serial.print(hotBias);

  Serial.print(" ******    outputHotBiasAdjust =  ");
  Serial.print(outputHotBiasAdjust);

  //Serial.print(" ******    minA =  ");
  //Serial.print(minA);
  //Serial.print(",  minB =  ");
  //Serial.print(minB);

  //Serial.print(" maxA =  ");
  //Serial.print(maxA);
  //Serial.print(",  maxB =  ");
  //Serial.print(maxB);

  //Serial.print ("  mapA  = ") ;
  //Serial.print (mapA) ;
  //Serial.print ("  mapB  = ") ;
  //Serial.print (mapB) ;


  if (abs(gapA) < 40)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  { //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }


  if (abs(gapB) < 40)
  { //we're close to setpoint, use conservative tuning parameters
    myPID2.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID2.SetTunings(aggKp, aggKi, aggKd);
  }

  // for reference --- PID3 myPID3(&T4, &outputHotBiasAdjust, &targetT4, T4consKp, T4consKi, T4consKd, DIRECT);

  if (abs(T4 - targetT4) < 0)  // temp. to isolate calc glitch in transition
  { //we're close to setpoint, use conservative tuning parameters
    myPID3.SetTunings(T4consKp, T4consKi, T4consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID3.SetTunings(T4aggKp, T4aggKi, T4aggKd);
  }

  /*Serial.println("**REFERENCE DATA ONLY FOR: PID myPID (&inputA, &outputA, &setpointA, consKp, consKi, consKd, DIRECT)") ;
    Serial.print (",  ");
    Serial.print (inputA);
    Serial.print (",  ");
    Serial.print (outputA);
    Serial.print (",  ");
    Serial.print (setpointA);
    Serial.print (",  ");
    Serial.print (consKp);
    Serial.print (",  ");
    Serial.print (consKi);
    Serial.print (",  ");
    Serial.print (consKd);
    Serial.print (",  ");
    Serial.println (DIRECT) ;
    Serial.print ("  maxA =  ") ;
    Serial.print (maxA);
    Serial.print ("  maxB =  ") ;
    Serial.println (maxB);
  */
  //Serial.println("**REFERENCE DATA ONLY FOR: PID2 myPID2(&inputB, &outputB, &setpointB, consKp, consKi, consKd, DIRECT)");
  //Serial.println("**REFERENCE DATA ONLY FOR: PID3  myPID3(&T4, &outputHotBiasAdjust, &targetT4, T4consKp, T4consKi, T4consKd, DIRECT)");



  myPID.Compute();
  analogWrite(A_MOTOR_2, outputA);     // magic occurs here as Brett B says...
  myPID2.Compute();
  analogWrite(B_MOTOR_2, outputB);
  myPID3.Compute();
  hotBias = outputHotBiasAdjust + hotBiasCalc3;  // ??? not necessary as myPID3 modifies hotBias to reach T4target



  // Serial.print("    gapA =  ");
  // Serial.print(gapA);
  //  Serial.print(",  inputA =  ");
  //  Serial.print(inputA);
  // Serial.print(",  outputA =  ");
  // Serial.print(outputA);



  //  Serial.print(" gapB =  ");
  // Serial.print(gapB);
  // Serial.print(",  inputB =  ");
  // Serial.print(inputB);



  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();
  //Serial.println("DONE");

  // print the device information
  //printData(tcDevice0);
  //printData(tcDevice1);
  //printData(tcDevice2);
  //printData(tcDevice3);
  //printData(tcDevice4);


  Serial.print("  **********  T0 = ");
  Serial.print(T0);
  Serial.print("   T1 = ");
  Serial.print(T1);
  Serial.print("   T2 = ");
  Serial.print(T2);
  Serial.print("     targetT4 = ");
  Serial.print(targetT4);
  Serial.print("   T4 = ");
  Serial.print(T4);
  Serial.print("   T5 = ");
  Serial.println(T5);


  // float tempC_TCx [5] = { sensors.getTempC(tcDevice0), sensors.getTempC(tcDevice1), sensors.getTempC(tcDevice2), sensors.getTempC(tcDevice3), sensors.getTempC(tcDevice4) };
  // return tempC_TCx [5]; // to do: turn these values to a global variable for PID control on exhaust temperature
  // Serial.print(" tempC_TCx [5]  ");
  // Serial.println(tempC_TCx [5]); // 5th variable correspomding 4 position. 0 is first address.


  //Serial.println(" line 661 executed ");


  //Serial.println(" line 672 executed ");

  //To Do : review next comments.  NOT correct possible....
  float tempC_tcDevice0 = sensors.getTempC(tcDevice0);   //tcDevice0 wye at split to hot & cool sides -TO DO: use later for DAQ
  delay (1);
  float tempC_tcDevice1 = sensors.getTempC(tcDevice1);  // tcDevice1 is T1 hot pipe -TO DO: use later for DAQ
  delay (1);
  float tempC_tcDevice2 = sensors.getTempC(tcDevice2);  // tcDevice2 is T2 cool pipe -TO DO: use later for DAQ
  delay (1);
  float tempC_tcDevice3 = sensors.getTempC(tcDevice3);  // tcDevice3 is T5 post oxi-cat (and control), could be T3 (air inj) -TO DO: use later for DAQ
  delay (1);
  float tempC_tcDevice4 = sensors.getTempC(tcDevice4);  // tcDevice4 is possibly T4 oxi-cat inlet
  delay (1);

  T0 =  tempC_tcDevice0;
  T1 =  tempC_tcDevice1;
  T2 =  tempC_tcDevice2;
  T5 =  tempC_tcDevice3;
  T4 =  tempC_tcDevice4;



  // Protection routine for thermocouple 4 data
  /*
    static int y5 = isnan ( T5);
    static int y4 = isnan ( T4);  //nan trap to catch bad data from thermocouple 4, isnan (X) =1 for nan case ...
    static int y2 = isnan ( T2);
    static int y1 = isnan ( T1);
    static int y0 = isnan ( T0);
  */

  // code below rejects T1, T2, T4 nan errors and selects last good value.

  if (isnan(T0))  // if the nan (not a number) occurs, then grab known good data to get over the "glitch", else update  ...
  {
    T0 = T0LastGoodValue;
    Serial.println("T1 NaN discovered ... ");
  }
  else
  {
    T0LastGoodValue = T0;  // ok to store T0 as good value for next decision loop
  }

  if (isnan(T1))  // if the nan (not a number) occurs, then grab known good data to get over the "glitch", else update  ...
  {
    T1 = T1LastGoodValue;
    Serial.println("T1 NaN discovered ... ");
  }
  else
  {
    T1LastGoodValue = T1;  // ok to store T1 as good value for next decision loop
  }

  if (isnan(T2))  // if the nan (not a number) occurs, then grab known good data to get over the "glitch", else update  ...
  {
    T2 = T2LastGoodValue;
    Serial.println("T2 NaN discovered ... ");
  }
  else
  {
    T2LastGoodValue = T2;  // ok to store T2 as good value for next decision loop
  }


  if (isnan(T4))  // if the nan (not a number) occurs, then grab known good data to get over the "glitch", else update  ...
  {
    T4 = T4LastGoodValue;
    Serial.println("T4 NaN discovered ... ");
  }
  else
  {
    T4LastGoodValue = T4;  // ok to store T4 as good value for next decision loop
  }

  if (isnan(T5))  // if the nan (not a number) occurs, then grab known good data to get over the "glitch", else update  ...
  {
    T5 = T5LastGoodValue;
    Serial.println("T5 NaN discovered ... ");
  }
  else
  {
    T5LastGoodValue = T5;  // ok to store T5 as good value for next decision loop
  }


  // this protects against NaN in print statements too which make it difficult to process for excel analysis

  //theoretical hb= (targetT4 - T2)/ (T1-T2) from model
  hotBiasCalc1 = (targetT4 - T2) / (T1 - T2); // first level estimation from simplest math
  hotBiasCalc2 =  (4.5229 * (pow(hotBiasCalc1, 3))) - (5.099 * (pow(hotBiasCalc1, 2))) + (2.451 * hotBiasCalc1); // lookup function from transient tests --> 2nd level correction - see Arduino Thermal Traces (v2).xlsx in test notes
  hotBiasCalc3 = min(hotBiasCalc2, 1.0);   // limits fcn to 1.000, needed above hotBiasCalc1 = 0.800
  Serial.print (hotBiasCalc1 );
  Serial.print (",  ");
  Serial.print (hotBiasCalc2 );
  Serial.print (",  ");
  Serial.print (hotBiasCalc3 );
  Serial.println (",  ");    // these are just temporary checks.


  // ALTERNATIVELY instead of polynomial for hotBiasCalc3:   TO DO - LOOKUP/indexing table (say 2 x 10) FUNCTION GOES HERE !!!!!  (Maybe use pointers?)


  /* Serial.print ("NAN checker: y5, y4, y2, y1, y0 =   ");
    Serial.print (y5 );
    Serial.print (",  ");
    Serial.print (y4 );
    Serial.print (",  ");
    Serial.print (y2 );
    Serial.print (",  ");
    Serial.print (y1 );
    Serial.print (", " );
    Serial.println (y0 );
    Serial.print ("Data T5, T4, T2, T1, T0 =   ");
    Serial.print (T5 );
    Serial.print (",  ");
    Serial.print (T4 );
    Serial.print (",  ");
    Serial.print (T2 );
    Seiu53gwfcxrial.print (",  ");
    Serial.print (T1 );
    Serial.print (" " );
    Serial.print (T0 );
  */
  //Serial.println("    line 801 executed ");
}
