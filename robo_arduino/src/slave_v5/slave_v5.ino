/******************************************************************
 * Bluetooth slave driver for Power Glove controller for InMoov 
 * arm via Pololu motor driver
 * 
 * Melih Erdrogan, Alex Renaud, Casimir Sowinski
 * 11/01/2015 
 *****************************************************************/

// Adding a median filter

/*
 * Index  0  
 * Middle 1
 * Ring   2
 * Pinky  3
 * Thumb  4
 * Wrist  5
 */
 
#include <PololuMaestro.h>

#define Serial1 SERIAL_PORT_HARDWARE_OPEN

MiniMaestro maestro(Serial1);
 
// Flags
boolean stringComplete  = false;    // whether the string is complete
boolean calibrated      = false;    // keeps track of whether cal has been completed
boolean cal_First       = true;     // keeps track of whether first pass through cal
boolean cal_Request     = false;    // kkeps track of whether to rerun calibration

String inputString      = "";       // a string to hold incoming data

const int samples       = 30;
int angle_Hist[6][samples] = {0};
long long angle_Ave[6]     = {0};

// Target names for printout
String target[6]        = {"Index", "Middle", "Ring", "Pinky", "Thumb", "Wrist"}; 

int bound_Lower[6]      = {0};      // raw lower limir
int bound_Upper[6]      = {0};      // raw upper limit
//int bound_Lower[6]    = {36, 26, 47, 108, 39, 250};
//int bound_Upper[6]    = {6, 9, 10, 161, 20, -560};

const int cal_Iterations      = 40;       // number of cal frames to take
int cal_Step = cal_Iterations;
int cal_Dummy           = 0;        // var for raw cal reading
int deadband            = 10;      // deadband, if diff bt last and new angle<this, dont update
int smoothing           = 2;        // Bigger=smoother,

const int delay_ms      = 10;  // number of ms to delay between iterations
int test = 0;

void setup() 
{ 
  // initialize serial: 
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600); 
  
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);  

  // Initialize finger position
  for(int i = 0;i < 5;i++)
  {
    maestro.setTarget(i, 6000);
    delay(100);
  }
  
}

void loop()
{  
  //testServos(); 
  //Serial.println(test++);     
  //printValues();

  //Serial.println(inputString);
  
  //*
  // Calibrate or read and update
  if(calibrated == false)
  {
    calibrate();
  }
  else
  {
    capture();
    update_Servos();
    //Serial.println((int)angle_Ave[0]);
  }  
  // To rerun calibration
  if(cal_Request)
  {
    calibrated = false;
    cal_Step = cal_Iterations;
    cal_First = true;
  }
  //*
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
*/
void serialEvent() 
{
  //Serial.println("serialEvent entered");
  while (Serial2.available()) 
  {    
    //Serial.println("Serial2 available");
    // get the new byte:
    char inChar = (char)Serial2.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') 
    {
      stringComplete = true;
      //Serial.println("stringComplete");
    }
  }
}

void capture()
{
  // Capture data
  serialEvent();  
  // Parse data 
  if (stringComplete) 
  {   
    // Loop through fingers, wrist  
    for(int i = 0; i < 6; i++)
    {      
      // Read values
      angle_Hist[i][0] = getValue(inputString, ':', i).toInt();  
    }     
    // Delay, reset flag and string
    //delay(delay_ms);    
    inputString = "";
    stringComplete = false;
    //Serial.println(inputString);
  }
}

// Find flex sensor minimum and maximum values
void calibrate()
{
  // Run through calibration cal_Iterations times
  if(cal_Step > 0)
  {
    // Print promp
    Serial.print("Calibrating. Please fully flex fingers and rotate wrist. ["); 
    Serial.print(cal_Step);
    Serial.print("]");
    Serial.print("\n");
    // Get data from sensors
    serialEvent();    
    // If data is ready, run parsing
    if (stringComplete) 
    {         
      // Check all fingers and wrist (targets)
      for(int i = 0; i < 6; i++)
      {        
        // If first pass, initialize bounds
        if(cal_First)
        {
          bound_Lower[i] = getValue(inputString, ':', i).toInt();
          bound_Upper[i] = getValue(inputString, ':', i).toInt();
          cal_First = false;
        }
        else  // else, update bounds with minimums and maximums found
        {
          // Get current reading
          cal_Dummy =  getValue(inputString, ':', i).toInt();

          // Set new bounds, if pinky (i == 3), reverse
          //if(i != 3)
          if(true)
          {
            if(cal_Dummy < bound_Lower[i])
            {
              bound_Lower[i] = cal_Dummy;
            }
            if(cal_Dummy > bound_Upper[i])
            {
              bound_Upper[i] = cal_Dummy;
            }   
          }
          else
          {
            if(cal_Dummy < bound_Upper[i])
            {
              bound_Upper[i] = cal_Dummy;
            }
            if(cal_Dummy > bound_Lower[i])
            {
              bound_Lower[i] = cal_Dummy;
            }               
          }
        }               
      } 
      // Delay, clear string/flag
      delay(delay_ms);
      inputString = "";
      stringComplete = false;
    }  
    // Decrement cal_Iterations
    cal_Step--;  
  }
  else
  {
    // Set flag
    calibrated = true;
    Serial.println("Calibration complete");
  }
}

// Parse 
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for(int i = 0; i <= maxIndex && found <= index; i++)
  {
    // If at the seperator (:) or end of data 
    if(data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  // Return value after seperator if any are found, "" otherwise
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void update_Servos()
{
  // Loop through all fingers, and wrist
  for(int i = 0; i < 5; i++)
  {    
    // New smoothing filter    
    // Back propagate readings
    for(int j = samples - 1; j > 0; j--)
    {
      angle_Hist[i][j] = angle_Hist[i][j - 1];
    }  
    
    // Reset tempAve
    angle_Ave[i] = 0;
    
    // Average last 10 readings
    for(int j = 0; j < samples; j++)
    {
      //Serial.print(i);
      //Serial.print(" ");
      //Serial.println(angle_Ave[i]);
      angle_Ave[i] += angle_Hist[i][j];
    }
    angle_Ave[i] = angle_Ave[i] / samples;

    /*
    if(i == 3)//pinky
    {
      angle_Ave[i] = map(angle_Ave[i], bound_Upper[i], bound_Lower[i], 9000, 3000);  
    }
    else if(i == 4)//thumb
    {
      angle_Ave[i] = map(angle_Ave[i], bound_Upper[i], bound_Lower[i],10000, 4000);
    }
    else
    {
      angle_Ave[i] = map(angle_Ave[i], bound_Upper[i], bound_Lower[i], 8000, 4000);
    }
    */    
    if(i != 3)
    {
      angle_Ave[i] = map(angle_Ave[i], bound_Upper[i], bound_Lower[i], 8000, 1000);  
    }
    else
    {
      angle_Ave[i] = map(angle_Ave[i], bound_Upper[i], bound_Lower[i], 1000, 8000);
    }

    if(angle_Ave[i] > 8000)
    {
      angle_Ave[i] = 8000;
    }
    else if(angle_Ave[i] < 4000)
    {
      angle_Ave[i] = 4000;
    }
    
    maestro.setTarget(i, angle_Ave[i]);
   
    if(i == 3)//pinky
    {
      maestro.setTarget(i, angle_Ave[i]);
    }
    else if(i == 4)//thumb
    {
      maestro.setTarget(i, angle_Ave[i]);
    }
    else
    {
      maestro.setTarget(i, angle_Ave[i]);
    }
    
    //maestro.setTarget(i, angle_Ave[i]);
    if(i == 0)
    {
      //Serial.println(angle_Hist[0][0]);
      
      
      
      //Serial.println(angle_Ave[i]);
    }
  }  
}

void testServos()
{
  
  for(int i = 5;i > -1;i--)
  {
    maestro.setTarget(i, 4000);
    delay(500);
  }  
  //delay(1000);
  for(int i = 0;i < 5;i++)
  {
    maestro.setTarget(i, 8000);
    delay(500);
  }
  //delay(1000);  
  
}

void printValues()
{
  /*
  //Serial.println(report);
  Serial.print("Index: ");
  Serial.print(angle_Index);
  Serial.print(",\tMiddle: ");
  Serial.print(angle_Middle);
  Serial.print(",\tRing: ");
  Serial.print(angle_Ring);
  Serial.print(",\tPinky: ");
  Serial.print(angle_Pinky);
  Serial.print(",\tThumb: ");
  Serial.print(angle_Thumb);
  Serial.print(",\tWrist: ");
  Serial.print(angle_Wrist); 
  Serial.print("\n");
  */
}
/*
// Used for testing
void toggle()
{
  if(tog)
  {
    tog = false;
  }
  else
  {
    tog = true;
  }
}
*/

