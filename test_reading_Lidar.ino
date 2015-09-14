#include <AFMotor.h>
 AF_DCMotor motor1(1);


int inByte = 0;         // incoming serial byte
int goalSpeedRPH=18000;     // goal speed for motor

unsigned char data[360];// array for holding the distance data, represent in cm, 2.55 m max for 360 degree.


unsigned char Data_status=0;
unsigned char Data_4deg_index=0;
unsigned char Data_loop_index=0;
unsigned char SpeedRPHhighbyte=0; // hold speed reading  
unsigned char SpeedRPHLowbyte=0; 
int SpeedRPH=0;         // actual speed for motor
unsigned char DistLowbyte = 0; // hold distance reading
unsigned char DistHighbyte =0 ;
int dist=0;             // reading of the distance in mm
unsigned char PWMduty = 224; //give enough speed to start motor   
unsigned char PWMdutyMax = 224;
unsigned char PWMdutyMin = 100;

// 

void setup() {
  // config serial
    Serial.begin(115200);  // USB serial
    Serial3.begin(115200);  // XV-11 LDS data 
    Serial.println("Arduino Neato XV-11 testing...");
    //config motor driver 
   
    motor1.setSpeed(PWMduty);
    motor1.run(FORWARD);
}

void loop() {
  // try to read from Lidar and decode
  if (Serial3.available() > 0) {
    // get incoming byte:  
    inByte =  Serial3.read();
    decodeData(inByte); 
    //Serial.write(inByte);
  }
}

/*
A full revolution will yield 90 packets, containing 4 consecutive readings each. 
The length of a packet is 22 bytes. This amounts to a total of 360 readings (1 per degree) on 1980 bytes.
Each packet is organized as follows:
<start byte> <index> <speed> <Data 0> <Data 1> <Data 2> <Data 3> <checksum>
where:
<start byte> is always 0xFA
<index> is the index byte in the 90 packets,
going from A0 (packet 0, readings 0 to 3) to F9 (packet 89, readings 356 to 359). 
<speed> is a two-byte information, little-endian. It represents the speed.
<Data 0> to <Data 3> are the 4 readings. Each one is 4 bytes long,
organized as follows :
byte 0 : <distance 7:0>
byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
byte 2 : <signal strength 7:0>
byte 3 : <signal strength 15:8>

*/
void decodeData(unsigned char inByte){
  switch (Data_status){
  case 0: // no header
  if (inByte==0xFA)
  {
    Data_status=1;
    Data_loop_index=1;
  }
    break;
  case 1: // Find 2nd FA , packet length 22 bytes
    if (Data_loop_index==22){
      if (inByte==0xFA)
      {
        Data_status=2;
        Data_loop_index=1;
      } 
      else 
      Data_status=0; // if not FA search again
    }
    else{
      Data_loop_index++;
    }
    break;
  case 2: // Read data out  
     if (Data_loop_index==22){
      if (inByte==0xFA)
      {
        Data_loop_index=1;
      } 
      else // if not FA search again
      Data_status=0;
    }
    else{
      readData(inByte);
      Data_loop_index++;
    }
    break;
  }
  
}
void readData(unsigned char inByte){
  switch (Data_loop_index){
    case 1: // <index>4 degree index 
    Data_4deg_index=inByte-0xA0;  // packet index from 0 to 89  
      //Serial.print(Data_4deg_index, HEX);  
      //Serial.print(": ");  
    break;
    case 2: //<speed> Speed in RPH low byte
    SpeedRPHLowbyte=inByte;
    break;
    case 3: // <speed>Speed in RPH high byte
    SpeedRPHhighbyte=inByte;
    SpeedRPH=(SpeedRPHhighbyte<<8)|SpeedRPHLowbyte;
    SpeedControl ( SpeedRPH); 
    Serial.println(SpeedRPH/60);    
    break;

    // actual data start from below
    case 4: // <Data 0>
    DistLowbyte = inbyte;
    break;
    case 5:
    DistHighbyte = inbyte ;
    processingData(0);   
    break;

    case 6: // quality
    break;
    case 7: // quality
    break;

    case 8: // <Data 1>
    DistLowbyte = inbyte;
    break;
    case 9:
    DistHighbyte = inbyte ;
    processingData(1);   
    break;

    case 10: // quality
    break;
    case 11: // quality
    break;

    case 12: // <Data 2>
    DistLowbyte = inbyte;
    break;
    case 13:
    DistHighbyte = inbyte ;
    processingData(2);   
    break;

    case 14: // quality
    break;
    case 15: // quality
    break;

    case 16: // <Data 3>
    DistLowbyte = inbyte;
    break;
    case 17:
    DistHighbyte = inbyte ;
    processingData(3);   
    break;

    case 18: // quality
    break;
    case 19: // quality
    break;
    
    

    default: // should implement checksum!! and send package
        break;
  }  
}

void processingData(int dataNr){
  byte check =DistHighbyte& 0x3f  // check <"invalid data" flag> <"strength warning" flag> 
  dist = (check<<8)| DistLowbyte;   
  // scale the range and save to array
  unsigned int cast =  dist / 1000 ;
  if (cast > 255){
    cast =255;
    }
  data [Data_4deg_index*4 + dataNr] = cast;  
  
  }

void SpeedControl ( int RPHinput) // simple close loop control
{
 if (Data_4deg_index%50==0) { /// used to control the rate of speed adjustment
  if (SpeedRPH<goalSpeedRPH)
     if (PWMduty<PWMdutyMax) PWMduty++; // limit the max PWM make sure it don't overflow 
  if (SpeedRPH>goalSpeedRPH)
     if(PWMduty>PWMdutyMin) PWMduty--;  //Have to limit the lowest pwm keep motor running       
  motor1.setSpeed(PWMduty);
 }
}
