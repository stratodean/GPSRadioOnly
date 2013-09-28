/*
STRATODEAN Three Tracker
 http://www.stratodean.co.uk
 (c) Mark Ireland
 
 August 2013 Version 1
 
 Thanks goes to all UKHAS authors that have contributed to this code.
 
 */
//-----------------------------------------------------------------------------------
//Setup includes and variables
#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <TinyGPS.h>

#define GPSLED A1                      //GPS LED pin (A because analogue pin)
#define RADIOLED A0                    //RADIO LED pin (A because analogue pin) 
#define RADIOPIN 9                     //RADIO pin
#define ASCII 7                        //AS CII mode (7 or 8)
#define STOPBITS 2                     //Stop Bits for transmission (1 or 2)
#define TXDELAY 0                      //Delay between transmissions
#define RTTY_BAUD 50                   //Baud rate for RTTY
#define ONE_SECOND F_CPU / 1024 / 16   //One second (clock speed / 1024 / 16)

TinyGPS tgps;

char callsign[6] = "MOD2";

unsigned long currentMillis;
long previousMillis = 0;

char txstring[100];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
unsigned int count=0; //Message counter
unsigned int countSaved=0; //Message saved counter

unsigned int alt; //Altitude
int sats; //Number of satellites
int GPSMode = 0;
char sLat[12] = "0"; //Latitude as string
char sLon[12] = "0"; //Longitude as string
float flat, flon;
unsigned long fixAge;
boolean fix = false;
int year;
byte month, day, hour, minute, second, hundredths;


//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
void setup()
{
  pinMode(GPSLED, OUTPUT);
  pinMode(RADIOLED, OUTPUT);
  pinMode(RADIOPIN, OUTPUT);
  Serial.begin(9600);
  flashLEDs(3);
  initialiseGPS();
  flashLEDs(2);
  initialiseInterrupt();
  flashLEDs(1);
}
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
void loop()
{
  //get new gps data
  getGPS();
  
}

//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
/*Start GPS*/
void initialiseGPS() {
  setGPSFlightMode();
  delay(500);
}

void getGPS(){

  Serial.flush();
  int i = 0;
  unsigned long start = millis();

  while ((i<60) && ((millis() - start) < 2000) ) 
  {
    if (Serial.available()) {
      char c = Serial.read();
      tgps.encode(c);
    }
  }



  tgps.f_get_position(&flat, &flon, &fixAge); //call tinyGPS for our position data

  //Setting LED if we have fix
  if (fixAge == TinyGPS::GPS_INVALID_AGE){
    digitalWrite(GPSLED,LOW);
    fix = false;    
  }   
  else if (fixAge > 4000)
  {
    digitalWrite(GPSLED,LOW);
    fix = false;
  }
  else
  {
    digitalWrite(GPSLED,HIGH);
    fix = true;
  }
  dtostrf(flat,9,6,sLat); //convert lat to string
  dtostrf(flon,9,6,sLon); //convert lon to string
  if (sLon[0] == ' ') //correct number of 0s in longitude string
  {
    sLon[0] = '0';
  }

  alt = tgps.f_altitude(); //call tinyGPS for our altitude data
  sats = tgps.satellites(); //call tinyGPS for satellite count
  tgps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fixAge); //call tinyGSPS for time data

}


void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  delay(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

void setGPSFlightMode()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                                                                         };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
  GPSMode = 6;
}

boolean getUBX_ACK(uint8_t *MSG)
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 4; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

/*End GPS*/
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
/*Start Radio*/
void initialiseInterrupt()
{
  //We are using timer2 as servo library uses timer1
  //This might not cause a problem, but better to be safe!
  cli();          //disable global interrupts
  TCCR1A = 0;     //set entire TCCR2A register to 0
  TCCR1B = 0;     //same for TCCR2B
  //OCR1A = 15624;
  OCR1A = F_CPU / 1024 / RTTY_BAUD;  //set compare match register to desired timer count  
  TCCR1B |= (1 << WGM12); //turn on CTC mode
  TCCR1B |= (1 << CS10); //Set CS10 and CS12 bits for prescaler
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A); //enable timer compare interrupt
  sei(); //enable global interrupts
}

ISR(TIMER1_COMPA_vect)
{

  switch(txstatus){
  case 0: //Optional delay
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)){
      txj=0;
      txstatus=1;
    }
    break;
  case 1: //Initalise transmission

    //create string to send
    sprintf(txstring, "$$$$%s,%i,%02d:%02d:%02d,%s,%s,%i,%i,%i",callsign,count,hour,minute,second,sLat,sLon,alt,fix,sats);

    //add checksum
    sprintf(txstring, "%s*%04X\n",txstring,gps_CRC16_checksum(txstring));
    //increment count
    count++;

    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;

    break;

  case 2: //Grab a char and transmit
    if (txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      sendBit(0); //start bit
      txi=0;
    }
    else
    {
      txstatus=0;
      txj=0; 
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) sendBit(1);
      else sendBit(0);  
      txc = txc >> 1;
      break;
    }
    else
    {
      sendBit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    }

  case 4: //End of transmission
    if (STOPBITS==2)
    {
      sendBit(1); //Stop Bit
      txstatus=2; //back to the start
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
  }
}

void sendBit (int bit){
  if (bit)
  {
    //high
    digitalWrite(RADIOPIN, HIGH);
    digitalWrite(RADIOLED, HIGH);
  }
  else
  {
    //low
    digitalWrite(RADIOPIN, LOW);
    digitalWrite(RADIOLED, LOW);
  }

}


/*End Radio*/
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
//Functions
void flashLEDs(int flashes)
{
  for(int flashno = 0; flashno < flashes; flashno++)
  {
    digitalWrite(GPSLED, HIGH);
    //digitalWrite(RADIOLED, HIGH);
    delay(200);
    digitalWrite(GPSLED, LOW);
    //digitalWrite(RADIOLED, LOW);
    delay(100);
  }
  delay(200);
}

