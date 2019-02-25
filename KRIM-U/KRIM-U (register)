//////////////////////////////////////////
//                KRIM-U                //
//               Versi B                //
//////////////////////////////////////////

#define SPEED_LEFT OCR1A  //PWM  NA B 9   timer1  A 16-bit
#define SPEED_RIGHT OCR1B //PWM  NB B 10  timer1  B 16-bit


//VARIABLE
bool opponentDistance(0), opponent(0), button(0), ultrasonicState(0), turn(0), state(0);
uint8_t rearDistance(0);
uint16_t infrared0, infrared1, infrared2, infrared3;    //CEK LAGI NILAI MAKSIMUM
uint32_t risingTime(0), fallingTime(0), rearLimit(0);


//FIXED VARIABLE
#define Front__LIMIT  155   //CEK LAGI NILAI MAKSIMUM
#define Rear__LIMIT         75    //NAH INI BERAPAAAA? perlu coba di lapangan
#define Infrared__LIMIT 650       //white color         PERLU KOREKSI. KATANYA PUTIH < 650


//PIN
#define Motor__LEFT         0x02  //PWM  NA B 9   timer1  A 16-bit    PAY ATTENTION TO OCR1A TCCR1A
#define Motor__RIGHT        0x04  //PWM  NB B 10  timer1  B 16-bit    PAY ATTENTION TO OCRAB TCCR1B
#define Motor__RIGHT_FORW   0x01  //N3 B 8
#define Motor__RIGHT_BACK   0x08  //N4 B 11

#define Motor__LEFT_FORW    0x40  //N1 D 6
#define Motor__LEFT_BACK    0x80  //N2 D 7
#define Ultrasonic__CENTER_TRIGGER  0x04  //output D 2
#define Ultrasonic__CENTER_ECHO     0x08  //input  D 3
#define Ultrasonic__REAR_TRIGGER   0x10  //output D 4
#define Ultrasonic__REAR_ECHO      0x20  //input  D 5

#define Infrared__ECHO0 0x01  //input C A0  14  Left
#define Infrared__ECHO1 0x02  //input C A1  15  Back
#define Infrared__ECHO2 0x04  //input C A2  16  Right
#define Infrared__ECHO3 0x08  //input C A3  17  Front


void setup()
{
  Serial.begin(2000000);
  
  // set prescale to 16
  _SFR_BYTE(ADCSRA) |= _BV(ADPS2);  //Setting register bits   (1)
  _SFR_BYTE(ADCSRA) &= ~_BV(ADPS1); //Clearing register bits  (0)
  _SFR_BYTE(ADCSRA) &= ~_BV(ADPS0); //Clearing register bits  (0)

  /*  I/O for Motor Speed & Direction
   *  I/O for Ultrasonic Sensor
   *  Input for Infrared Sensor
   */
  DDRB |= (Motor__LEFT | Motor__RIGHT | Motor__RIGHT_FORW | Motor__RIGHT_BACK);
  DDRD |= (Motor__LEFT_FORW | Motor__LEFT_BACK | (Ultrasonic__CENTER_TRIGGER & ~Ultrasonic__CENTER_ECHO) | (Ultrasonic__REAR_TRIGGER & ~Ultrasonic__REAR_ECHO));
  DDRC &= ~(Infrared__ECHO0 | Infrared__ECHO1 | Infrared__ECHO2 | Infrared__ECHO3);

  /* Change TCCR0B affects millis() and delay().
   * - Pins 5 and 6: controlled by Timer 0
   * - Pins 9 and 10: controlled by timer 1
   * - Pins 11 and 3: controlled by timer 2
   */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); //Set freq pin 9 (LEFT MOTOR)
  TCCR1B = _BV(CS10);                                           //Set freq pin 10 (RIGHT MOTOR)

  SPEED_LEFT = 0;
  SPEED_RIGHT = 0;
}


//GENERATE RANDOM NUMBER
/*  Modified from 128-bit xorshift+ by Sebastiano Vigna
 *  Xorshift originally discovered by George Marsaglia in 2003
 *  100 random numbers, 41 numbers are 0 and 59 numbers are 1
 */

uint8_t seed[2] = {28, 21};

uint8_t Zrand()
{
  uint8_t x = seed[0];
  uint8_t const y = seed[1];
  seed[0] = y;
  x ^= x << 3;  //a
  seed[1] = x ^ y ^ (x >> 2) ^ (y >> 4); //b, c
  return ((seed[1] + y) & 1);
}

uint16_t centerUltrasonic()
{
  PORTD &= ~Ultrasonic__CENTER_TRIGGER;                 //Turn TRIGGER pin to LOW
  do
  {
    ultrasonicState = PIND & Ultrasonic__CENTER_ECHO;
  }while(ultrasonicState);                              //Wait until TRIGGER turn LOW
  
  PORTD |= Ultrasonic__CENTER_TRIGGER;                  //Turn TRIGGER pin to HIGH
  do
  {
    ultrasonicState = PIND & Ultrasonic__CENTER_ECHO;
  }while(!(ultrasonicState));                           //Wait until TRIGGER turn HIGH
  risingTime = micros();                                //Time of the rising edge
  
  PORTD &= ~Ultrasonic__CENTER_TRIGGER;                 //Turn TRIGGER pin to LOW
  do
  {
    ultrasonicState = PIND & Ultrasonic__CENTER_ECHO;
  }while(ultrasonicState);                              //Wait until TRIGGER turn LOW
  fallingTime = micros();                               //Time of the falling edge

  return (((fallingTime - risingTime) / 2) * 0.0343);   //Timing from HIGH to LOW
}

uint8_t rearUltrasonic()
{
  PORTD &= ~Ultrasonic__REAR_TRIGGER;                  //Turn TRIGGER pin to LOW
  do
  {
    ultrasonicState = PIND & Ultrasonic__REAR_ECHO;
  }while(ultrasonicState);                              //Wait until TRIGGER turn LOW
  
  PORTD |= Ultrasonic__REAR_TRIGGER;                   //Turn TRIGGER pin to HIGH
  do
  {
    ultrasonicState = PIND & Ultrasonic__REAR_ECHO;
  }while(!(ultrasonicState));                           //Wait until TRIGGER turn HIGH
  risingTime = micros();                                //Time of the rising edge
  
  PORTD &= ~Ultrasonic__REAR_TRIGGER;                   //Turn TRIGGER pin to LOW
  do
  {
    ultrasonicState = PIND & Ultrasonic__REAR_ECHO;
  }while(ultrasonicState);                              //Wait until TRIGGER turn LOW
  fallingTime = micros();                               //Time of the falling edge
  rearLimit = (((fallingTime - risingTime) / 2) * 0.0343);      //Timing from HIGH to LOW
  if(rearLimit < (Rear__LIMIT + 5) || rearLimit > Rear__LIMIT)  //Aman
    return 1;
  else if (rearLimit < Rear__LIMIT)   //Musuh di belakang
    return 0;
  else                                //Hampir jatuh
    return 2;
}

void forward()
{
  /*  Go a Head
   *  Turn L&R BACK LOW
   *  Turn L&R FORW HIGH
   */
  PORTB &= ~Motor__RIGHT_BACK;
  PORTD &= ~Motor__LEFT_BACK;
  PORTB |= Motor__RIGHT_FORW;
  PORTD |= Motor__LEFT_FORW;
  SPEED_LEFT = 255;
  SPEED_RIGHT = 255; 
}

void backward()
{
  /*  Go Backward
   *  Turn L&R BACK LOW
   *  Turn L&R FORW HIGH
   */
  PORTB &= ~Motor__RIGHT_FORW;
  PORTD &= ~Motor__LEFT_FORW;
  PORTB |= Motor__RIGHT_BACK;
  PORTD |= Motor__LEFT_BACK;
  SPEED_LEFT = 255;
  SPEED_RIGHT = 255; 
}

void backLeft()
{
  /*  Turn Back Left
   *  Turn L&R BACK HIGH
   *  Turn L&R FORW LOW
   */
  PORTB &= ~Motor__RIGHT_FORW;
  PORTD &= ~Motor__LEFT_FORW;
  PORTB |= Motor__RIGHT_BACK;
  PORTD |= Motor__LEFT_BACK;
  SPEED_LEFT = 127;
  SPEED_RIGHT = 255;
}

void backRight()
{
  /*  Turn Back Right
   *  Turn L&R BACK HIGH
   *  Turn L&R FORW LOW
   */
  PORTB &= ~Motor__RIGHT_FORW;
  PORTD &= ~Motor__LEFT_FORW;
  PORTB |= Motor__RIGHT_BACK;
  PORTD |= Motor__LEFT_BACK;
  SPEED_LEFT = 255;
  SPEED_RIGHT = 127;
}

void forwLeft()
{
  /*  Turn Forw Left
   *  Turn L&R FORW HIGH
   *  Turn L&R BACK LOW
   */
  PORTB &= ~Motor__RIGHT_BACK;
  PORTD &= ~Motor__LEFT_BACK;
  PORTB |= Motor__RIGHT_FORW;
  PORTD |= Motor__LEFT_FORW;
  SPEED_LEFT = 255;
  SPEED_RIGHT = 127;
}

void forwRight()
{
  /*  Turn Forw Right
   *  Turn L&R FORW HIGH
   *  Turn L&R BACK LOW
   */
  PORTB &= ~Motor__RIGHT_BACK;
  PORTD &= ~Motor__LEFT_BACK;
  PORTB |= Motor__RIGHT_FORW;
  PORTD |= Motor__LEFT_FORW;
  SPEED_LEFT = 255;
  SPEED_RIGHT = 127;
}

void loop()
{
  infrared0 = analogRead(Infrared__ECHO0);    //Left
  infrared1 = analogRead(Infrared__ECHO1);    //Back
  infrared2 = analogRead(Infrared__ECHO2);    //Right
  infrared3 = analogRead(Infrared__ECHO3);    //Front


  if(infrared0 > Infrared__LIMIT || infrared1 > Infrared__LIMIT || infrared2 > Infrared__LIMIT || infrared3 > Infrared__LIMIT)
  {
    opponentDistance = centerUltrasonic();
    rearDistance = rearUltrasonic();
    if(opponentDistance < Front__LIMIT && rearDistance)
    {
      state = 0;
      forward();
      Serial.println(opponentDistance + "\tMAJU");
    }
    else if(rearDistance >> 1)  //Hampir jatuh
    {
      //  When the motor is weaker than the opponent's
      if(!state)
        turn = Zrand();
      if(turn)
      {
        backRight();
        Serial.println("HINDAR MUNDUR KANAN");
      }
      else
      {
        backLeft();
        Serial.println("HINDAR MUNDUR KIRI");
      }
    }
    else if(!(rearDistance >> 2))   //Musuh di belakang
    {
      if(!state)
        turn = Zrand();
      if(turn)
      {
        forwRight();
        Serial.println("MUSUH BELAKANG, MAJU KANAN");
      }
      else
      {
        forwLeft();
        Serial.println("MUSUH BELAKANG, MAJU KIRI");
      }
    }
    else
    {
      if(!state)
        turn = Zrand();
      if(turn)
      {
        forwLeft();
        Serial.println("CARI KIRI");
      }
      else
      {
        forwLeft();
        Serial.println("CARI KANAN");
      }
    }
  }
  else if(infrared3 <= Infrared__LIMIT)  //depan     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    backward();
    Serial.println("TEPI DEPAN");
    delay(2500);
  }
  else if(infrared1 <= Infrared__LIMIT)  //belakang     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    forward();
    Serial.println("TEPI BELAKANG");
    delay(2500);
  }
  else if(infrared0 <= Infrared__LIMIT)  //kiri     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    backRight();
    Serial.println("TEPI KIRI");
    delay(2500);
  }
  else  //kanan     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    backLeft();
    Serial.println("TEPI KANAN");
    delay(2500);
  }
}
