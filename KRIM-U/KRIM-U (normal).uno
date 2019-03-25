//////////////////////////////////////////
//                KRIM-U                //
//               Versi A                //
//////////////////////////////////////////


//Variable
bool opponent(0), rightOpponent(0), button(0);
uint8_t turn(0);
uint16_t infrared0(0), infrared1(0), infrared2(0), infrared3(0);    //CEK LAGI NILAI MAKSIMUM
uint16_t distance(0);                                      //CEK LAGI NILAI MAKSIMUM

#define Motor__SPEED_LEFT 9
#define Motor__SPEED_RIGHT 10
#define Motor__LEFT_FORW 7
#define Motor__LEFT_BACK 8
#define Motor__RIGHT_FORW 11
#define Motor__RIGHT_BACK 12

#define Ultrasonic__CENTER_TRIGGER 4
#define Ultrasonic__CENTER_ECHO 2
#define Ultrasonic__RIGHT_TRIGGER 5
#define Ultrasonic__RIGHT_ECHO 3

#define Infrared__ECHO0 14
#define Infrared__ECHO1 15
#define Infrared__ECHO2 16
#define Infrared__ECHO3 6
#define Infrared__LIMIT 650   //white color (koreksi)

#define Battery__LEVEL 17

void setup()
{
  Serial.begin(2000000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(Motor__SPEED_LEFT, OUTPUT);
  pinMode(Motor__SPEED_RIGHT, OUTPUT);
  pinMode(Motor__LEFT_FORW, OUTPUT);
  pinMode(Motor__LEFT_BACK, OUTPUT);
  pinMode(Motor__RIGHT_FORW, OUTPUT);
  pinMode(Motor__RIGHT_BACK, OUTPUT);

  pinMode(Ultrasonic__CENTER_TRIGGER, OUTPUT);
  pinMode(Ultrasonic__CENTER_ECHO, INPUT);
  pinMode(Ultrasonic__RIGHT_TRIGGER, OUTPUT);
  pinMode(Ultrasonic__RIGHT_ECHO, INPUT);

  pinMode(Infrared__ECHO0, INPUT);
  pinMode(Infrared__ECHO1, INPUT);
  pinMode(Infrared__ECHO2, INPUT);
  pinMode(Infrared__ECHO3, INPUT);

  pinMode(Battery__LEVEL, INPUT);
}

uint16_t centerUltrasonic()
{
  digitalWrite(Ultrasonic__CENTER_TRIGGER, 0);  //Turn TRIGGER pin to LOW
  delayMicroseconds(2);
  digitalWrite(Ultrasonic__CENTER_TRIGGER, 1); //Turn TRIGGER pin to HIGH
  delayMicroseconds(10);
  digitalWrite(Ultrasonic__CENTER_TRIGGER, 0);  //Turn TRIGGER pin to LOW
  return ((pulseIn(Ultrasonic__CENTER_ECHO, 1) / 2) * 0.0343); //Timing from HIGH to LOW
}

bool rightUltrasonic()
{
  digitalWrite(Ultrasonic__RIGHT_TRIGGER, 0);  //Turn TRIGGER pin to LOW
  delayMicroseconds(2);
  digitalWrite(Ultrasonic__RIGHT_TRIGGER, 1); //Turn TRIGGER pin to HIGH
  delayMicroseconds(10);
  digitalWrite(Ultrasonic__RIGHT_TRIGGER, 0);  //Turn TRIGGER pin to LOW
  if(((pulseIn(Ultrasonic__RIGHT_ECHO, 1) / 2) * 0.0343) < 155) //Timing from HIGH to LOW
    return true;
  else
    return false;
}

void moveForw()
{
  rightOpponent = false;
  /*  Turn L&R BACK LOW
   *  Turn L&R FORW HIGH
   */
  digitalWrite(Motor__RIGHT_FORW, 1);
  digitalWrite(Motor__LEFT_FORW, 1);
  digitalWrite(Motor__RIGHT_BACK, 0);
  digitalWrite(Motor__LEFT_BACK, 0);
  analogWrite(Motor__SPEED_LEFT, 255);
  analogWrite(Motor__SPEED_RIGHT, 255);
}

void moveBack()
{
  digitalWrite(Motor__RIGHT_FORW, 0);
  digitalWrite(Motor__LEFT_FORW, 0);
  digitalWrite(Motor__RIGHT_BACK, 1);
  digitalWrite(Motor__LEFT_BACK, 1);
  analogWrite(Motor__SPEED_LEFT, 255);
  analogWrite(Motor__SPEED_RIGHT, 255);
}

void turnLeft()
{
  /*  Turn Left
   *  Turn L&R BACK & L FORW LOW
   *  Turn R FORW HIGH
   */
  rightOpponent = rightUltrasonic();  //find out opponent in the right side
  digitalWrite(Motor__RIGHT_FORW, 1);
  digitalWrite(Motor__LEFT_FORW, 0);
  digitalWrite(Motor__RIGHT_BACK, 0);
  digitalWrite(Motor__LEFT_BACK, 0);
  analogWrite(Motor__SPEED_LEFT, 0);
  analogWrite(Motor__SPEED_RIGHT, 255);
}

void turnRight()
{
  /*  Turn Right
   *  Turn L&R BACK & R FORW LOW
   *  Turn L FORW HIGH
   */
  digitalWrite(Motor__RIGHT_FORW, 0);
  digitalWrite(Motor__LEFT_FORW, 1);
  digitalWrite(Motor__RIGHT_BACK, 0);
  digitalWrite(Motor__LEFT_BACK, 0);
  analogWrite(Motor__SPEED_LEFT, 255);
  analogWrite(Motor__SPEED_RIGHT, 0);
}

void avoidLeft()
{
  /*  Turn L&R FORW LOW
   *  Turn l&R BACK HIGH
   */
  digitalWrite(Motor__RIGHT_FORW, 0);
  digitalWrite(Motor__LEFT_FORW, 0);
  digitalWrite(Motor__RIGHT_BACK, 1);
  digitalWrite(Motor__LEFT_BACK, 1);
  analogWrite(Motor__SPEED_LEFT, 255);
  analogWrite(Motor__SPEED_RIGHT, 127);
}

void avoidRight()
{
  /*  Turn L&R FORW LOW
   *  Turn l&R BACK HIGH
   */
  digitalWrite(Motor__RIGHT_FORW, 0);
  digitalWrite(Motor__LEFT_FORW, 0);
  digitalWrite(Motor__RIGHT_BACK, 1);
  digitalWrite(Motor__LEFT_BACK, 1);
  analogWrite(Motor__SPEED_LEFT, 127);
  analogWrite(Motor__SPEED_RIGHT, 255);
}

void loop()
{
  infrared0 = analogRead(Infrared__ECHO0);    //Left
  infrared1 = analogRead(Infrared__ECHO1);    //Back
  infrared2 = analogRead(Infrared__ECHO2);    //Right
  infrared3 = analogRead(Infrared__ECHO3);    //Front
  
  if(infrared0 > Infrared__LIMIT || infrared1 > Infrared__LIMIT || infrared2 > Infrared__LIMIT || infrared3 > Infrared__LIMIT)
  {
    distance = centerUltrasonic();
    if(distance < 155)
    {
      moveForw();
      Serial.println("MAJU");
    }
    else
    {
      if(rightOpponent == 0)
      {
        turnLeft();
        Serial.println("BELOK KIRI");
      }
      else
      {
        turnRight();
        Serial.println("BELOK KANAN");
      }
    }
  }
  else if(infrared3 <= Infrared__LIMIT)  //depan     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    moveBack();
    Serial.println("HINDAR DEPAN");
  }
  else if(infrared1 <= Infrared__LIMIT)  //belakang     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    moveForw();
    Serial.println("HINDAR BELAKANG");
  }
  else if(infrared0 <= Infrared__LIMIT)  //kiri     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    avoidLeft();
    Serial.println("HINDAR KIRI");
  }
  else  //kanan     PERLU KOREKSI. KATANYA PUTIH < 650
  {
    avoidRight();
    Serial.println("HINDAR KANAN");
  }
}
