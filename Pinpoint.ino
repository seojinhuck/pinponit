#include <waypointClass.h>
#include <Servo.h>  
#include <Wire.h>    
#include <Adafruit_Sensor.h>  
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS.h>  
#include <SoftwareSerial.h>     
#include <math.h> 

// Adafruit Magnetometer Adafruit_HMC5883 = I2C :  SCL (A5) &  SDA (A4)

byte servoPin = 8;
byte servoPin2 = 9;

Servo esc_motor;

Servo esc_motor2;
int motor2;

#define GPSBAUD 9600
#define RXPin 4
#define TXPin 3

TinyGPS gps;
SoftwareSerial uart_gps(RXPin, TXPin);
SoftwareSerial ss(4, 3);

// Setup magnemeter  (compass); uses I2C  설정 자력계(나침반); I2C 사용
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
sensors_event_t compass_event;


void getgps(TinyGPS &gps);



// Compass navigation 나침반 탐색
int targetHeading;              // where we want to go to reach current waypoint 현재 웨이포인트에 도달하기 위해 가고자 하는 곳 
int currentHeading;             // where we are actually facing now 우리가 지금 실제로 직면하고 있는 곳 
int headingError;               //  targetHeading과 currentHeading 간의 부호 있는(+/-) 차이 
#define HEADING_TOLERANCE 5     //  targetHeading을 가로채기 위해 회전을 시도하지 않는 허용 오차 +/-(도 단위) 



// Waypoints
#define WAYPOINT_DIST_TOLERANE  5   //  웨이포인트까지의 허용 오차(미터); 이 허용 범위 내에서 다음 웨이포인트로 이동합니다.
#define NUMBER_WAYPOINTS 5          //  여기에 웨이 포인트의 숫자를 입력하십시오(0에서 (n-1)까지 실행됨)
int waypointNumber = -1;            // 현재 웨이포인트 번호; 0에서 (NUMBER_WAYPOINTS -1)까지 실행됩니다. -1에서 시작하고 setup() 중에 초기화됩니다.
waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(30.508302, -97.832624), waypointClass(30.508085, -97.832494), waypointClass(30.507715, -97.832357), waypointClass(30.508422, -97.832760), waypointClass(30.508518,-97.832665) };



// Speeds (range: 0 - 255) / 속도(범위: 1100 - 1900) 
#define FAST_SPEED 1900
#define NORMAL_SPEED 1700
#define TURN_SPEED 1600
#define SLOW_SPEED 1600

int speed = NORMAL_SPEED;

void setup() 
{
  
  // Set the speed to start, from 0 (off) to 255 (max speed) / 시작할 속도를 1500(꺼짐)에서 1900(최대 속도)까지 설정합니다. esc 변경
  
  esc_motor.attach(servoPin);
  esc_motor2.attach(servoPin2);
  esc_motor.writeMicroseconds(1500);
  esc_motor2.writeMicroseconds(1500);
 
  Serial.begin(9600);
  

  Serial.println("Program begin...");

  Serial.begin(9600);
  ss.begin(9600);
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();

   

   
  
  // start Mag / Compass // 매그 / 나침반 시작
  if(!compass.begin())
    {
      #ifdef DEBUG
        Serial.println(F("COMPASS ERROR"));
      #endif
    }
}

    void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    delay(100);
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}

    //  새로운 GPS 데이터가 수신된 후 호출됩니다. 우리의 위치와 코스/웨이포인트까지의 거리를 업데이트합니다.

       
  
   // processGPS(void)
    
int readCompass(void)
{
  compass.getEvent(&compass_event);    
  float heading = atan2(compass_event.magnetic.y, compass_event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location. / 방향이 정해지면 해당 위치의 자기장 '오차'인 '적위각'을 추가해야 합니다.
  // Find yours here: http://www.magnetic-declination.com/ 
  // Cedar Park, TX: Magnetic declination: 4° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians / 텍사스 주 시더 파크: 자기 편각: 4° 11' EAST(양수); 1도e = 0.0174532925라디안
  
  #define DEC_ANGLE 0.069
  heading += DEC_ANGLE;
  
  // Correct for when signs are reversed. / 기호가 반전된 경우에 적합합니다.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination. / 편각 추가로 인한 랩을 확인하십시오.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability. / 가독성을 위해 라디안을 각도로 변환합니다.
  float headingDegrees = heading * 180/M_PI; 
  
  return ((int)headingDegrees); 
}  // readCompass()


void calcDesiredTurn(void)
{
    // calculate where we need to turn to head to destination / 목적지로 향하기 위해 방향을 전환해야 하는 위치를 계산합니다.
    headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap / 나침반 랩 조정
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading / targetHeading을 가로채기 위해 회전할 방향을 계산합니다. 회전 변경
    if (abs(headingError) <= HEADING_TOLERANCE) {     // if within tolerance, don't turn / 허용 범위 내에 있으면 회전하지 마십시오.
         esc_motor.writeMicroseconds(1700);
         esc_motor2.writeMicroseconds(1700);}
        
    else if (headingError < 0) { //left
        esc_motor.writeMicroseconds(1300);
        esc_motor2.writeMicroseconds(1700);}
        
      
    else if (headingError > 0) { //right
        esc_motor.writeMicroseconds(1700);
        esc_motor2.writeMicroseconds(1300);}
    
    else {
      esc_motor.writeMicroseconds(1700);
      esc_motor2.writeMicroseconds(1700);}
      
      
} 

  
      
  
















  
