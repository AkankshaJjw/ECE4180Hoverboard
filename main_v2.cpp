#include "mbed.h"
#include "wave_player.h"
#include "SDFileSystem.h"
#include "Motor.h"
#include "RGBLed.h"
#include "Servo.h"
#include "ultrasonic.h"
#include "uLCD_4DGL.h"
#include "LSM9DS1.h"

#define PI 3.14159
#define DECLINATION -4.94 // Declination (degrees) in Atlanta,GA.
int sonar_flag = 0; //1 = display distance, 0 = avoid colission
float range = 0.0005; // for servo/motors calibration
float position = 0.5; // for servo/motors calibration

Serial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p13,p14,p11); // serial tx, serial rx, reset pin;
ultrasonic mu(p15, p16, .1, 1, &dist); //sonar
DigitalOut led1(p18);
DigitalOut led2(p19);

Servo dcmotorBottom(p21); // pwm
Servo servoBottom(p22); // pwm
Servo dcmotorBack(p23); // pwm

// Setup to play wav file from SD Card
AnalogOut DACout(p26);
wave_player waver(&DACout);
SDFileSystem sd(p5, p6, p7, p8, "sd"); //SD card
 
// Setup for bluetooth
Serial blue(p28,p27);
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

void playSound(char * wav)
{
    FILE *wave_file;
    wave_file = fopen(wav,"r");
    waver.play(wave_file);
    fclose(wave_file);
}

void bottom_motor_on()
{
    for (float i=0; i<=0.8; i+=0.1) {
        dcmotorBottom = i;
        wait(0.1);
    }
}

void dist(int distance) 
{   if(sonar_flag ==1) { //display distance
    //put code here to happen when the distance is changed
    printf("Distance changed to %dmm\r\n", distance); //print to pc for debugging
    uLCD.cls();
    uLCD.printf("Distance changed to %dmm\r\n", distance); //print to uLCD
    wait(2);
    } else { //avoid colision: beep and stop motor
        if(distance < 2){   
        playSound("/sd/Beeping.wav")
        dcmotorBack = 0.0;
        }
    }
    
}

float printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
// touchy trig stuff to use arctan to get compass heading (scale is 0..360)
    mx = -mx;
    float heading;
    if (my == 0.0)
        heading = (mx < 0.0) ? 180.0 : 0.0;
    else
        heading = atan2(mx, my)*360.0/(2.0*PI);
    //pc.printf("heading atan=%f \n\r",heading);
    heading -= DECLINATION; //correct for geo location
    if(heading>180.0) heading = heading - 360.0;
    else if(heading<-180.0) heading = 360.0 + heading;
    else if(heading<0.0) heading = 360.0  + heading;


    // Convert everything from radians to degrees:
    //heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    pc.printf("Pitch: %f,    Roll: %f degress\n\r",pitch,roll);
    pc.printf("Magnetic Heading: %f degress\n\r",heading);
    return heading;
}

void display_compass()
{   
    while(!IMU.tempAvailable());
    IMU.readTemp();
    while(!IMU.magAvailable(X_AXIS));
    IMU.readMag();
    while(!IMU.accelAvailable());
    IMU.readAccel();
    while(!IMU.gyroAvailable());
    IMU.readGyro();
    pc.printf("\nIMU Temperature = %f C\n\r",25.0 + IMU.temperature/16.0);
    pc.printf("        X axis    Y axis    Z axis\n\r");
    pc.printf("gyro:  %9f %9f %9f in deg/s\n\r", IMU.calcGyro(IMU.gx), IMU.calcGyro(IMU.gy), IMU.calcGyro(IMU.gz));
    pc.printf("accel: %9f %9f %9f in Gs\n\r", IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), IMU.calcAccel(IMU.az));
    pc.printf("mag:   %9f %9f %9f in gauss\n\r", IMU.calcMag(IMU.mx), IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz));
    float heading = printAttitude(IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), IMU.calcAccel(IMU.az), IMU.calcMag(IMU.mx),
                    IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz));
    double a = heading * 0.0174533; //deg to rad
    x2 = x_c + r*cos(a);
    y2 = y_c + r*sin(a);
    uLCD.cls();
    uLCD.circle(x_c, y_c, r, WHITE);
    uLCD.line(x_c, y_c, (int)x2, (int)y2, BLUE);
    wait(0.001);
}


int main()
{   
    // this part of the display compass script only needs to be set up once
    int x_c = 60;
    int y_c = 60;
    int r = 30;
    uLCD.circle(x_c, y_c, r, WHITE);
    int z = 0;
    double x2 = x_c + r*cos((double)z);
    double y2 = x_c + r*sin((double)z);
    uLCD.line(x_c, y_c, (int)x2, (int)y2, BLUE);
    LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    IMU.calibrate(1);
    IMU.calibrateMag(0);

    //calibrate motors
    dcmotorBack.calibrate(range, position);
    dcmotorBottom.calibrate(range, position);
    servoBottom.calibrate(range, position)
    
    bottom_motor_on(); // bottom motor always on
    float servoAngle = 90.0; // ranges from 0 to 180 degrees where 90 is center
    mu.startUpdates(); // start mesuring the distance
    char bnum = 0;
    char bhit = 0;
    while(1) 
    {   
        sonar_flag = 0;
        mu.checkDistance();
        display_compass(); // this part of the compass script needs to be done repeatedly
        servoBottom.position(servoAngle);
        if (blue.getc() == '!') {
            if (blue.getc() == 'B') { //button data packet
                bnum = blue.getc(); //button number
                bhit = blue.getc(); //1=hit, 0=release
                if (blue.getc() == char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                    myled = bnum - '1'; //current button number will appear on LEDs
                    switch (bnum) {
                        case '1': //number button 1: speeds up motor
                            if (bhit=='1') {
                                //add hit code here
                                motorSpeed += 0.2; // speed up
                                myRGBled.write(0.0,1.0,0.0); //green
                            } else {
                                //add release code here
                            }
                            break;
                        case '2': //number button 2: slows down motor
                            if (bhit=='1') {
                                //add hit code here
                                motorSpeed -= 0.2; // speed down
                                myRGBled.write(0.0,0.0,1.0); //blue
                            } else {
                                //add release code here
                            }
                            break;
                        case '3': //number button 3: display distance from sonar
                            if (bhit=='1') {
                                //add hit code here   
                                sonar_flag = 1; 
                                mu.checkDistance();
                            } else {
                                //add release code here
                            }
                            break;
                        case '4': //number button 4: play honking sound
                            if (bhit=='1') {
                                //add hit code here                              
                                playSound("/sd/Honk.wav"); //write name of sound file
                            } else {
                                //add release code here
                            }
                            break;
                        case '5': //button 5 up arrow: turn on bottom servo and back motor
                            if (bhit=='1') {
                                //add hit code here  
                                for(float p=0; p<1.0; p += 0.1) {
                                    dcmotorBack = p;
                                    wait(0.2);
                                }
                                myled1 = 1;                             
                            } else {
                                //add release code here
                            }
                            break;
                        case '6': //button 6 down arrow: switch it off (turn off back motor)
                            if (bhit=='1') {
                                //add hit code here
                                dcmotorBack = 0.0;
                                myled1 = 0;
                                myled2 = 1;
                            } else {
                                //add release code here
                            }
                            break;
                        case '7': //button 7 left arrow: turns servo leftwards
                            if (bhit=='1') {
                                //add hit code here
                                if (servoAngle > 0.0) {
                                    servoAngle -= 30; //turn servo left
                                }
                                myled3 = 1;
                                myled4 = 0;
                            } else {
                                //add release code here
                            }
                            break;
                        case '8': //button 8 right arrow: turns servo rightwards
                            if (bhit=='1') {
                                //add hit code here
                                if (servoAngle < 180.0) {
                                    servoAngle += 30.0; //turn servo right
                                }
                                myled4 = 1;
                                myled3 = 0;
                            } else {
                                //add release code here
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}