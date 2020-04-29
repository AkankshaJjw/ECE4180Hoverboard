#include "mbed.h"
#include "wave_player.h"
#include "SDFileSystem.h"
#include "Servo.h"
#include "ultrasonic.h"
#include "uLCD_4DGL.h"
#include "LSM9DS1.h"

#define PI 3.14159
#define DECLINATION -4.94 // Declination (degrees) in Atlanta,GA.
int global_running = 1; //1 = display distance, 0 = avoid colission
float range = 0.0005; // for servo/motors calibration
float position = 0.5; // for servo/motors calibration
double x2;
double y2;
int x_c;
int y_c;
int r;
int z;

Serial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p13,p14,p11); // serial tx, serial rx, reset pin;
DigitalOut led1(p26);
DigitalOut led2(p19);
LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);

Servo dcmotorBottom(p21); // pwm
Servo servoBottom(p22); // pwm
Servo dcmotorBack(p23); // pwm

// Setup to play wav file from SD Card
AnalogOut DACout(p18);
wave_player waver(&DACout);
SDFileSystem sd(p5, p6, p7, p8, "sd"); //SD card
 
// Setup for bluetooth
Serial blue(p28,p27);
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

void col_ESC() {
    dcmotorBottom = 0.0;
    dcmotorBack = 0.0;
    wait(0.5); //ESC detects signal
    //Required ESC Calibration/Arming sequence  
    //sends longest and shortest PWM pulse to learn and arm at power on
    dcmotorBottom = 1.0; //send longest PWM
    dcmotorBack = 1.0; //send longest PWM
    wait(8);
    dcmotorBottom = 0.0; //send shortest PWM
    dcmotorBack = 0.0; //send shortest PWM
    wait(8);
}


void dist(int distance) 
{   //display distance
    //put code here to happen when the distance is changed
    printf("Distance changed to %dmm\r\n", distance); //print to pc for debugging
    uLCD.cls();
    uLCD.printf("Distance changed to %dmm\r\n", distance); //print to uLCD
    
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



int main()
{   
    
    //calibrate motors
    servoBottom.calibrate(range, position);
    col_ESC();
    
    Ticker sonar_int;
    
    char bnum = 0;
    char bhit = 0;
    
    //open wav file and play it
    FILE *wave_file;
    printf("\n\n\nHello, wave world!\n");
    wave_file=fopen("/sd/honk.wav","r");
    
    while(1) 
    {   
        // If the motor is stopped and there are no collisions, start the motor
        if (global_running && dcmotorBottom == 0.0){
            for (float p=0.0; p<=0.9; p += 0.05) { //Throttle up slowly to full throttle
                dcmotorBottom = p;
                wait(1.0);
            }
        }
        if (global_running){
            // Read bluetooth controller input
            if (blue.getc() == '!') {
                if (blue.getc() == 'B') { //button data packet
                    bnum = blue.getc(); //button number
                    bhit = blue.getc(); //1=hit, 0=release
                    if (blue.getc() == char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                        switch (bnum) {
                            case '1': //number button 1: not used currently
                                if (bhit=='1') {
                                    //add hit code here
                                } else {
                                    //add release code here
                                }
                                break;
                            case '2': //number button 2: not used currently
                                if (bhit=='1') {
                                    //add hit code here
                                } else {
                                    //add release code here
                                }
                                break;
                            case '3': //number button 3: display distance from sonar
                                if (bhit=='1') {
                                } else {
                                    //add release code here
                                }
                                break;
                            case '4': //number button 4: play honking sound
                                if (bhit=='1') {
                                    //add hit code here    
                                    waver.play(wave_file);                          
                                } else {
                                    //add release code here
                                }
                                break;
                            case '5': //button 5 up arrow: turn on bottom servo and back motor
                                if (bhit=='1') {
                                    //add hit code here  
                                    for (float p=0.0; p<=0.3; p += 0.025) { //Throttle up slowly to full throttle
                                        dcmotorBack = p;
                                        wait(1.0);
                                    }
                                                           
                                } else {
                                    //add release code here
                                    dcmotorBack = 0.0;
                                }
                                break;
                            case '6': //button 6 down arrow: switch it off (turn off back motor)
                                if (bhit=='1') {
                                    
                                } else {
                                    //add release code here
                                }
                                break;
                            case '7': //button 7 left arrow: turns servo leftwards
                                if (bhit=='1') {
                                   servoBottom = 1.0;
                                } else {
                                    //add release code here
                                    servoBottom = 0.5;
                                }
                                break;
                            case '8': //button 8 right arrow: turns servo rightwards
                                if (bhit=='1') {
                                    servoBottom = 0.0;
                                } else {
                                    //add release code here
                                    servoBottom = 0.5;
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
}