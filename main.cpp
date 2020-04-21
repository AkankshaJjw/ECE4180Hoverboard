#include "mbed.h"
#include "wave_player.h"
#include "SDFileSystem.h"
#include "Motor.h"
#include "RGBLed.h"
#include "Servo.h"
 
 
Motor motorBottom(p26, p11, p12); // pwm, fwd, rev
Servo servoBack(p25); // pwm
 
// Setup RGB led using PWM pins and class
RGBLed myRGBled(p22,p23,p24); //red, green, blue PWM pins
 
// Setup to play wav file from SD Card
AnalogOut DACout(p18);
wave_player waver(&DACout);
SDFileSystem sd(p5, p6, p7, p8, "sd"); //SD card
 
// Setup for bluetooth
Serial blue(p28,p27);
BusOut myled(LED1,LED2,LED3,LED4);
 
//setting any unused analog input pins to digital outputs reduces A/D noise
DigitalOut P16(p16);
DigitalOut P17(p17);
DigitalOut P19(p19);
DigitalOut P20(p20);
 
void playSound(char * wav)
{
    FILE *wave_file;
    wave_file = fopen(wav,"r");
    waver.play(wave_file);
    fclose(wave_file);
}
 
 
int main()
{
    float motorSpeed = 0.5; 
    float servoAngle = 90.0; //ranges from 0 to 180 degrees where 90 is center
    myRGBled.write(1.0,1.0,1.0); //white
    char bnum = 0;
    char bhit = 0;
    while(1) 
    {
        motorBottom.speed(motorSpeed);
        servoBack.position(servoAngle);
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
                        case '3': //number button 3: turn on/off
                            if (bhit=='1') {
                                //add hit code here    
                                if (motorSpeed == 0.0) {
                                    motorSpeed = 0.5; //if off turn motor on
                                    myRGBled.write(1.0,1.0,1.0); //white
                                } else {
                                    motorSpeed = 0.0; //if on turn motor off
                                    myRGBled.write(1.0,0.0,0.0); //red
                                }
                            } else {
                                //add release code here
                            }
                            break;
                        case '4': //number button 4: can be used to play sounds
                            if (bhit=='1') {
                                //add hit code here                              
                                playSound(); //write name of sound file
                            } else {
                                //add release code here
                            }
                            break;
                        case '5': //button 5 up arrow: not used currently
                            if (bhit=='1') {
                                //add hit code here                               
                            } else {
                                //add release code here
                            }
                            break;
                        case '6': //button 6 down arrow: not used currently
                            if (bhit=='1') {
                                //add hit code here
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