#include "mbed.h"
#include "wave_player.h"
#include "SDFileSystem.h"
#include "Servo.h"
#include "ultrasonic.h"
#include "uLCD_4DGL.h"
#include "LSM9DS1.h"

volatile bool global_running = true; //1 = display distance, 0 = avoid collision
float range = 0.0005; // for servo/motors calibration
float position = 0.5; // for servo/motors calibration
unsigned int frame_limiter = 0; // Limits the framerate of the lcd

Serial pc(USBTX, USBRX);
uLCD_4DGL uLCD(p13,p14,p11); // serial tx, serial rx, reset pin;
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
DigitalOut exled1(p19);
DigitalOut exled1(p26);


// Setup for IMU
#define PI 3.14159
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -4.94*PI/180 // Declination (radians) in Atlanta,GA.
// Calculate heading.
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
float calculateHeading(float mx, float my)
{
// touchy trig stuff to use arctan to get compass heading
    mx = -mx;
    float heading;
    if (my == 0.0)
        heading = (mx < 0.0) ? PI : 0.0;
    else
        heading = atan2(mx, my);
    //pc.printf("heading atan=%f \n\r",heading);
    heading -= DECLINATION; //correct for geo location
    if(heading>PI) heading = heading - 2*PI;
    else if(heading<-PI) heading = 2*PI + heading;
    else if(heading<0.0) heading = 2*PI + heading;
    return heading;
}

// ESC/ Brushless DC Motor setup
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
{   
    // Set global running flag based on distance to sonar
    if(distance < 100){
        global_running = false;
    }
    else{
        if (global_running == false){
            global_running = true;
            // Wait 2 seconds before starting again
            wait(2);
        }
    }
    // If there is an obstruction, print the distance until it is clear
    if(!global_running){
        // Only update once every 4 loops
        if (frame_limiter % 4 == 0){
            uLCD.cls();
            uLCD.printf("Distance:\r\n%dmm\r\n", distance); //print to uLCD
        }
        frame_limiter++;
    }

    
}
ultrasonic mu(p15, p16, .1, 1, &dist);
//have updates every .1 seconds and a timeout after 1
//second, and call dist when the distance changes

void honk(){
    FILE *wave_file;
    wave_file=fopen("/sd/honk.wav","r"); 
    waver.play(wave_file); 
    fclose(wave_file);  
    myled1=!myled1;
    myled2=!myled2;
    myled3=!myled3;
    myled4=!myled4;
}



int main()
{   
    uLCD.reset();
    //calibrate motors
    servoBottom.calibrate(range, position);
    col_ESC();
    
    Ticker honk_int;
    
    char bnum = 0;
    char bhit = 0;
    float speed = 0.3;
    uLCD.baudrate(BAUD_3000000); //jack up baud rate to max for fast display
    wait(0.5);

    IMU.begin();
    if (!IMU.begin()) {
        // red circle of death
        uLCD.circle(63,63, 51, RED);
    }
    IMU.calibrate(1);
    uLCD.cls();
    uLCD.printf("Calibrating:\r\nSpin Me!\r\n"); //print to uLCD
    IMU.calibrateMag(0);
    uLCD.cls();
    
    int x=0;
    int y=50;
    mu.startUpdates();//start measuring the distance
    bool start_motor = true;
    bool moving = false;
    
    
    while(1) 
    {   
        // Check for collision
        mu.checkDistance();
        // If the motor is stopped and there are no collisions, start the motor
        if (global_running && start_motor){
            for (float p=0.0; p<=0.9; p += 0.1) { //Throttle up slowly to full throttle
                dcmotorBottom = p;
                wait(0.2);
            }
            // Draw the circle for the compass on the LCD
            uLCD.cls();
            uLCD.circle(63,63, 51, WHITE);
            start_motor = false;

        }
        if (global_running){
            // Update the compass reading
            // Only update once every 150000 loops
            if (frame_limiter % 150000 == 0){
                if(IMU.magAvailable(X_AXIS)){
                    IMU.readMag();
                    int prev_x = x;
                    int prev_y = y;
                    float heading = calculateHeading(IMU.mx, IMU.my);
                    x = 63+50*sin(heading+PI)+ 0.5;
                    y = 63+50*cos(heading+PI)+ 0.5;
                    uLCD.line(63, 63, prev_x, prev_y, BLACK);
                    uLCD.line(63, 63, x, y, RED);
                    uLCD.printf("%.2f\r", heading*180/PI);
                }
            }
            frame_limiter++;
            // Read bluetooth controller input
            if (blue.readable()){
                if (blue.getc() == '!') {
                    if (blue.getc() == 'B') { //button data packet
                        bnum = blue.getc(); //button number
                        bhit = blue.getc(); //1=hit, 0=release
                        if (blue.getc() == char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                            switch (bnum) {
                                case '1': //number button 1: Increase max speed
                                    if (bhit=='1') {
                                        //add hit code here
                                        if(speed < 0.3){
                                            speed = speed + 0.06;
                                        }
                                    } else {
                                        // When button is released, set rear motor to max speed
                                        // if it is currently running
                                        if(moving) dcmotorBack = speed;
                                    }
                                    break;
                                case '2': //number button 2: Decrease max speed
                                    if (bhit=='1') {
                                        //add hit code here
                                        if(speed > 0.06){
                                            speed = speed - 0.06;
                                        }
                                    } else {
                                        // When button is released, set rear motor to max speed
                                        // if it is currently running
                                        if(moving) dcmotorBack = speed;
                                    }
                                    break;
                                case '3': //number button 3: light the external leds
                                    if (bhit=='1') {
                                        exled1 = !exled1;
                                        exled2 = !exled2;
                                    } else {
                                        //add release code here
                                    }
                                    break;
                                case '4': //number button 4: play honking sound
                                    if (bhit=='1') {
                                        honk();                       
                                    } else {
                                        //add release code here
                                    }
                                    break;
                                case '5': //button 5 up arrow: turn on bottom servo and back motor
                                    if (bhit=='1') {
                                        //add hit code here  
                                        for (float p=0.0; p<=speed; p += 0.06) { //Throttle up slowly to full throttle
                                            dcmotorBack = p;
                                            wait(0.2);
                                        }
                                        moving = true;
                                                               
                                    } else {
                                        //add release code here
                                        dcmotorBack = 0.0;
                                        moving = false;
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
                }// End of bluetooth controller loop
            }
        }
        else
        {
            // Collision detection: Stop all motors and honk until obstruction is cleared
            dcmotorBack = 0.0;
            dcmotorBottom = 0.0;
            servoBottom = 0.5;
            start_motor = true;
            moving = false;
            unsigned int honk_limiter = 0;
            while(!global_running){
                // Check if the obstruction is clear
                mu.checkDistance();
                // Honk every once in a while
                if(honk_limiter % 6000000 == 0){
                    honk();
                }
                honk_limiter++;
                
            }
             
        }
        
    }
}