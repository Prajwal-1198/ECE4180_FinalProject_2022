#include "mbed.h"
#include "LSM9DS1.h"
#include "MBed_Adafruit_GPS.h"

Serial pc(USBTX, USBRX);
Serial esp(p13, p14); // tx, rx
DigitalOut reset(p25);

DigitalOut accel_led(LED1);
DigitalOut pressure_led(LED2);
DigitalOut crash_led1(LED3);
DigitalOut crash_led2(LED4);

// GPS //
int counter = 10;
char gps_char;
Timer refresh_Timer;
const int refresh_Time = 2000;

double accident_latitude;
float accident_longitude;

// Speaker //
PwmOut speaker(p26);

// WiFi //
Timer t;
int  cnt, ended, timeout;
char buf[2024];
char snd[1024];
char accident_lat_lon_str[64];


char ssid[64] = "Prajwal's Galaxy S20 FE 5G";     // enter WiFi router ssid inside the quotes
char pwd[64] = "praj1234"; // enter WiFi router password inside the quotes

// Accelerometer //
LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);

// Pressure Sensor //
AnalogIn Pressure_In(p15);

float accel_X_previous = 0.0;
float accel_X_current = 0.0;
float accel_Y_previous = 0.0;
float accel_Y_current = 0.0;
float accel_Z_previous = 0.0;
float accel_Z_current = 0.0;

float pressure_sensor = 0.0;

bool accel_Y_changed = false;
bool accel_Z_changed = false;

int accel_iteration_num = 0;
bool accelerometer_crash_detected = false;
bool pressure_sensor_crash_detected = false;
bool accident_detected = false;

void SendCMD(), getreply(), ESPconfig1(), ESPconfig2(), ESPsetbaudrate(), SendValue();

void dev_recv()
{
    while (esp.readable()) {
        pc.putc(esp.getc());
    }
}

void pc_recv()
{
    while (pc.readable()) {
        esp.putc(pc.getc());
    }
}


int main()
{
    // WiFi //
    reset = 0; //hardware reset for 8266
    pc.baud(9600);  // set what you want here depending on your terminal program speed
    pc.printf("\f\n\r-------------ESP8266 Hardware Reset-------------\n\r");
    wait(0.5);
    reset = 1;
    timeout = 2;
    getreply();

    esp.baud(9600);   // change this to the new ESP8266 baudrate if it is changed at any time.

    //ESPsetbaudrate();   //******************  include this routine to set a different ESP8266 baudrate  ******************
    ESPconfig1();

    // Accelerometer Setup //
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed To Communicate With LSM9DS1\n");
    }
    IMU.calibrate();

    // GPS //
    Serial* gps_Serial;
    gps_Serial = new Serial(p28, p27);
    Adafruit_GPS myGPS(gps_Serial);

    myGPS.begin(9600);
    //These commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);

    wait(1);
    refresh_Timer.start(); //starts the clock on the timer

    // continuosly get AP list and IP
    while (accident_detected == false) {
        
        // Accelerometer //
        IMU.readAccel();
        accel_X_current = IMU.ax;
        accel_Y_current = IMU.ay;
        accel_Z_current = IMU.az;
        pc.printf("Accelerometer:    X-Axis = %f    Y-Axis = %f    Z-Axis = %f \n\r", accel_X_current, accel_Y_current, accel_Z_current);

        // Gather GPS Data - Happens Irrespective of Accident Detection - Only Send on Accident Detection
        gps_char = myGPS.read();   //queries the GPS
        if (gps_char) { pc.printf("%c", gps_char); } //this line will echo the GPS data if not paused
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if (myGPS.newNMEAreceived()) {
            if (!myGPS.parse(myGPS.lastNMEA())) {
                continue;
            }
        }
        //check if enough time has passed to warrant printing GPS info to screen
        if (refresh_Timer.read_ms() >= refresh_Time) {
            refresh_Timer.reset();
            pc.printf("Time: %d:%d:%d.%u\n", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
            pc.printf("Date: %d/%d/20%d\n", myGPS.day, myGPS.month, myGPS.year);
            pc.printf("Fix: %d\n", (int)myGPS.fix);
            pc.printf("Location: %5.2f%c, %5.2f%c\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
            pc.printf("Satellites: %d\n", myGPS.satellites);
        }

        // Detect on Accelerometer
        if (((accel_Y_previous - accel_Y_current) > (5000)) && (accel_iteration_num != 0)) {
            accel_Y_changed = true;
        }
        if (((accel_Z_previous - accel_Z_current) > (5000)) && (accel_iteration_num != 0)) {
            accel_Z_changed = true;
        }

        if (accel_Z_changed == true && accel_Y_changed == true) {
            accelerometer_crash_detected = true;
            accel_led = 1;
        }
        else if (accel_Z_changed == true && accel_Y_changed == false) {
            accel_Z_changed = false;
        }
        else if (accel_Z_changed == false && accel_Y_changed == true) {
            accel_Y_changed = false;
        }

        // Read & Detect Pressure Sensor
        pressure_sensor = Pressure_In.read();
        pc.printf("Pressure Sensor Value:  %f\r\n\n", pressure_sensor);
        if (pressure_sensor > 0.3)
        {
            pressure_sensor_crash_detected = true;
            pressure_led = 1;
        }

        accel_iteration_num = 1;
        accel_X_previous = accel_X_current;
        accel_Y_previous = accel_Y_current;
        accel_Z_previous = accel_Z_current;

        if ((accelerometer_crash_detected == true) && (pressure_sensor_crash_detected == true)) {
            
            accident_detected = true;
            accident_latitude = myGPS.latitude;
            accident_longitude = myGPS.longitude;
        }
        //sleep();
        wait(0.1);
    } // End - while(accident_detected == false)

    if (accident_detected == true)
    {

        ESPconfig2();        //******************  include Config to set the ESP8266 configuration  ***********************

        pc.attach(&pc_recv, Serial::RxIrq);
        esp.attach(&dev_recv, Serial::RxIrq);

        // Blare Speaker Alarm
        // Increase Volume by changing PWM duty cycle
        for (int i = 0; i < 26; i = i + 2) {
            speaker.period(1.0 / 969.0);
            speaker = float(i) / 50.0;
            wait(.5);
            speaker.period(1.0 / 800.0);
            wait(.5);
        }
        // Decrease Volume by changing PWM duty cycle
        for (int i = 25; i >= 0; i = i - 2) {
            speaker.period(1.0 / 969.0);
            speaker = float(i) / 50.0;
            wait(.5);
            speaker.period(1.0 / 800.0);
            wait(.5);
        }
    }
}

// Sets new ESP8266 baurate, change the esp.baud(xxxxx) to match your new setting once this has been executed
void ESPsetbaudrate()
{
    strcpy(snd, "AT+CIOBAUD=115200\r\n");   // change the numeric value to the required baudrate
    SendCMD();
}
void ESPconfig1()
{
    wait(5);
    pc.printf("\f---------- Starting ESP Config ----------\r\n\n");
    strcpy(snd, ".\r\n.\r\n");
    SendCMD();
    wait(1);

    pc.printf("\n---------- Connecting to AP ----------\r\n");
    pc.printf("ssid = %s   pwd = %s\r\n", ssid, pwd);
    strcpy(snd, "wifi.sta.config(\"");
    strcat(snd, ssid);
    strcat(snd, "\",\"");
    strcat(snd, pwd);
    strcat(snd, "\")\r\n");
    SendCMD();
    timeout = 10;
    getreply();
    pc.printf(buf);

    wait(5);

    pc.printf("\n---------- Get IP's ----------\r\n");
    strcpy(snd, "print(wifi.sta.getip())\r\n");
    SendCMD();
    timeout = 3;
    getreply();
    pc.printf(buf);

    wait(1);

    pc.printf("\n---------- Get Connection Status ----------\r\n");
    strcpy(snd, "print(wifi.sta.status())\r\n");
    SendCMD();
    timeout = 5;
    getreply();
    pc.printf(buf);

    pc.printf("\n\n\n  If you get a valid (non zero) IP, ESP8266 has been set up.\r\n");
    pc.printf("  Run this if you want to reconfig the ESP8266 at any time.\r\n");
    pc.printf("  It saves the SSID and password settings internally\r\n");
    wait(10);

    pc.printf("\n---------- Setting up http server ----------\r\n");
    strcpy(snd, "srv=net.createServer(net.TCP)\r\n");
    SendCMD();
    wait(1);
    strcpy(snd, "srv:listen(80,function(conn)\r\n");
    SendCMD();
    wait(1);
    strcpy(snd, "conn:on(\"receive\",function(conn,payload)\r\n");
    SendCMD();
    wait(1);
}
//  +++++++++++++++++++++++++++++++++ This is for ESP8266 config only, run this once to set up the ESP8266 +++++++++++++++
void ESPconfig2()
{


    strcpy(snd, "conn:send(\"<!DOCTYPE html><html><h1> !!ACCIDENT DETECTED!! </h1> <h2> Patient Name: XYZ </h2> <h2> Blood Group: B+ve </h2>\")\r\n");
    SendCMD();
    wait(1);

    sprintf(snd, "conn:send(\"<h1> latitude = %f, longitude = %f </h1></html>\")\r\n end)\r\n", accident_latitude, accident_longitude);
    SendCMD();
    wait(1);

    strcpy(snd, "conn:on(\"sent\",function(conn) conn:close() end)\r\n");
    SendCMD();
    wait(1);
    strcpy(snd, "end)\r\n");
    SendCMD();
    wait(1);

    getreply();

    pc.printf("\r\nDONE");
}

void SendCMD()
{
    esp.printf("%s", snd);
}

void getreply()
{
    memset(buf, '\0', sizeof(buf));
    t.start();
    ended = 0;
    cnt = 0;
    while (!ended) {
        if (esp.readable()) {
            buf[cnt] = esp.getc();
            cnt++;
        }
        if (t.read() > timeout) {
            ended = 1;
            t.stop();
            t.reset();
        }
    }
}