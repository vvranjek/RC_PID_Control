#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

//assume that pin 32 is receiving PWM input
#define CHANNEL_1_PIN 2
#define ANALOG_VOLT PIN_A1
#define ANALOG_ADJ1 PIN_A4
#define ANALOG_ADJ2 PIN_A5
#define PWM_OUT_PIN 4
#define SD_CS_PIN 5

#define AMP_MAX 50

unsigned long timer_start;
unsigned int last_interrupt_time; //calcSignal is the interrupt handler

bool direction = true; //forward = true, backwards = false;
bool pid_enable = false;
uint16_t pwm_time;
int16_t analog_volt_in;
float analog_adj1;
float analog_adj2;
float pwm_in;
float pwm_out_val;
float pwm_sp;
float pwm_pid_sp;
float current_adj;
float current_max;
float current_in;
float current_sp;

bool reverse = true;

long pulse_time;

// Pid settings
double kp = 0.0;
double ki, ki_default = 0.001;
double kd = 0.0;
double ff = 0.0;
double i_error_max = 1.01;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double cumError, rateError;

static float pwm_adjusted;

void calcSignal();
double computePID(double inp, double setPoint);
void PIDreset();



//this is all normal arduino stuff
void setup() {
    timer_start = 0;

    pinMode(CHANNEL_1_PIN, INPUT);
    pinMode(ANALOG_VOLT, INPUT);
    pinMode(ANALOG_ADJ1, INPUT);
    pinMode(ANALOG_ADJ2, INPUT);
    pinMode(PWM_OUT_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(2), calcSignal, CHANGE); //  function for creating external interrupts at pin2 on Rising (LOW to HIGH)
    //attachInterrupt(digitalPinToInterrupt(2),calcSignalDown,FALLING);  //  function for creating external interrupts at pin2 on Rising (LOW to HIGH)

    Serial.begin(115200);

    Serial.println("RCControl start");


    //SD.begin(SD_CS_PIN);


    // set up variables using the SD utility library functions:
    Sd2Card card;
    SdVolume volume;
    SdFile root;

    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
        Serial.println("SD init failed");
    } 

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    // if (!volume.init(card)) {
    //     Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    //     while (1);
    // }

}

void loop() {
    Serial.print("\r");

    //Serial.print(pulse_time);

    static float pwm_in_old;

    pwm_in = (reverse ? -1 : 1 )*(float)(pulse_time - 1500) / 500.0;

    if (pwm_in > 0) {
        direction = true;
    }
    else {
        direction = false;
        pwm_in = -pwm_in;
    }

    float slew_rate = 0.02;

    if (pwm_in > pwm_in_old) {
        pwm_in = pwm_in_old + slew_rate;
    }

    pwm_in_old = pwm_in;


    //Serial.print(pwm_in);

    // Get analaog values
    analog_volt_in = analogRead(ANALOG_VOLT);
    analog_adj1 = (float)analogRead(ANALOG_ADJ1) / 1024;
    analog_adj2 = (float)analogRead(ANALOG_ADJ2) / 1024;
    
    // Filter current adjust
    static float current_adj_prev;
    float current = analog_adj1 * AMP_MAX;
    current_adj = current_adj_prev + ((current - current_adj_prev) * 0.1);
    current_adj_prev = current_adj;
    current_max = current_adj;

    // Calculate curret setpoint
    float current_prev = current_in;
    current_in = current_prev + (((float)analog_volt_in / 12.6) - current_prev) * 1;
    current_sp = current_max * (pwm_in);

    // Get ki value from potentiomenter
    ki = analog_adj2 * 2.0 * ki_default;


    // Current PID control

    int mode = 2    ;
    switch (mode) {

    // Regular PID
    case 0:
        if (pwm_in < 0.05) {
            pwm_in = 0;
            pwm_sp = 0;
            computePID(current_in, 0.0);
        } else {
            //Serial.println("PID");
            pwm_pid_sp = computePID(current_in, current_sp);
            pwm_sp = pwm_pid_sp;
        }
        break;

    // PID only when current is too high
    case 1:
        if (pwm_in < 0.05) {
            pwm_in = 0;
            pwm_sp = 0;
            pid_enable = false;
            pwm_pid_sp = computePID(current_in, 0.0);
        } 
        else if (current_in > current_max * 0.9 || pid_enable == true) {
            pid_enable = true;
            //Serial.println("PID");
            pwm_pid_sp = computePID(current_in, current_max);
            pwm_sp = pwm_pid_sp;

            if (pwm_in < pwm_pid_sp) {
                pid_enable = false;  
            }

        
        } 
        else {
            Serial.println("Forward");
            PIDreset();
            pwm_sp = pwm_in;
        }
        break;

    // No PID. Custom simple controller
    case 2:
        if (pwm_in < 0.05) {
                pwm_in = 0;
                pwm_sp = 0;
                pwm_adjusted = 0;
                break;
            } 
        else if (current_in > current_max) {

            pwm_adjusted += 0.005;
        }
        else {
            if (pwm_adjusted > 0) {
                pwm_adjusted -= 0.005;
            }
            else {
                pwm_adjusted = 0.0;
            }
            
        }

        pwm_sp = pwm_in - pwm_adjusted;

        break;

    }

    if (1) {
        Serial.print(" Current val/sp/max ||| pwm_sp / pwm_pid_sp: ");
        Serial.print(pwm_in);
        Serial.print(" / ");
        Serial.print(pwm_sp);
        Serial.print(" / ");
        Serial.print(pwm_in_old);
        Serial.print(" ||| ");
        Serial.print(current_in);
        Serial.print(" / ");
        Serial.print(current_max);
    }
    
    //Serial.print("  \nread: ");

    

    delay(20);

    pwm_out_val = direction ? pwm_sp : -pwm_sp;
    uint16_t pwm_out_us = 1500 + (reverse ? -1 : 1 ) * pwm_out_val * 500;

    digitalWrite(PWM_OUT_PIN, HIGH);
    delayMicroseconds(pwm_out_us);
    digitalWrite(PWM_OUT_PIN, LOW);

    //Serial.println(current_adj);

    //28 = 2.25A
    //45 = 3.5A
    // 80 = 6.3

    // k = 12.6
}


double computePID(double inp, double setPoint) {
    currentTime = millis();                             //get current time
    elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

    error = setPoint - inp;    
    
    double i_error = error * elapsedTime;                  // determine error
    if (i_error > i_error_max) i_error = i_error_max;
    if (i_error < -i_error_max) i_error = -i_error_max;

    cumError += i_error * elapsedTime;               // compute integral
    //float i_max = 10.0;
    //constrain(cumError, -i_max, i_max);
    rateError = (error - lastError) / elapsedTime; // compute derivative

    //Serial.print(error);

    // PID calculation
    double out = kp * error + ki * cumError + kd * rateError; //PID output
    out = out + setPoint * ff;


    if (out > 1.0)
        out = 1.0;
    if (out < -0.0)
        out = -0.0;

    if (1) {
        Serial.print(" ");
        Serial.print(setPoint);
        Serial.print(" / ");
        Serial.print(inp);
        Serial.print(" / ");
        Serial.print(i_error);
        Serial.print(" ||| ");
        Serial.print(out);
        //Serial.print(" / ");
        //Serial.print(pwm_pid_sp);
        Serial.print("\r");

    }





    lastError = error;          //remember current error
    previousTime = currentTime; //remember current time

    return out; //have function return the PID output
}

void PIDreset() {
    error = 0;
    rateError = 0;
    cumError = pwm_sp/ki/2;
    currentTime = millis();
    previousTime = millis();
}

void calcSignal() {
    if (digitalRead(CHANNEL_1_PIN) == HIGH) {
        timer_start = micros();
    } else {
        if (timer_start != 0) {
            pulse_time = (micros() - timer_start);
            timer_start = 0;
        }
    }
}