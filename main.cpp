/*
EMT1B - PES - Groep 3
Revisie: 02
*/

// Includes
#include "mbed.h"
#include "rtos.h"
#include "HCSR04/HCSR04.h"

//#include "vr3-ultrasonic.cpp"

// Defines
#define SLEEP_TIME                  1000 // (msec)
#define PRINT_AFTER_N_LOOPS         20

// Declarations
Serial pc(USBTX, USBRX);
Thread thread1;
Thread thread2;
Thread thread3;
Thread thread4;


Timer ts1;
Timer ts2;
Timer ts3;
Timer ts4;
Timer fallprotection;

Timer tl;
Timer tr;

// Startknop
void toggle_start(void); // function prototype
Timer debounce; // define debounce timer
InterruptIn button_in(USER_BUTTON);        //startknop pin





// In - PwmOut
DigitalOut led1(LED1);

DigitalOut relais_l1_MO(A1);    // Motoren rijden
DigitalOut relais_l2_MB(A2);    // Motorshield Borstels
DigitalOut relais_l3_FAN(A3);   // Fan


// Shield 1 (CN6 - H1+2) Motor L + R
PwmOut L1_13(D9);
PwmOut L1_14(D8);   
PwmOut L1_15(D10);
PwmOut L1_16(D11);

// Shield 2 (CN5 - H1+2) Borstels
PwmOut L2_13(D12);
PwmOut L2_14(D13);
PwmOut L2_15(D14);
PwmOut L2_16(D15);

// Status LED
PwmOut LED_R(PB_15);
PwmOut LED_G(PB_14);
PwmOut LED_B(PB_13);

// States
enum State {
    STATE_WAIT_START,
    STATE_STARTUP,
    STATE_OBSTACLE_DETECTED,
    STATE_FORWARD,
    STATE_REVERSE,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} state = STATE_STARTUP;

enum led_color {
    RED,
    ORANGE,
    GREEN,
    BLUE,
    PURPLE
} color = RED; 

// Global data
struct distances {
  double d_s1;
  double d_s2;
  double d_s3;
  double d_s4;
} sensor_data;


void read_s1() {
    HCSR04 s1(D3, D2);              // Sensor 1 - Echo en Trigger
    s1.setRanges(2, 110);           // Ranges

    while (true) {
         ts1.reset();
         ts1.start();
         s1.startMeasurement();
         while(!s1.isNewDataReady()) {
             // wait for new data
             // waiting time depends on the distance
         }
         sensor_data.d_s1 = s1.getDistance_mm();
         ts1.stop();
         ThisThread::sleep_for(125 - ts1.read_ms()); // time the loop

    }

    
}

void read_s2(){
    HCSR04 s2(D5, D4);              // Sensor 2 - Echo en Trigger
    s2.setRanges(2, 110);           // Ranges

    while (true) {
         ts2.reset();
         ts2.start();
         s2.startMeasurement();
         while(!s2.isNewDataReady()) {
             // wait for new data
             // waiting time depends on the distance
         }
         sensor_data.d_s2 = s2.getDistance_mm();
         ts2.stop();
         ThisThread::sleep_for(125 - ts2.read_ms()); // time the loop

    }
}

void read_s3(){
    HCSR04 s3(D7, D6);              // Sensor 3 - Echo en Trigger
    s3.setRanges(2, 110);           // Ranges

    while (true) {
         ts3.reset();
         ts3.start();
         s3.startMeasurement();
         while(!s3.isNewDataReady()) {
             // wait for new data
             // waiting time depends on the distance
         }
         sensor_data.d_s3 = s3.getDistance_mm();
         ts3.stop();
         ThisThread::sleep_for(125 - ts3.read_ms()); // time the loop

    }
    
}

void read_s4(){
    HCSR04 s4(A5, A4);              // Sensor 4 - Echo en Trigger
    s4.setRanges(2, 110);           // Ranges

    while (true) {
         ts4.reset();
         ts4.start();
         s4.startMeasurement();
         while(!s4.isNewDataReady()) {
             // wait for new data
             // waiting time depends on the distance
         }
         //sensor_data.d_s4 = 100;
         sensor_data.d_s4 = s4.getDistance_mm();
         ts4.stop();
         ThisThread::sleep_for(125 - ts4.read_ms()); // time the loop

    }
    
}





// Startbutton interrupt
void toggle_start(){

    led1 = !led1;
    if (debounce.read_ms()>200){
        if (state == STATE_WAIT_START){
            state = STATE_STARTUP;
        } else {
            state = STATE_WAIT_START;  // Start-stop systeem
        }
        
        debounce.reset();
    }
}


void status_led(led_color color){
    if (color == 0){
        // ROOD
        LED_R = 1.0;
        LED_G = 0.0;
        LED_B = 0.0;
    } else if (color == 1){
        // ORANJE
        LED_R = 0.9;
        LED_G = 0.15;
        LED_B = 0.0;
    } else if (color == 2){
        // GROEN
        LED_R = 0.0;
        LED_G = 1.0;
        LED_B = 0.0;
    } else if (color == 3){
        // BLAUW
        LED_R = 0.0;
        LED_G = 0.0;
        LED_B = 1.0;
    } else if (color == 4){
        //PAARS
        LED_R = 1.0;
        LED_G = 0.0;
        LED_B = 1.0;
    }
}




/*                 Shield 1 
         | L13 | L14 |-| L15 | L16 |
Brake    |  L  |  L  |-|  L  |  L  | 
Forward  |  L  |  H  |-|  L  |  H  |  
Back     |  H  |  L  |-|  H  |  L  | 
Brake    |  H  |  H  |-|  H  |  H  |

*/

void forward(){
    relais_l1_MO = false;
    L1_13 = 0.5;    // Linker motor vooruit
    L1_14 = 0.0;    
    L1_15 = 0.5;    // Rechter motor vooruit
    L1_16 = 0.0;    
}

void brake(){       
    relais_l1_MO = false;       // Motoren stoppen
    L1_13 = 0.0;
    L1_14 = 0.0;    
    L1_15 = 0.0;
    L1_16 = 0.0;      
}


void turn_left(){
    status_led(PURPLE);
    pc.printf("Turn Left\r\n");
    tl.start();
    while((sensor_data.d_s1 > 50) && (tl.read_ms() < 600)){
        relais_l1_MO = false;
        L1_13 = 0.7;
        L1_14 = 0.0;    // Linker motor vooruit
        L1_15 = 0.0;
        L1_16 = 0.0;
        pc.printf("Turn Left loop");
    }
    tl.stop();
    tl.reset();
    state = STATE_FORWARD;
}

void turn_right(){
    status_led(PURPLE);
    tr.start();
    while((sensor_data.d_s1 > 50) && (tr.read_ms() < 600)){
        relais_l1_MO = false;
        L1_13 = 0.0;
        L1_14 = 0.0;    // Rechter motor vooruit
        L1_15 = 0.7;
        L1_16 = 0.0;
    }
    tr.stop();
    tr.reset();
    state = STATE_FORWARD;
}

void turn_around(){
    status_led(PURPLE);
    tr.start();
    while((sensor_data.d_s1 > 50) && (tr.read_ms() < 850)){
        relais_l1_MO = false;
        L1_13 = 0.0;
        L1_14 = 0.0;    // Rechter motor vooruit
        L1_15 = 1.0;
        L1_16 = 0.0;
    }
    tr.stop();
    tr.reset();
    state = STATE_FORWARD;
}







// main() runs in its own thread in the OS
int main()
{




    //PWM setup
    L1_13.period_ms(1);
    L1_14.period_ms(1);
    L1_15.period_ms(1);
    L1_16.period_ms(1);
    L1_13 = 0.0;
    L1_14 = 0.0;
    L1_15 = 0.0;
    L1_16 = 0.0;

    L2_13.period_ms(1);
    L2_14.period_ms(1);
    L2_15.period_ms(1);
    L2_16.period_ms(1);
    L2_13 = 0.0;
    L2_14 = 0.0;
    L2_15 = 0.0;
    L2_16 = 0.0;

    LED_R.period_ms(1);
    LED_G.period_ms(1);    
    status_led(RED);

    
    //Threads
    thread1.start(read_s1);
    thread2.start(read_s2);
    thread3.start(read_s3);
    thread4.start(read_s4);

    // Motor + Fan beveiliging relais,  TRUE = Relay UIT, FALSE = Relay AAN!
    relais_l1_MO = true;   // Motoren
    relais_l2_MB = true;   // Motor borstels
    relais_l3_FAN = true;  // FAN

    debounce.start();
    //button_in.mode(PullDown);
    button_in.fall(&toggle_start); // attach the address of the toggle


/*
    while (true) {
        ThisThread::sleep_for(50);
        //led1 = !led1;

        pc.printf("d_s1 = %5.1f mm, d_s2 = %5.1f mm \r", sensor_data.d_s1, sensor_data.d_s2);

        //pc.printf("d_s1 = %5.1f mm\r\n", sensor_data.d_s1);
        //pc.printf("d_s2 = %5.1f mm\r\n", sensor_data.d_s2);

    }
*/

    while(true){



        pc.printf("state = %d, d_s1 = %5.1f mm, d_s2 = %5.1f mm, d_s3 = %5.1f mm, d_s4 = %5.1f mm \r", state, sensor_data.d_s1, sensor_data.d_s2, sensor_data.d_s3, sensor_data.d_s4);

        switch(state){
            case STATE_WAIT_START:
                // Noodstop ingedrukt, motoren uitzeggen van rijden+zuigen+vegen
                pc.printf("STATE_WAIT_START");
                relais_l1_MO = true;   // MotorEN
                relais_l2_MB = true;   // Borstels
                relais_l3_FAN = true;  // Fan
                break;
            case STATE_STARTUP:
                // Motoren opstarten van zuigen en vegen via relais
                pc.printf("STATE_STARTUP");
                status_led(ORANGE);
                relais_l2_MB = false;  // borstels
                L2_13 = 0.0;
                L2_14 = 0.3;
                L2_15 = 0.0;
                L2_16 = 1.0;
                ThisThread::sleep_for(3500);
                relais_l3_FAN = false;   // FAN
                ThisThread::sleep_for(1500);

                state = STATE_FORWARD;
                break;
            case STATE_FORWARD:
                led1 = true;
                if (color != GREEN){
                    status_led(GREEN);
                    
                }
                relais_l1_MO = false;
                pc.printf("STATE_FORWARD\r\n");
                
                if (sensor_data.d_s4 > 100){
                    // We rijden bijna van de trap af!
                    pc.printf("TURN AROUND! ");
                    fallprotection.start();
                    if (fallprotection.read_ms() > 150){
                        turn_around();
                        fallprotection.stop();
                        fallprotection.reset();
                    }
                } else if (sensor_data.d_s4 < 75){
                        fallprotection.stop();
                        fallprotection.reset();
                } 
                
                if ((sensor_data.d_s1 < 600) || (sensor_data.d_s2 < 600) || (sensor_data.d_s3 < 600)) {
                    pc.printf("STATE_FORWARD - obstacle");
                    state = STATE_OBSTACLE_DETECTED;
                    brake();
                    
                } else {
                    pc.printf("STATE_FORWARD - geen obstacle");
                    forward();  
                }
                break; 
            case STATE_OBSTACLE_DETECTED:
                // Obstacle detected
                led1 = false;
                brake();
                status_led(BLUE);
                pc.printf("STATE_OBSTACLE_DETECTED");
                if(sensor_data.d_s3 < 350){ // > 200 s3
                    // Links draaien
                    pc.printf("STATE_OBSTACLE_DETECTED - Links draaien");
                    turn_left();
                } else if (sensor_data.d_s2 < 350){ // > 200 s2
                    // Rechts draaien
                    turn_right();
                    pc.printf("STATE_OBSTACLE_DETECTED - Rechts draaien");
                } else {
                    // Links draaien
                    pc.printf("STATE_OBSTACLE_DETECTED - Middle -> links draaien");
                    turn_left();
                }
            default: break;
        }
        //ThisThread::sleep_for(500);
    }

}








    //SystemReport sys_state( SLEEP_TIME * PRINT_AFTER_N_LOOPS /* Loop delay time in ms */);
    //int count = 0;
    //    if ((0 == count) || (PRINT_AFTER_N_LOOPS == count)) {
    //        // Following the main thread wait, report on the current system status
    //        sys_state.report_state();
    //        count = 0;
    //    }
    //    ++count;


