#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include <stdlib.h>

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define A                0.02  //Complemetary filter ratio // TODO decrease to 0.01
#define MAX_GYRO_RATE    300   // deg/s
#define MAX_ROLL_ANGLE   45.0  // deg
#define MAX_PITCH_ANGLE  45.0  // deg
#define PWM_MAX          1900
#define PWM_OFF          1000 // DO NOT CHANGE!!!!!!!!!!! OR SET KILL TO 1000
#define frequency        25000000.0
#define LED0             0x6			
#define LED0_ON_L        0x6		
#define LED0_ON_H        0x7		
#define LED0_OFF_L       0x8		
#define LED0_OFF_H       0x9		
#define LED_MULTIPLYER   4
#define NEUTRAL_THRUST   1400 // 1350 // 1450 // 1500
#define P_PITCH          5 // 7 // 6 // 5 // 13 // 13 // 10 // 5
#define D_PITCH          0.7 // 1.75 //3 // 3 // 522
#define I_PITCH          0.075 // 0.05
#define MAX_PITCH_I      100 // 75 //50 
#define P_ROLL           5 // 7 // 6 // 5 // 15 // 9 // 13
#define D_ROLL           0.7 // 0.6 // 0.8 // 1.75 // 1.0 // 1.2 // 1.75
#define I_ROLL           0.075
#define MAX_ROLL_I       100
#define P_YAW            2 // 1 //0.5 // 2 // 7 // 13
#define PITCH_MAX        7 // 10 // 8 // 11 // 9 // 10 // 15 // Degrees
#define ROLL_MAX         7 // 10 // 8 // 11 // 9 // 15 // Degrees
#define YAW_MAX          100 //5 // DPS
#define JOY_NEUTRAL      128
#define JOY_HIGH         240
#define JOY_LOW          16


// Max gyro - 400
// P - 5
// D - 0.5


// Pitch PID values
// P 13, D 3 -> Good for pitch hold.
// PID Pitch & Roll Good -> P 13 D 1.75 I 0.075 & 100 for max


// scp pi@192.168.0.1:/home/pi/flight_controller/W5M5_Zero_Yaw_Velocity.csv /home/marno/Classes/Winter23/Quadrotor/Autonomous_quadrotor/data/

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// //week2 addition
// struct Keyboard {
//   char key_press;
//   int heartbeat;
//   int version;
// };

//week 6 Joystick
struct Keyboard {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num; // heart
};
 
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_keyboard();
void trap(int signal);
void safety_check(Keyboard keyboard);
// void keyboard_controls(Keyboard keyboard);
void joystick_control(Keyboard keyboard);
void get_joystick(Keyboard keyboard);
void init_pwm();
void init_motor(uint8_t channel);
void set_PWM( uint8_t channel, float time_on_us);
void pid_update();

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gabs(filtered_pitch)>MAX_Pitch_ANGLEyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
struct timespec t_heartbeat;
long time_curr_heartbeat;
long time_prev_heartbeat = 0.0;
int hearbeat_prev = 0;
float yaw=0.0;
float pitch_angle=0.0;
float roll_angle=0.0;
//intitialising variables used to average data for calibration
float pi = 3.14159;
float filtered_pitch = 0.0;
float filtered_roll = 0.0;
float roll_gyro_delta=0.0;
float pitch_gyro_delta = 0.0;
FILE *file_p;
int pwm;
int pwm_0 = 1000;
int pwm_1 = 1000;
int pwm_2 = 1000;
int pwm_3 = 1000;
float pitch_error;
float pitch_error_I = 0.0;
float roll_error;
float roll_error_I = 0.0;
float desired_thrust = NEUTRAL_THRUST;
int prev_version = 1000; //  For keyboard control
float desired_pitch = 0.0;
float desired_roll = 0.0;
float desired_yaw_velocity = 0.0;
float yaw_error_velocity;
// Joystick variables
int joy_pitch = 0;
int joy_roll = 0;
int joy_yaw = 0;
int joy_thrust = 0;
int joy_keypress = 0;
int sequence_num = 0;

// Flags
int Flag_pause = 1;

Keyboard* shared_memory;
int run_program=1;
 
int main (int argc, char *argv[])
{
    //in main function before calibrate imu add
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);

    delay(1000);

    setup_imu();
    calibrate_imu();
    setup_keyboard();
    signal(SIGINT, &trap);
    delay(1000);

    //to refresh values from shared memory first
    Keyboard keyboard=*shared_memory;

    // Save values to CSV
    // file_p = fopen("common_filter.csv", "w+");
    // float temp_pitch = 0.0;
    // float temp_roll = 0.0;
    // file_p = fopen("Pitch_P_controller.csv", "w+");
    // file_p = fopen("W4M3.csv", "w+"); 
    // file_p = fopen("W4M4.csv", "w+");
    // file_p = fopen("W5M2_PID_pitch.csv", "w+");
    // file_p = fopen("W5M3_Desired_Roll.csv", "w+");
    // file_p = fopen("W5M5_Zero_Yaw_Velocity.csv", "w+");

    while(run_program==1)
    { 
      read_imu();
      update_filter();
      //Introducing 100millisec delay to allow the displayed data to be more readable
      // delay(100);
      // printf("\n Gyro(xyz) = %10.5f %10.5f %10.5f     Pitch = %10.5f Roll = %10.5f", imu_data[0], imu_data[1], imu_data[2], pitch_angle, roll_angle);
      
      // printf("\n Pitch: %10.5f, Filtered pitch: %10.5f, pitch_gyro_delta: %10.5f , gyro x: %10.5f ", pitch_angle, filtered_pitch, pitch_gyro_delta, imu_data[0]);
      // printf("\n Roll: %10.5f, Filtered roll: %10.5f, roll_gyro_delta: %10.5f , gyro y: %10.5f ", roll_angle, filtered_roll, roll_gyro_delta, imu_data[1]);
      

      // printf(" Filtered pitch: %10.5f, Filtered roll: %10.5f\n", filtered_pitch, filtered_roll);
      
      
      // Save values to CSV
      // temp_pitch += pitch_gyro_delta;P_ROLL
      // fprintf(file_p, "%f, %f, %f,\n", temp_pitch, pitch_angle, filtered_pitch);
      // fprintf(file_p, "%f, %f, %f\n", temp_roll, roll_angle, filtered_roll);
      // fprintf(file_p, "%d, %d, %f, %f,\n", pwm_0, pwm_1, pitch_angle*(20), filtered_pitch*20);
      // fprintf(file_p, "%d, %d, %f, %f,\n", pwm_0, pwm_1, filtered_pitch*20, imu_data[0]*20);
      // fprintf(file_p, "%d, %d, %f, %f, %f\n", pwm_0, pwm_1,  pitch_angle*(20), temp_pitch*20, filtered_pitch*20);
      // fprintf(file_p, "%f, %f\n", filtered_pitch, desired_pitch);
      // fprintf(file_p, "%f, %f\n", filtered_roll, desired_roll);
    //   fprintf(file_p, "%d, %d, %d, %d, %f,\n", pwm_0, pwm_1, pwm_2, pwm_3, imu_data[2]*15); // yaw P controller
      

      // to refresh values from shared memory first
      Keyboard keyboard=*shared_memory;
      safety_check(keyboard);
      //   keyboard_controls(keyboard);
      joystick_control(keyboard);
      get_joystick(keyboard);

      if (Flag_pause == 0) // Pause motors
      {
        // Update motor speeds
        pid_update();
      }

      // set_PWM(0,1100);
      // set_PWM(1,1100);
      // set_PWM(2,1100);
      // set_PWM(3,1100);

    //   printf("keypress: %d  pitch: %d  roll: %d  yaw: %d  thrust: %d  sequence_num: %d\n", keyboard.keypress, keyboard.pitch, keyboard.roll, keyboard.yaw, keyboard.thrust, keyboard.sequence_num);
      // printf("key_press: %d  heartbeat: %d  version: %d\n", keyboard.key_press, keyboard.heartbeat, keyboard.version);
    //   printf("pwm_0: %d  pwm_1: %d  pwm_2: %d  pwm_3: %d  pitch_error: %f run = %d\n", pwm_0, pwm_1, pwm_2, pwm_3, pitch_error, run_program);
    // printf("desired_thrust: %f", desired_thrust);
      // printf("pitch_error_I: %5.1f  Filtered_pitch: %5.1f  pitch_error: %5.1f \n", pitch_error_I, filtered_pitch, pitch_error);
      // printf("Filtered_pitch: %5.1f  desired_pitch: %5.1f \n", filtered_pitch, desired_pitch);
      // printf("Filtered_roll: %5.1f  desired_roll: %5.1f \n", filtered_roll, desired_roll);
    }

    // Save values to CSV
    // fclose(file_p);

    // Kill all motors
    printf("\n Killing Motors! \n"); 
    set_PWM(0,PWM_OFF);
    set_PWM(1,PWM_OFF); 
    set_PWM(2,PWM_OFF);
    set_PWM(3,PWM_OFF);
    delay(1000);
    printf("\n Motors dead \n");

    return 0;
}

void calibrate_imu()
{
  // initialize sum variables locally
  float x_gyro_sum =0.0;
  float y_gyro_sum=0.0;
  float z_gyro_sum=0.0;
  float roll_sum=0.0;
  float pitch_sum=0.0;
  float accel_z_sum=0.0;

  // loop that runs to sum data over a 1000 iterations
  for (int i=0; i<1000; i++){
    read_imu();

    x_gyro_sum += imu_data[0];
    y_gyro_sum += imu_data[1];
    z_gyro_sum += imu_data[2];
    roll_sum += roll_angle;
    pitch_sum += pitch_angle;
    accel_z_sum += imu_data[5];
  }

  // Add negetive to eliminate drift
  //Averaging data using the summation variables
  x_gyro_calibration = -x_gyro_sum/1000.0;
  y_gyro_calibration = -y_gyro_sum/1000.0;
  z_gyro_calibration = -z_gyro_sum/1000.0;
  roll_calibration = -roll_sum/1000.0;
  pitch_calibration = -pitch_sum/1000.0;
  accel_z_calibration = -accel_z_sum/1000.0;

  printf("\n calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void read_imu()
{
  int address=59;// set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[3]=vw*(-2.0-2.0)/(-32768.0-32767.0);// convert vw from raw values to "g's"
  
  
  address=61;// set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=vw*(-2.0-2.0)/(-32768.0-32767.0);// convert vw from raw values to "g's"
  
  
  address=63;// set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=vw*(-2.0-2.0)/(-32768.0-32767.0);// convert vw from raw values to g's
  
  
  address=67;// set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]= -(x_gyro_calibration + vw*(-500.0-500.0)/(-32768.0-32767.0));// convert vw from raw values to degrees/second 
  
  address=69;// set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
 imu_data[1]=y_gyro_calibration + vw*(-500.0-500.0)/(-32768.0-32767.0);// convert vw from raw values to degrees/second     
  
  address=71;// set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=z_gyro_calibration + vw*(-500.0-500.0)/(-32768.0-32767.0);// convert vw from raw values to degrees/second
  
  // Calculate Roll, Pitch and Yaw
  pitch_angle = (atan2(imu_data[4],-imu_data[5])*180/pi) + pitch_calibration;
  roll_angle = (atan2(imu_data[3],-imu_data[5])*180/pi) + roll_calibration;
  // yaw = todo
}

void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;

  //comp. filter for roll, pitch here: 
  roll_gyro_delta = imu_data[1]*imu_diff; // Gyro roll angle calculation
  filtered_roll = roll_angle*A + (1-A)*(roll_gyro_delta+filtered_roll);

  pitch_gyro_delta = imu_data[0]*imu_diff; // Gyro pitch angle calculation
  filtered_pitch = pitch_angle*A + (1-A)*(pitch_gyro_delta+filtered_pitch);
}

int setup_imu()
{
  wiringPiSetup ();

  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address

  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}

//function to add
void setup_keyboard()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;

  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment.  */
  // sprintf(shared_memory, "test!!!!.");
}

//when cntrl+c pressed, kill motors
void trap(int signal)
{
  printf("\n ending program - control C was pressed - Kill Motors \n\r");
  // Kill all motors
  set_PWM(0,PWM_OFF);
  set_PWM(1,PWM_OFF); 
  set_PWM(2,PWM_OFF);
  set_PWM(3,PWM_OFF);
  run_program=0;
}

// void safety_check(Keyboard keyboard)
// {
// /*
//   Safety checks that stops the student program when any of the following cases are violated/detected:
//   – Any gyro rate > 300 degrees/sec
//   – Roll angle > 45 or <-45
//   – Pitch angle >45 or <-45
//   – Keyboard press of space
//   – Keyboard timeout (Heart beat does not update in 0.25 seconds)
// */

//   //get current time in nanoseconds
//   timespec_get(&t_heartbeat,TIME_UTC);
//   time_curr_heartbeat = t_heartbeat.tv_nsec;
//   //compute time since last execution
//   double passed_time = time_curr_heartbeat-time_prev_heartbeat;

//   //check for rollover
//   if(passed_time<=0.0)
//   {
//     passed_time+=1000000000.0;
//   }

//   //convert to seconds
//   passed_time=passed_time/1000000000.0;

//   if (hearbeat_prev != keyboard.heartbeat)
//   { // Reset previous heartbeat time stamp if a new heartbeat is detected.
//     hearbeat_prev = keyboard.heartbeat;
//     time_prev_heartbeat = time_curr_heartbeat;
//   }
//   else if (passed_time>0.25)
//   { // If the previous heartbeat is the same as the current heartbeat and 0.25s has passed
//     // Stop the student from executing.
//     printf("Keyboard timedout! (Heartbeat)");
//     run_program=0;
//   }

//   if (keyboard.keypress == 32)
//   { // If the keyboards space bar is pressed, then stop the student code.
//     printf("\n Space bar was pressed!");
//     run_program=0;
//   }
//   else if (abs(filtered_pitch)>MAX_PITCH_ANGLE || abs(filtered_roll)>MAX_ROLL_ANGLE)
//   { // If the pitch or roll angles are larger than the max allowable angle, then stop the student code.
//     printf("\n Pitch or Roll angle exceeds maximum limit: Pitch: %10.5f  Roll: %10.5f", filtered_pitch, filtered_roll);
//     run_program=0;
//   }
//   else if (abs(imu_data[0])>MAX_GYRO_RATE || abs(imu_data[1])>MAX_GYRO_RATE || abs(imu_data[2])>MAX_GYRO_RATE)
//   { // If any of the 3 gyro rates are larger than the max allowable gyro rate, then stop the student code.
//     printf("\n Gyro rate exceeds maximum limit: x: %10.5f  y: %10.5f  z: %10.5f", imu_data[0], imu_data[1], imu_data[2]);
//     run_program=0;
//   }
// }

void safety_check(Keyboard keyboard) // Joystick
{
/*
  Safety checks that stops the student program when any of the following cases are violated/detected:
  – Any gyro rate > 300 degrees/sec
  – Roll angle > 45 or <-45
  – Pitch angle >45 or <-45
  – Keyboard press of space
  – Keyboard timeout (Heart beat does not update in 0.25 seconds)
*/

  //get current time in nanoseconds
  timespec_get(&t_heartbeat,TIME_UTC);
  time_curr_heartbeat = t_heartbeat.tv_nsec;
  //compute time since last execution
  double passed_time = time_curr_heartbeat-time_prev_heartbeat;

  //check for rollover
  if(passed_time<=0.0)
  {
    passed_time+=1000000000.0;
  }

  //convert to seconds
  passed_time=passed_time/1000000000.0;

  if (hearbeat_prev != keyboard.sequence_num)
  { // Reset previous heartbeat time stamp if a new heartbeat is detected.
    hearbeat_prev = keyboard.sequence_num;
    time_prev_heartbeat = time_curr_heartbeat;
  }
  else if (passed_time>0.5) // TODO was 0.25
  { // If the previous heartbeat is the same as the current heartbeat and 0.25s has passed
    // Stop the student from executing.
    printf("Keyboard timedout! (Heartbeat)");
    printf("Heartbeat_prev  %d  Current heart %d  passed_time %f", hearbeat_prev, keyboard.sequence_num, passed_time);
    run_program=0;
  }

  if (keyboard.keypress == 32)
  { // If the joystick A button is pressed, then stop the student code.
    printf("\n A button was pressed!");
    run_program=0;
  }
  else if (abs(filtered_pitch)>MAX_PITCH_ANGLE || abs(filtered_roll)>MAX_ROLL_ANGLE)
  { // If the pitch or roll angles are larger than the max allowable angle, then stop the student code.
    printf("\n Pitch or Roll angle exceeds maximum limit: Pitch: %10.5f  Roll: %10.5f", filtered_pitch, filtered_roll);
    run_program=0;
  }
  else if (abs(imu_data[0])>MAX_GYRO_RATE || abs(imu_data[1])>MAX_GYRO_RATE || abs(imu_data[2])>MAX_GYRO_RATE)
  { // If any of the 3 gyro rates are larger than the max allowable gyro rate, then stop the student code.
    printf("\n Gyro rate exceeds maximum limit: x: %10.5f  y: %10.5f  z: %10.5f", imu_data[0], imu_data[1], imu_data[2]);
    run_program=0;
  }
}

void pid_update()
{
  // Calculate pitch error
  pitch_error = desired_pitch - filtered_pitch;
  pitch_error_I += pitch_error*I_PITCH;

  // Calculate roll error
  roll_error = desired_roll - filtered_roll;
  roll_error_I += roll_error*I_ROLL;

  // Calculate yaw error
  yaw_error_velocity = desired_yaw_velocity - imu_data[2];

  // Limit pitch integral term
  if (pitch_error_I > MAX_PITCH_I)
  {
    pitch_error_I = MAX_PITCH_I;
  }
  else if (pitch_error_I < -MAX_PITCH_I)
  {
    pitch_error_I = -MAX_PITCH_I;
  }
  // Limit roll integral term
  if (roll_error_I > MAX_ROLL_I)
  {
    roll_error_I = MAX_ROLL_I;
  }
  else if (roll_error_I < -MAX_ROLL_I)
  {
    roll_error_I = -MAX_ROLL_I;
  }

  // PID - Controller for Pitch and Roll
  // P - Controller for Yaw
  pwm_0 = desired_thrust - pitch_error*P_PITCH + imu_data[0]*D_PITCH - pitch_error_I + roll_error*P_ROLL - imu_data[1]*D_ROLL + roll_error_I + yaw_error_velocity*P_YAW;
  pwm_1 = desired_thrust + pitch_error*P_PITCH - imu_data[0]*D_PITCH + pitch_error_I + roll_error*P_ROLL - imu_data[1]*D_ROLL + roll_error_I - yaw_error_velocity*P_YAW;
  pwm_2 = desired_thrust + pitch_error*P_PITCH - imu_data[0]*D_PITCH + pitch_error_I - roll_error*P_ROLL + imu_data[1]*D_ROLL - roll_error_I + yaw_error_velocity*P_YAW;
  pwm_3 = desired_thrust - pitch_error*P_PITCH + imu_data[0]*D_PITCH - pitch_error_I - roll_error*P_ROLL + imu_data[1]*D_ROLL - roll_error_I - yaw_error_velocity*P_YAW;
//   pwm_0 = desired_thrust - pitch_error*P_PITCH + imu_data[0]*D_PITCH - pitch_error_I - roll_error*P_ROLL + imu_data[1]*D_ROLL - roll_error_I + yaw_error_velocity*P_YAW;
//   pwm_1 = desired_thrust + pitch_error*P_PITCH - imu_data[0]*D_PITCH + pitch_error_I - roll_error*P_ROLL + imu_data[1]*D_ROLL - roll_error_I - yaw_error_velocity*P_YAW;
//   pwm_2 = desired_thrust + pitch_error*P_PITCH - imu_data[0]*D_PITCH + pitch_error_I + roll_error*P_ROLL - imu_data[1]*D_ROLL + roll_error_I + yaw_error_velocity*P_YAW;
//   pwm_3 = desired_thrust - pitch_error*P_PITCH + imu_data[0]*D_PITCH - pitch_error_I + roll_error*P_ROLL - imu_data[1]*D_ROLL + roll_error_I - yaw_error_velocity*P_YAW;

  // Limit PWM signal at 1000 - 1300s
  if (pwm_0 > PWM_MAX)
  {
    pwm_0 = PWM_MAX;
  }
  else if (pwm_0 < PWM_OFF)
  {
    pwm_0 = PWM_OFF;
  }

  if (pwm_1 > PWM_MAX)
  {
    pwm_1 = PWM_MAX;
  }
  else if (pwm_1 < PWM_OFF)
  {
    pwm_1 = PWM_OFF;
  }

  if (pwm_2 > PWM_MAX)
  {
    pwm_2 = PWM_MAX;
  }
  else if (pwm_2 < PWM_OFF)
  {
    pwm_2 = PWM_OFF;
  }

  if (pwm_3 > PWM_MAX)
  {
    pwm_3 = PWM_MAX;
  }
  else if (pwm_3 < PWM_OFF)
  {
    pwm_3 = PWM_OFF;
  }

  // Set motor speed - Motor number and speed - 1000->1300
  set_PWM(0,pwm_0);
  set_PWM(1,pwm_1);
  set_PWM(2,pwm_2);
  set_PWM(3,pwm_3);
}

// void keyboard_controls(Keyboard keyboard)
// {
// /*
//   Safety checks that stops the student program when any of the following cases are violated/detected:
//   - u (117) - Start the motors
//   - p (112) - Pause the motors
//   - c (99) - Calibrate 
//   - + (43) - Increase thrust
//   - - (45) - Decrease thrust
//   - w (119) - Increase desired pitch
//   - s (115) - Decrease desired pitch
//   - a (97) - Increase roll left
//   - d (100) - Decrease roll right
// */
//   if (keyboard.key_press == 43 && prev_version != keyboard.version)
//   {
//     thrust += 5;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 45 && prev_version != keyboard.version)
//   {
//     thrust -= 5;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 112 && prev_version != keyboard.version)
//   {
//     Flag_pause = 1;
//     printf("\n Pause motors \n\r");
//     // Kill all motors
//     set_PWM(0,PWM_OFF);
//     set_PWM(1,PWM_OFF); 
//     set_PWM(2,PWM_OFF);
//     set_PWM(3,PWM_OFF);
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 99 && prev_version != keyboard.version)
//   {
//     printf("\n Calibrate IMU \n\r");
//     // Kill all motors
//     set_PWM(0,PWM_OFF);
//     set_PWM(1,PWM_OFF); 
//     set_PWM(2,PWM_OFF);
//     set_PWM(3,PWM_OFF);
//     delay(100);
//     // Reset all calibration values
//     x_gyro_calibration = 0.0;
//     y_gyro_calibration = 0.0;
//     z_gyro_calibration = 0.0;
//     roll_calibration = 0.0;
//     pitch_calibration = 0.0;
//     accel_z_calibration = 0.0;
//     // Calibrate IMU
//     calibrate_imu();
//     // Set integral windup back to 0
//     pitch_error_I = 0.0;
//     roll_error_I = 0.0;
//     desired_pitch = 0.0;
//     desired_roll = 0.0;
//     thrust = NEUTRAL_THRUST;
//     prev_version = keyboard.version;
//     printf("\n Done calibrating IMU \n\r");
//   }
//   else if (keyboard.key_press == 117 && prev_version != keyboard.version)
//   {
//     printf("\n Unpause motors \n\r");
//     Flag_pause = 0;
//     // thrust = NEUTRAL_THRUST;
//     prev_version = keyboard.version;
//   }
//   // Pitch
//   else if (keyboard.key_press == 119 && prev_version != keyboard.version)
//   {
//     desired_pitch += 1.0;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 115 && prev_version != keyboard.version)
//   {
//     desired_pitch -= 1.0;
//     prev_version = keyboard.version;
//   }
//   // ROLL
//   else if (keyboard.key_press == 97 && prev_version != keyboard.version)
//   {
//     desired_roll += 1.0;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 100 && prev_version != keyboard.version)
//   {
//     desired_roll -= 1.0;
//     prev_version = keyboard.version;
//   }
// }

void joystick_control(Keyboard keyboard)
{
/*
  Safety checks that stops the student program when any of the following cases are violated/detected:
  - u (34) - Start the motors
  - p (33) - Pause the motors
  - c (35) - Calibrate 
*/
//   if (keyboard.key_press == 43 && prev_version != keyboard.version)
//   {
//     thrust += 5;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 45 && prev_version != keyboard.version)
//   {
//     thrust -= 5;
//     prev_version = keyboard.version;
//   }
  if (keyboard.keypress == 33)
  {
    Flag_pause = 1;
    printf("\n Pause motors \n\r");
    // Kill all motors
    set_PWM(0,PWM_OFF);
    set_PWM(1,PWM_OFF); 
    set_PWM(2,PWM_OFF);
    set_PWM(3,PWM_OFF);
    // prev_version = keyboard.version;
  }
  else if (keyboard.keypress == 35)
  {
    printf("\n Calibrate IMU \n\r");
    // Kill all motors
    set_PWM(0,PWM_OFF);
    set_PWM(1,PWM_OFF); 
    set_PWM(2,PWM_OFF);
    set_PWM(3,PWM_OFF);
    delay(100);
    // Reset all calibration values
    x_gyro_calibration = 0.0;
    y_gyro_calibration = 0.0;
    z_gyro_calibration = 0.0;
    roll_calibration = 0.0;
    pitch_calibration = 0.0;
    accel_z_calibration = 0.0;
    // Calibrate IMU
    calibrate_imu();
    // Set integral windup back to 0
    pitch_error_I = 0.0;
    roll_error_I = 0.0;
    desired_pitch = 0.0;
    desired_roll = 0.0;
    desired_thrust = NEUTRAL_THRUST;
    printf("\n Done calibrating IMU \n\r");
  }
  else if (keyboard.keypress == 34)
  {
    printf("\n Unpause motors \n\r");
    Flag_pause = 0;
    // thrust = NEUTRAL_THRUST;
    // prev_version = keyboard.version;
  }
  // Pitch
//   else if (keyboard.key_press == 119 && prev_version != keyboard.version)
//   {
//     desired_pitch += 1.0;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 115 && prev_version != keyboard.version)
//   {
//     desired_pitch -= 1.0;
//     prev_version = keyboard.version;
//   }
//   // ROLL
//   else if (keyboard.key_press == 97 && prev_version != keyboard.version)
//   {
//     desired_roll += 1.0;
//     prev_version = keyboard.version;
//   }
//   else if (keyboard.key_press == 100 && prev_version != keyboard.version)
//   {
//     desired_roll -= 1.0;
//     prev_version = keyboard.version;
//   }
}

void get_joystick(Keyboard keyboard)
{
// /*
//   Update global variables for the joystick
// */
    // Update joystick values
    joy_pitch = keyboard.pitch;
    joy_roll = keyboard.roll;
    joy_yaw = keyboard.yaw;
    joy_thrust = keyboard.thrust;
    joy_keypress = keyboard.keypress;
    sequence_num = keyboard.sequence_num;

    desired_pitch = ((float)(2.0 * PITCH_MAX)/(JOY_HIGH - JOY_LOW))*(joy_pitch-128);
    desired_roll = -((float)(2.0 * ROLL_MAX)/(JOY_HIGH - JOY_LOW))*(joy_roll-128);
    desired_yaw_velocity = ((float)(2.0 * YAW_MAX)/(JOY_HIGH - JOY_LOW))*(joy_yaw-128);
    desired_thrust = ((float)(PWM_MAX - PWM_OFF)/(JOY_HIGH - JOY_LOW))*(joy_thrust-128) + NEUTRAL_THRUST;
    printf("desired thrust: %f  desired_pitch: %f  desired_roll: %f  desired_yaw_velocity: %f  heartbeat: %d\n", desired_thrust, desired_pitch, desired_roll, desired_yaw_velocity, sequence_num);
}
//gcc -o spin spin.cpp -lwiringPi -lm
void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
     // printf("I2C connected \n\r");
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}

void set_PWM( uint8_t channel, float time_on_us)
{
  if(1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}
