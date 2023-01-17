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
#define A                0.02  //Complemetary filter ratio
#define MAX_GYRO_RATE    300   // deg/s
#define MAX_ROLL_ANGLE   45.0  // deg
#define MAX_PITCH_ANGLE  45.0  // deg

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

//week2 addition
struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
};
 
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_keyboard();
void trap(int signal);
void safety_check(Keyboard keyboard);

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

Keyboard* shared_memory;
int run_program=1;
 
int main (int argc, char *argv[])
{
    setup_imu();
    calibrate_imu();
    setup_keyboard();
    signal(SIGINT, &trap);

    //to refresh values from shared memory first
    Keyboard keyboard=*shared_memory;

    // Save values to CSV
    // file_p = fopen("common_filter.csv", "w+");
    // float temp_pitch = 0.0;
    // float temp_roll = 0.0;

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
      // temp_pitch += pitch_gyro_delta;
      // temp_roll += roll_gyro_delta;
      // fprintf(file_p, "%f, %f, %f\n", temp_pitch, pitch_angle, filtered_pitch);
      // fprintf(file_p, "%f, %f, %f\n", temp_roll, roll_angle, filtered_roll);

      // to refresh values from shared memory first
      Keyboard keyboard=*shared_memory;

      printf("key_press: %d  heartbeat: %d  version: %d\n", keyboard.key_press, keyboard.heartbeat, keyboard.version);
      safety_check(keyboard);

    }

    // Save values to CSV
    // fclose(file_p);

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
  printf("\nending program - control C was pressed - Kill Motors\n\r");
  run_program=0;
}

void safety_check(Keyboard keyboard)
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

  if (hearbeat_prev != keyboard.heartbeat)
  { // Reset previous heartbeat time stamp if a new heartbeat is detected.
    hearbeat_prev = keyboard.heartbeat;
    time_prev_heartbeat = time_curr_heartbeat;
  }
  else if (passed_time>0.25)
  { // If the previous heartbeat is the same as the current heartbeat and 0.25s has passed
    // Stop the student from executing.
    printf("Keyboard timedout! (Heartbeat)");
    run_program=0;
  }

  if (keyboard.key_press == 32)
  { // If the keyboards space bar is pressed, then stop the student code.
    printf("\n Space bar was pressed!");
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