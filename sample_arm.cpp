#include "Arduino.h"
#include <Wire.h>
#include <sample_arm.h>
int status;
int steps,step,step_direction;
int accel_counter = 0;
long last_time, lockout_time;
float angle_lower, angle_upper,delta_upper,delta_lower;
float lower_array[10];
float upper_array[10];
int bufferIndex = 0;
int rate_lower, rate_upper;

//status values
//max time = 90 seconds, to protect the arm from struggling too much

//////////////////////////////////////////////////////////////////////
//@I SHARED FUNCTIONS //////////////////////////////////////////////////-
////////////////////////////////////////////////////////////////////
//should be initiated at the beginning of the sketch
void sample_arm::begin(){
	Wire.begin(PIN_SDA,PIN_SCL);
	last_time = millis();
	lockout_time = millis();
	//initialize accelerometers
	start_accel(I2C_ADDR_UPPER);
	start_accel(I2C_ADDR_LOWER);
	//initialize motor drive
	reset_PWM_driver();
  //read program here in the future
	status = 0;
	step = 0;
	steps  = 3;
	step_direction = 1;
	program[0].angle_lower = 111.4;
	program[0].angle_upper = 68.7;
	program[1].angle_lower = 69.3;
	program[1].angle_upper = 27.2;
	program[2].angle_lower = 57.9;
	program[2].angle_upper = -57;

	//get initial accelerometer reading
	for(int k = 0; k<10;k++){
		measure_accel(1);
		measure_accel(2);
		delay(10);
	}
	//get initial angle readings
	update_angles();
}


//////////////////////////////////////////////////////////////////
/////Accelerometer methods
/////////////////////////////////////////////////////////////////
//initialize accelerometer
void sample_arm::start_accel(byte addr){
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

//read accelerometers
void sample_arm::measure_accel(int segment){
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  int minVal=265;
  int maxVal=402;
  int MPU_addr;
  if(segment==1){
    MPU_addr = I2C_ADDR_LOWER;
  }else{
    MPU_addr = I2C_ADDR_UPPER;
  }
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  if(segment==1){
     lower_array[accel_counter/2]= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI) - 180;
  }else{
     upper_array[accel_counter/2]= 180 - RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  }
	accel_counter++;
	if(accel_counter > 19) accel_counter = 0;
}

//summarize accelerometer arrays
float sample_arm::accel_summary(float yy[]){
    double ym = 0.0;
    for(int k = 0; k<10;k++){
      ym = ym + yy[k];
    }
    ym = ym/10.0;
    return(ym);
}

void sample_arm::update_angles(){
	measure_accel(1);
	measure_accel(2);
	angle_lower = accel_summary(lower_array);
	angle_upper = accel_summary(upper_array);
}
/////////////////////////////////////////////////////////////////
///Motor methods
////////////////////////////////////////////////////////////////
void sample_arm::reset_PWM_driver(){
  Wire.beginTransmission(I2C_ADDR_PWM_DRIVER);
  Wire.write(0x0);
  Wire.write(0xa1);
  Wire.endTransmission();
  control_motor(0,0); //turn off all motors
}

void sample_arm::stop_arm(){
	control_motor(0,0); //turn off all motors
}


void sample_arm::control_motor(byte port_number, byte level) {
  //level = 0 turn all motors off, port_number must also be set zero.
  //level = 1 turn motor off
  //level 40 to 100 sets the motor level
	//set the default levels to turn the motor off
  int on_bytes = 0;
  int off_bytes = 4096;
  byte valve_addr = 0x6;
	int relevel = int(level);
	//set the address to turn off all motors
	if(port_number == 0 && level == 0){
		valve_addr = 0xFA;
	}
	//don't let the level fall below 40, otherwise the motor will stall
	if(level > 1 && level < 40) level = 40;
	if(level > 100) level = 100;
	//translate the level here
  if(level >= 40){
   on_bytes = 0;
	 relevel = relevel*4095;
	 relevel = relevel/100;
   off_bytes = relevel;
  }
  Wire.beginTransmission(I2C_ADDR_PWM_DRIVER);
  Wire.write(valve_addr + 4*port_number);
  Wire.write(on_bytes);
  Wire.write(on_bytes>>8);
  Wire.write(off_bytes);
  Wire.write(off_bytes>>8);
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////
//calculate direction and rates
//////////////////////////////////////////////////////////////
void sample_arm::calc_motor_settings(){
	float delta_ratio;
	//update angle readings
	update_angles();
	delta_lower = program[step].angle_lower - angle_lower;
	delta_upper = program[step].angle_upper - angle_upper;
	//lockout logic
	//if still in lockout then just return
	if(millis() < (lockout_time + 500L)) return(void());
  //if both are within 2 degrees, then take a pause if it's been a second since the last pause
	if(abs(delta_lower)<2.0f && abs(delta_upper)<2.0f && millis() > (lockout_time + 1500L)){
		lockout_time = millis();
		stop_arm();
		return(void());
	}

	//if moving lower arm more than 5 degrees, then estimate effect on upper arm
	//estimate current actuator length
	if(delta_lower>5.0f){
		//this method doesn't work too well so it might needs to be constrained
		float actuator_length,future_upper_angle;
		actuator_length = calculate_actuator_length();
		future_upper_angle = calc_future_upper_angle(program[step].angle_lower,actuator_length);
		delta_upper = program[step].angle_upper - future_upper_angle;
	}
	//set speeds so that the arms reach their positions at about the same time
	delta_ratio = max(1.0,abs(delta_upper))/max(1.0,abs(delta_lower)) * LOWER_DEGREES_PER_SECOND/UPPER_DEGREES_PER_SECOND;
	//if the value > 1 then upper is moving faster, < 1 then lower is faster
	rate_upper = 100;
	rate_lower = 100;
	if(delta_ratio<1){
		rate_upper = int(delta_ratio*100.0f);
		if(rate_upper<40) rate_upper = 40; //avoid stalling motors
	}else{
		rate_lower = int(100.0f/delta_ratio);
		if(rate_lower<40) rate_lower = 40;
	}
  //set motor speeds here
	//if motors within 2 degrees then slow down
	if(abs(delta_lower)<2.0f && abs(delta_upper)<2.0f){
		rate_upper = min(40,rate_upper);
		rate_lower = min(40,rate_lower);
	}
	//if within 1 degree then just stop
	if(abs(delta_upper)< 1.0f) rate_upper = 0;
	if(abs(delta_lower)<1.0f) rate_lower = 0;
	//if both are within a degree then stop
	if(rate_lower==0 && rate_upper==0){
		step += step_direction;
		if(step > (steps-1)) step = steps - 1;
		if(step < 0) step = 0;
	};
	if(delta_lower >= 0.0f){
			control_motor(4,0); //make sure that the other direction is turned off
			control_motor(5,rate_lower);
		}else{
			control_motor(5,0);
			control_motor(4,rate_lower);
		}
	if(delta_upper >= 0.0f){
			control_motor(7,0);
			control_motor(6,rate_upper);
		}else{
			control_motor(6,0);
			control_motor(7,rate_upper);
	}
}

float sample_arm::calculate_actuator_length(){
	float a3 = 180.0f - angle_lower + angle_upper;
  float s3 = sq(LOWER_MOMENT_LENGTH) + sq(UPPER_MOMENT_LENGTH);
	s3 = s3 - 2*LOWER_MOMENT_LENGTH*UPPER_MOMENT_LENGTH*cos(a3/RAD_TO_DEG);
	s3 = sqrt(s3);
  return(s3);
}

float sample_arm::calc_future_upper_angle(float future_lower_angle, float actuator_length){
	float a3 = (sq(UPPER_MOMENT_LENGTH) + sq(LOWER_MOMENT_LENGTH) - sq(actuator_length))/(2*LOWER_MOMENT_LENGTH*UPPER_MOMENT_LENGTH);
	a3 = acos(a3);
  a3 = a3*RAD_TO_DEG;
  float future_upper_angle = a3-180 + future_lower_angle;
  return(future_upper_angle);
}

///interface methods
String sample_arm::report(char type){
	String reply = "none";
	if(type=='a'){
		update_angles();
		reply = "upper a = " + String(angle_upper) + " lower a = " + String(angle_lower);
	}
	if(type=='r') reply = "upper rate = " + String(rate_upper) + " lower rate = " + String(rate_lower);
	if(type=='d') reply = "upper delta = " + String(delta_upper) + " lower delta = " + String(delta_lower);
	if(type=='x'){
		step_direction = 0 - step_direction;
		reply = "direction = " + String(step_direction);
	}
	return(reply);
}
