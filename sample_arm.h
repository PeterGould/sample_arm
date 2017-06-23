
#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
//clock values
#define I2C_ADDR_LOWER   0x68
#define I2C_ADDR_UPPER   0x69
#define I2C_ADDR_PWM_DRIVER   0x40
#define MAX_NO_PROGRESS 2000
//minimum motor speed to avoid stalling
#define MIN_MOTOR_SPEED 30
//lower segment needs more motor when extended
#define MIN_MOTOR_SPEED_EXTENDED 50
//motor ports
#define MOTOR1_UP 0
#define MOTOR1_DOWN 1
#define MOTOR2_UP 3
#define MOTOR2_DOWN 2

#define PIN_SDA 5
#define PIN_SCL 4
#define LOWER_DEGREES_PER_SECOND 3.25
#define UPPER_DEGREES_PER_SECOND 5.55
#define LOWER_MOMENT_LENGTH 17.0
#define UPPER_MOMENT_LENGTH 5.25
/**
* A class to control the Olympia Circuits Valve Controller
*/
class sample_arm
{
	private:

	public:
    struct spot{
    float angle_lower,angle_upper;
    int limits[8];
    };

		spot program[10];
		int status;
		int steps,step,step_direction;
		int accel_counter = 0;
		long last_time, lockout_time, last_progress_time;
		float angle_lower, angle_upper,delta_upper,delta_lower,last_delta;
		float lower_array[10];
		float upper_array[10];
		int bufferIndex = 0;
		int rate_lower, rate_upper;
		String unit_name;

		/**
       * Initializes an instance of the SODA class. Should be called in each sketch before any other SODA functions.
       */
		void begin();

		void start_accel(byte addr);

		void measure_accel(int segment);

		float accel_summary(float yy[]);

		void update_angles();

		void reset_PWM_driver();

		void stop_arm();

		void control_motor(byte port_number, byte level);

		void calc_motor_settings();

		float calculate_actuator_length();

		float calc_future_upper_angle(float future_lower_angle, float actuator_length);

		void reset_progress();

		String get_name();

		void set_name(String new_name);

		void write_program(String a_program);

		bool read_program();

		void run_program();

		String report(char type);

		void move_to(float lower, float upper);


	};
