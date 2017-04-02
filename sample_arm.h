
#include <Arduino.h>
#include <Wire.h>

//clock values
#define I2C_ADDR_LOWER   0x68
#define I2C_ADDR_UPPER   0x69
#define I2C_ADDR_PWM_DRIVER   0x40
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
    double angle_lower,angle_upper;
    int limits[8];
    };

		spot program[10];
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

		String report(char type);

	};
