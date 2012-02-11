#include "WPILib.h"
#include <math.h>

//cRIO Modules
const int	DIGITAL_SIDECAR_PORT = 1, ///< cRIO module port set for the Digital Sidecar
			ANALOG_MODULE_PORT = 1, ///< cRIO module port set for the Analog Module
			SOLENOID_MODULE_PORT = 1; ///< cRIO module port set for the Solenoid Module

//Joystick Ports
const int	JOYSTICK_0_PORT = 1, ///< First Joystick USB port
			JOYSTICK_1_PORT = 2, ///< Second Joystick USB port
			JOYSTICK_2_PORT = 3; ///< Third Joystick USB port

//Motor Channels
const int	LEFT_DRIVE_CHANNEL = 1,	///< Digital Sidecar output channel for the left drive motors
			RIGHT_DRIVE_CHANNEL = 3, ///< Digital Sidecar output channel for the right drive motors

			SHIFTER_CHANNEL = 5, ///< Digital Sidecar output channel for the servos that shift motor gears

			CONVEYOR_CHANNEL = 7, ///< Digital Sidecar output channel for the conveyor motors

			TOP_LAUNCHER_CHANNEL = 10, ///< Digital Sidecar output channel for the top launch motor @todo Change to 7
			BOTTOM_LAUNCHER_CHANNEL = 9; ///< Digital Sidecar output channel for the bottom launch motor @todo Change to 8


//Digital Channels
const int	LAUNCH_ENCODER_TOP_CHANNEL_A = 7, ///< Digital Sidecar input channel for the top launch motor encoder channel A
			LAUNCH_ENCODER_TOP_CHANNEL_B = 5, ///< Digital Sidecar input channel for the top launch motor encoder channel B
			LAUNCH_ENCODER_BOTTOM_CHANNEL_A = 8, ///< Digital Sidecar input channel for the bottom launch motor encoder channel A
			LAUNCH_ENCODER_BOTTOM_CHANNEL_B = 3; ///< Digital Sidecar input channel for the bottom launch motor encoder channel B

//Analog Channels
const int 	GYRO_CHANNEL = 1, ///< Analog Module channel for the gyroscopic sensor
			ULTRASONIC_CHANNEL = 3, ///< Analog Module channel for the ultrasonic sensor

			BALL_SENSOR_0_CHANNEL = 5, ///< Analog Module channel for the bottom proximity sensor to detect balls in the conveyor
			BALL_SENSOR_1_CHANNEL = 7; ///< Analog Module channel for the top proximity sensor to detect balls in the conveyor


//Joystick Button Constants
const int	TURNING_BUTTON = 2; ///< Joystick button to activate the bot's turning mode

//Other Constants
const float	SHIFT_SERVO_MIN = 0.1, ///< The gear-shifting servo setting for high gear
			SHIFT_SERVO_MAX = 0.75; ///< The gear-shifting servo setting for low gear

const double MAX_MOTOR_RPM = 2000.0, ///< @todo Get actual value for the maximum RPM
			PULSES_PER_REVOLUTION = 250.0, ///< Pulses per motor revolution emitted by the encoders
			BALL_SENSOR_THRESHOLD = 1.2, ///< The threshold that the proximity sensors must be above to indicate the presence of the ball
			AIM_TURNING_SPEED = 0.25, ///< The speed to turn at when aiming the shooter
			PI = 3.14159, ///< Pi to 6 significant figures
			MIN_TOP_LAUNCHER_RPM = 500; ///< The minimum rpm to run the top mmotor at to launch

//PID Constants
const float PID_P = 0.01, ///< Proportional value for the launch motor PID controllers
			PID_I = 0.001, ///< Integral value for the launch motor PID controllers
			PID_D = 0.0001; ///< Derivative value for the launch motor PID controllers

// Field Dimensions in inches
const float TARGET_WIDTH = 24.0, ///< The width of the retro-reflective target
			TARGET_HEIGHT = 18.0, ///< The height of the retro-reflective target
			HOOP_DISTANCE = 15.0; ///< The length of the basketball hoop from the wall
/**
 * @class MainRobot
 * @brief This is the main class that controls the robot
 *
 * This is the main class that controls the robot, based
 * off of the IterativeRobot class provided by WPILib.  This
 * class controls all functions of the robot except image
 * processing.  This class defines all of the components of
 * the robot, as well as all methods that control the robot.
 *
 * @author Jack Palen
 * @author Wil Thomason
 * @author Will Kunkel
 * @author Nyle Rodgers
 * @author Andy Zhu
 * @author Robert Jones
 */
class MainRobot : public IterativeRobot {
	Victor 	*leftDrive, ///< Victors for the left drive motors
			*rightDrive, ///< Victors for the right drive motors
			*conveyor; ///< Victors for the conveyor motors

	Servo 	*shifter; ///< Servos for the gear shifters

	Jaguar 	*topLauncher, ///< Jaguar for the top launch motor
			*bottomLauncher; ///< Jaguar for the bottom launch motor

	Encoder *launchEncoderTop; ///< Encoder on the top launch motor
			//*launchEncoderBottom; ///< Encoder on the bottom launch motor

	PIDController	*shooterTopControl, ///< PID controller for the top launch motor
					*shooterBottomControl; ///< PID controller for the bottom launch motor

	Joystick 	*stick0, ///< The first Joystick
				*stick1, ///< The second Joystick
				*stick2; ///< The third Joystick

	AnalogChannel	*ultrasonic, ///< Analog channel for the ultrasonic sensor
					*ballSensor0, ///< Analog channel for the bottom lift proximity sensor
					*ballSensor1; ///< Analog channel for the top lift proximity sensor

	Gyro *gyro; ///< The gyroscopic sensor

	DriverStationLCD 	*dashboardLCD; ///< The output to the dashboard's user output dialogue
	Dashboard 			*dashboard; ///< The driver station's dashboard

	// Global variables used to keep track of the number of balls
	int ballCount; ///< The current amount of balls that has passed the bottom conveyor proximity sensor and have not been fired

	bool ballSensor0HadBall; ///< Whether the bottom proximity sensor had a ball in front of it the last time it was checked

public:

	/**
	 * @brief Constructor for the MainRobot class
	 *
	 * This initializes all of the variables for the class,
	 * and sets up all of the components of the robot.  It
	 * also sets the period for the periodic functions.
	 *
	 * @author Jack Palen
	 * @author Wil Thomason
	 * @author Will Kunkel
	 * @author Nyle Rodgers
	 * @author Andy Zhu
	 */
	MainRobot(void) {
		leftDrive		= new Victor(DIGITAL_SIDECAR_PORT,LEFT_DRIVE_CHANNEL);
		rightDrive		= new Victor(DIGITAL_SIDECAR_PORT,RIGHT_DRIVE_CHANNEL);
		conveyor		= new Victor(DIGITAL_SIDECAR_PORT,CONVEYOR_CHANNEL);

		shifter			= new Servo(DIGITAL_SIDECAR_PORT, SHIFTER_CHANNEL);

		topLauncher		= new Jaguar(DIGITAL_SIDECAR_PORT,TOP_LAUNCHER_CHANNEL);
		bottomLauncher	= new Jaguar(DIGITAL_SIDECAR_PORT,BOTTOM_LAUNCHER_CHANNEL);

		launchEncoderTop 	= new Encoder(LAUNCH_ENCODER_TOP_CHANNEL_A, LAUNCH_ENCODER_TOP_CHANNEL_B, false, Encoder::k1X);
		//launchEncoderBottom	= new Encoder(LAUNCH_ENCODER_BOTTOM_CHANNEL_A, LAUNCH_ENCODER_BOTTOM_CHANNEL_B, false, Encoder::k1X);
		launchEncoderTop->SetDistancePerPulse(1.0/PULSES_PER_REVOLUTION);

		stick0			= new Joystick(JOYSTICK_0_PORT);
		stick1			= new Joystick(JOYSTICK_1_PORT);
		stick2			= new Joystick(JOYSTICK_2_PORT);

		ultrasonic		= new AnalogChannel(ULTRASONIC_CHANNEL);
		ballSensor0		= new AnalogChannel(BALL_SENSOR_0_CHANNEL);
		ballSensor1		= new AnalogChannel(BALL_SENSOR_1_CHANNEL);

		gyro			= new Gyro(ANALOG_MODULE_PORT, GYRO_CHANNEL);

		dashboardLCD	= DriverStationLCD::GetInstance();

		launchEncoderTop->SetPIDSourceParameter(Encoder::kRate);

		shooterTopControl	= new PIDController(PID_P, PID_I, PID_D ,launchEncoderTop,topLauncher);
		shooterTopControl->SetInputRange(-10.0,10.0);
		shooterTopControl->SetOutputRange(-1.0,1.0);

		ballCount = 0;
		ballSensor0HadBall = false;

		SetPeriod( 0.0 );
	}


	/**
	 * @brief Get the total RPM for both shooter motors
	 *
	 * This method returns the toatl RPM for the shooter motors,
	 * given the distance to the target and the hieght of the hoop
	 * relative to the height of the shooter.  This formula was
	 * determined with experimental data, which also showed that the
	 * ratio of the speeds of the motors had a negligible effect on
	 * trajectory, and that the total RPM was all that affected the
	 * shot.
	 *
	 * @param distance The distance to the target
	 * @param relativeHoopHeight The height of the target hoop relative to the height of the shooter
	 * @return The total RPM for both shooter motors
	 * @author Will Kunkel
	 */
	float GetTotalShooterRPM( double distance, double relativeHoopHeight ) {
		float totalBottom = (2.8763 * distance) - (2 * relativeHoopHeight);
		if ( totalBottom < 0 ) {
			throw 1;
		}
		return ((2594.59 * distance)/sqrt(totalBottom));
	}

	/**
	 * @brief Set the RPM of the shooter motors
	 *
	 * This method sets the RPM of both shooter motors based on the
	 * the distance to the target and the height of the target.
	 * This method calls GetTotalShooterRPM() with its arguments and
	 * then sets the top motor to at least a minimum speed and the bottom
	 * motor to the highest speed possible to generate backspin.
	 *
	 * @param distance
	 * @param relativeHoopHeight
	 * @author Will Kunkel
	 */
	void SetShooterRPM( double distance, double relativeHoopHeight ) {
		float totalRPM = 0.0;
		try {
			totalRPM = GetTotalShooterRPM( distance, relativeHoopHeight );
		}
		catch( int e ) {
			dashboardLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Shot is impossible");
			return;
		}
		float 	topShooterRPM = 0.0,
				bottomShooterRPM = 0.0;

		if ( totalRPM > MIN_TOP_LAUNCHER_RPM ) {
			topShooterRPM = MIN_TOP_LAUNCHER_RPM;
			totalRPM -= MIN_TOP_LAUNCHER_RPM;
			if( totalRPM > MAX_MOTOR_RPM ) {
				bottomShooterRPM = MAX_MOTOR_RPM;
				totalRPM -= MAX_MOTOR_RPM;
				topShooterRPM += totalRPM;
			}
			else {
				bottomShooterRPM = totalRPM;
			}
		}
		else {
			dashboardLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Shot is impossible");
			return;
		}
		shooterTopControl->SetSetpoint(topShooterRPM);
		shooterBottomControl->SetSetpoint(bottomShooterRPM);
	}

	/**
	 * @brief Drive the robot
	 *
	 * Drive the robot based on the y-values from the first and second
	 * joysticks.  Alternatively, if the joystick turning button is pressed,
	 * instead turn the robot in place based upon the x-values from the
	 * second joystick.  In addition, change to high gear if the second
	 * joystick trigger is pressed and low gear if the first joystick trigger
	 * is pressed.
	 *
	 * @author Jack Palen
	 */
	void Drive(void) {
		float left		= stick0->GetY();
		float right		= stick1->GetY();
		float turning	= stick1->GetX();

		if (stick1->GetTrigger()) {
			HighGear();
		}
		else if (stick0->GetTrigger()) {
			LowGear();
		}

		if (stick0->GetRawButton(TURNING_BUTTON)) { //if you hold this button, the robot will only rotate, with stick1x
			leftDrive->Set(turning);
			rightDrive->Set(-turning);
		}
		else {
			leftDrive->Set(left);
			rightDrive->Set(right);
		}
	}

	/**
	 * @brief Shift the drive motors to high gear
	 *
	 * Shift the drive motors to high gear.
	 *
	 * @author Jack Palen
	 */
	void HighGear(void) {
		shifter->Set(SHIFT_SERVO_MIN);
	}

	/**
	 * @brief Shift the drive motors to low gear
	 *
	 * Shift the drive motors to low gear.
	 *
	 * @author Jack Palen
	 */
	void LowGear(void) {
		shifter->Set(SHIFT_SERVO_MAX);
	}

	/**
	 * @brief Run conveyor and count balls
	 *
	 * Run the conveyor based on the y-values from the third joystick
	 * and count the balls that pass the bottom proximity sensor.  If
	 * the conveyor is running forward, increase the ball count.  If it
	 * is running in reverse, decrease the ball count.
	 *
	 * @author Will Kunkel
	 * @author Nyle Rodgers
	 */
	void RunConveyor(void) {
		conveyor->Set(stick2->GetY());
		if((ballSensor0->GetVoltage() >= BALL_SENSOR_THRESHOLD)
				&& !ballSensor0HadBall
				&& (conveyor->Get() > 0)) {
			ballCount++;
			ballSensor0HadBall = true;
		}
		else if((ballSensor0->GetVoltage() >= BALL_SENSOR_THRESHOLD)
				&& !ballSensor0HadBall
				&& (conveyor->Get() < 0)) {
			ballCount--;
			ballSensor0HadBall = true;
		}
		else {
			ballSensor0HadBall = false;
		}
	}

	/**
	 * @brief Shoot the balls
	 *
	 * This method has yet to be implemented.
	 *
	 * @todo Implement this method
	 * @param hoop
	 */
	void Shoot( int hoop ) {
		;
	}

	//Aiming / Shooting Methods
	/**
	 * @brief Calculate how much the robot must turn to be aimed correctly
	 *
	 * Given the angle between the target and the wall of the arena
	 * of the robot and the distance to the nearest edge of the
	 * targetcalculate the angle the robot must turn to
	 * make the shot.
	 *
	 * @param theta The angle between the wall and the target
	 * @param distance The distance to the nearest edge of the target
	 * @return The angle the robot needs to turn
	 * @author Jack Palen
	 * @author Robert Jones
	 */
	double AngleChange(double theta, double distance) {
		double sin_theta = sin(theta);
		double cos_theta = cos(theta);
		return atan(
				((TARGET_WIDTH/2)
						+ distance*sin_theta
						+ sqrt((-4*distance*cos_theta)
								+ pow(((TARGET_WIDTH/2) + distance*sin_theta),2)))
				/ (2*distance*cos_theta))
				- theta;
	}

	/**
	 * @brief Turn the robot to aim for shooting
	 *
	 * Given an angle in radians, turn the robot such that it is facing
	 * the direction specified by the angle.  Check the image so that
	 * the robot is turning in the correct direction.
	 *
	 * @param angle The angle to turn
	 * @author Jack Palen
	 */
	void AimRobot(float angle) {
		leftDrive->Set(0.0);
		rightDrive->Set(0.0);
		LowGear();
		Wait(.25);
		gyro->Reset();
		while (fabs(gyro->GetAngle() * PI / 180.0) < angle) {
			/*if (targetisonleftsideofvision) { ///@todo Change targetisonleftsideofvision to a usable value
				leftDrive->Set(-AIM_TURNING_SPEED);
				rightDrive->Set(AIM_TURNING_SPEED);
			}
			else {
				leftDrive->Set(AIM_TURNING_SPEED);
				rightDrive->Set(-AIM_TURNING_SPEED);
			}
			*/
		}
		//gyro->Reset();
	}

	//Required Methods

	/// The autonomous initialization method provided by WPILib
	void AutonomousInit(void){
		;
	}

	/// The autonomous periodic method provided by WPILib
	void AutonomousPeriodic(void) {
		;
	}
	/// The autonomous continuos method provided by WPILib
	void AutonomousContinuous(void) {
		;
	}

	/**
	 * @brief The teleop initialization method provided by WPILib
	 *
	 * This method starts the encoders and enables the PID controllers.
	 * It also shifts the robot into high gear.
	 *
	 * @author Jack Palen
	 * @author Wil Thomason
	 */
	void TeleopInit(void) {
		launchEncoderTop->Start();
		launchEncoderTop->Reset();
		//shooterControl->Enable();
		HighGear();
		shooterTopControl->Enable();
	}

	/**
	 * @brief The teleop periodic method provided by WPILib
	 *
	 * Print all values to the dashboard.
	 *
	 * @author Jack Palen
	 * @author Will Kunkel
	 */
	void TeleopPeriodic(void) {
		dashboardLCD->Clear();
		dashboardLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Encoder Count: %d",launchEncoderTop->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Victor Rate: %g", topLauncher->Get());
		//dashboardLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Encoder Rate: %g",launchEncoderTop->GetRate());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Ultrasonic: %d",ultrasonic->GetValue());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Gearshift: %g",shifter->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line5, 1, "GyroAngle: %g",gyro->GetAngle());
		dashboardLCD->UpdateLCD();
		//PIDSetLauncherSpeed();
		//topLauncher->Set(stick2->GetY());
	}

	/**
	 * @brief The teleop continuous method provided by WPILib
	 *
	 * This method drives the motors, activates shooting, switches gears,
	 * and handles all or nearly all driver input.  It primarily calls
	 * other methods.
	 *
	 * @author Jack Palen
	 * @author Wil Thomason
	 * @author Will Kunkel
	 */
	void TeleopContinuous(void) {
		//Drive();
		//shooterControl->SetSetpoint(stick2->GetY());
		//topLauncher->Set(stick2->GetY());

		if (stick1->GetTrigger()) {
			HighGear();
		}
		else if (stick0->GetTrigger()) {
			LowGear();
		}

		if (stick2->GetRawButton(11)) {
			gyro->Reset();
		}
		//topLauncher->Set(stick2->GetY());
		//BottomLauncher->Set(stick2->GetY());
	}
};

START_ROBOT_CLASS(MainRobot); ///< Start the robot code
