#include "WPILib.h"
#include <math.h>

//cRIO Modules
const int	DIGITAL_SIDECAR_PORT = 1,
			ANALOG_MODULE_PORT = 1,
			SOLENOID_MODULE_PORT = 1;

//Joystick Ports
const int	JOYSTICK_0_PORT = 1,
			JOYSTICK_1_PORT = 2,
			JOYSTICK_2_PORT = 3;

//Motor Channels
const int	LEFT_DRIVE_CHANNEL = 2,
			RIGHT_DRIVE_CHANNEL = 3,

			SHIFTER_CHANNEL = 5,

			CONVEYOR_CHANNEL = 7,

			TOP_LAUNCHER_CHANNEL = 1, //TODO: Change to 7
			BOTTOM_LAUNCHER_CHANNEL = 9; //TODO: Change to 8


//Digital Channels
const int	LAUNCH_ENCODER_TOP_CHANNEL_A = 7,
			LAUNCH_ENCODER_TOP_CHANNEL_B = 5,
			LAUNCH_ENCODER_BOTTOM_CHANNEL = 8; //URGENT: Do we only need one, and if so, one for both?

//Analog Channels
const int 	GYRO_CHANNEL = 1,
			ULTRASONIC_CHANNEL = 3,

			BALL_SENSOR_0_CHANNEL = 5,
			BALL_SENSOR_1_CHANNEL = 7;


//Joystick Button Constants
const int	TURNING_BUTTON = 2;

//Other Constants
const int TRIGGER_ITERATIONS = 8; //TODO: Put the number of times the ball acquisition code will be run befoure a ball passes the sensor here

const float	SHIFT_SERVO_MIN = 0.1, //High gear
			SHIFT_SERVO_MAX = 0.75; //Low gear

const double MAX_MOTOR_RPM = 2000.0, //TODO: Get actual value for the maximum RPM
			PULSES_PER_REVOLUTION = 250.0, //pulses for revolution on the endoders
			BALL_SENSOR_THRESHOLD = 1.2;

//PID Constants
const float PID_P = 0.01,
			PID_I = 0.001,
			PID_D = 0.0001;

class MainRobot : public IterativeRobot {
	//TODO: Change Jaguars to Victors
	Victor 	*leftDrive,
			*rightDrive,
			*conveyor;

	Servo 	*shifter;

	Jaguar 	*topLauncher,
			*bottomLauncher;

	Encoder *launchEncoderTop;
			//*launchEncoderBottom;

	PIDController	*shooterTopControl,
					*shooterBottomControl;

	Joystick 	*stick0,
				*stick1,
				*stick2;

	AnalogChannel	*ultrasonic,
					*ballSensor0,
					*ballSensor1;

	Gyro *gyro;

	DriverStationLCD 	*dashboardLCD;
	Dashboard 			*dashboard;

	//global variables used to keep track of the number of balls
	int ballCount;

	bool ballSensor0HadBall;

	double throttle;
public:
	MainRobot(void) {
		leftDrive		= new Victor(DIGITAL_SIDECAR_PORT,LEFT_DRIVE_CHANNEL);
		rightDrive		= new Victor(DIGITAL_SIDECAR_PORT,RIGHT_DRIVE_CHANNEL);
		conveyor		= new Victor(DIGITAL_SIDECAR_PORT,CONVEYOR_CHANNEL);

		shifter			= new Servo(DIGITAL_SIDECAR_PORT, SHIFTER_CHANNEL);

		topLauncher		= new Jaguar(DIGITAL_SIDECAR_PORT,TOP_LAUNCHER_CHANNEL);
		bottomLauncher	= new Jaguar(DIGITAL_SIDECAR_PORT,BOTTOM_LAUNCHER_CHANNEL);

		launchEncoderTop 	= new Encoder(LAUNCH_ENCODER_TOP_CHANNEL_A, LAUNCH_ENCODER_TOP_CHANNEL_B, false, Encoder::k1X);
		//launchEncoderBottom	= new Encoder(LAUNCH_ENCODER_BOTTOM_CHANNEL);
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

		throttle = 0;

		SetPeriod( 0.0 );
	}


	//New Methods
	double GetTotalShooterRPM( double distance, double relativeHoopHeight ) {
		double totalBottom = (2.8763 * distance) - (2 * relativeHoopHeight);
		if ( totalBottom < 0 ) {
			throw 1;
		}
		return ((2594.59 * distance)/sqrt(totalBottom));
	}

	void SetShooterRPM( double distance, double relativeHoopHeight ) {
		double totalRPM = 0.0;
		try {
			GetTotalShooterRPM( distance, relativeHoopHeight );
		}
		catch( int e ) {
			dashboardLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Shot is impossible");
			return;
		}
		//TODO: Set shooter speed based on totalRPM
	}
	//Update the drive motor speeds on the robot from stick0 and stick1 positions
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

	//Sets the gear shifting servos to the values that they currently aren't
	void HighGear() {
		shifter->Set(SHIFT_SERVO_MIN);
	}

	void LowGear() {
		shifter->Set(SHIFT_SERVO_MAX);
	}

	//This counts balls, and runs the conveyor
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

	void ResetBallCount( int newBallCount ) {
		ballCount = newBallCount;
	}

	void PIDSetLauncherSpeed() {
		if (stick2->GetTrigger())
		{
			shooterTopControl->SetSetpoint(-0.5);
		}
		else if(stick2->GetRawButton(2)){
			shooterTopControl->SetSetpoint(0.0);

		}

	}

	void PIDSetLow() {
		shooterTopControl->SetSetpoint(0.0);
	}

	void PIDSetHigh() {
		shooterTopControl->SetSetpoint(0.5);
	}

	void Shoot( int hoop ) {
		;
	}

	//Required Methods

	void AutonomousInit(void){
		;
	}

	void AutonomousPeriodic(void) {
		;
	}

	void AutonomousContinuous(void) {
		;
	}

	void TeleopInit(void) {
		launchEncoderTop->Start();
		launchEncoderTop->Reset();
		//shooterControl->Enable();
		HighGear();
		shooterTopControl->Enable();
	}

	void TeleopPeriodic(void) {
		dashboardLCD->Clear();
		dashboardLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Encoder Count: %d",launchEncoderTop->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Victor Rate: %g", topLauncher->Get());
		//dashboardLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Encoder Rate: %g",launchEncoderTop->GetRate());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Ultrasonic: %d",ultrasonic->GetValue());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Gearshift: %g",shifter->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line5, 1, "GyroAngle: %g",gyro->GetAngle());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Throttle: %g",throttle);
		dashboardLCD->UpdateLCD();
		//PIDSetLauncherSpeed();
		if(stick2->GetTrigger()) PIDSetHigh();
		else if(stick2->GetRawButton(2)) PIDSetLow();
		//topLauncher->Set(stick2->GetY());
	}

	void TeleopContinuous(void) {
		throttle = stick2->GetThrottle();
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

START_ROBOT_CLASS(MainRobot);
