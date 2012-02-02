#include "WPILib.h"

//cRIO Modules
const int DIGITAL_SIDECAR_PORT = 1;
const int ANALOG_MODULE_PORT = 1;
const int SOLENOID_MODULE_PORT = 1;

//Joystick Ports
const int JOYSTICK_0_PORT = 1;
const int JOYSTICK_1_PORT = 2;
const int JOYSTICK_2_PORT = 3;

//Motor Channels
const int LEFT_DRIVE_0_CHANNEL = 1;
const int LEFT_DRIVE_1_CHANNEL = 2;
const int RIGHT_DRIVE_0_CHANNEL = 3;
const int RIGHT_DRIVE_1_CHANNEL = 4;

const int LEFT_SHIFTER_CHANNEL = ;
const int RIGHT_SHIFTER_CHANNEL = ;

const int TOP_LAUNCHER_CHANNEL = 7;
const int BOTTOM_LAUNCHER_CHANNEL = 8;

const int CONVEYOR_0_CHANNEL = 6;
const int CONVEYOR_1_CHANNEL = 9;

//Digital Channels
const int LAUNCH_ENCODER_TOP_CHANNEL_A = 7;
const int LAUNCH_ENCODER_TOP_CHANNEL_B = 5;
const int LAUNCH_ENCODER_BOTTOM_CHANNEL = 8;

//Other Constants
const int MAX_MOTOR_RPM = 200; //TODO: Get actual value for the maximum RPM
const int TURNING_BUTTON = 2; which joystick button toggles turning mode

const float SHIFT_SERVO_MIN = 0.0;
const float SHIFT_SERVO_MAX = 1.0;
const float SHIFT_SERVO_ERROR = 0.1;

class MainRobot : public IterativeRobot {
	//TODO: Change Jaguars to Victors
	Victor *leftDrive0;
	Victor *leftDrive1;
	Victor *rightDrive0;
	Victor *rightDrive1;
	Servo *leftShifter;
	Servo *rightShifter;
	Jaguar *topLauncher;
	Jaguar *bottomLauncher;
	Victor *conveyor0;
	Victor *conveyor1;
	Encoder *launchEncoderTop;
	PIDController *shooterControl;
	//Encoder *launchEncoderBottom;
	Joystick *stick0;
	Joystick *stick1;
	Joystick *stick2;
	DriverStationLCD *dashboardLCD;
	Dashboard *dashboard;
	AnalogChannel *ultrasonic;
	int test;
	DigitalInput *switch0;
public:
	MainRobot(void) {
		leftDrive0 = new Victor(DIGITAL_SIDECAR_PORT,LEFT_DRIVE_0_CHANNEL);
		leftDrive1 = new Victor(DIGITAL_SIDECAR_PORT,LEFT_DRIVE_1_CHANNEL);
		rightDrive0 = new Victor(DIGITAL_SIDECAR_PORT,RIGHT_DRIVE_0_CHANNEL);
		rightDrive1 = new Victor(DIGITAL_SIDECAR_PORT,RIGHT_DRIVE_1_CHANNEL);
		leftShifter = new Servo(DIGITAL_SIDECAR_PORT,LEFT_SHIFTER_CHANNEL);
		rightShifter = new Servo(DIGITAL_SIDECAR_PORT,LEFT_SHIFTER_CHANNEL);
		topLauncher = new Jaguar(DIGITAL_SIDECAR_PORT,TOP_LAUNCHER_CHANNEL);
		bottomLauncher = new Jaguar(DIGITAL_SIDECAR_PORT,BOTTOM_LAUNCHER_CHANNEL);
		conveyor0 = new Victor(DIGITAL_SIDECAR_PORT,CONVEYOR_0_CHANNEL);
		conveyor1 = new Victor(DIGITAL_SIDECAR_PORT,CONVEYOR_1_CHANNEL);
		launchEncoderTop = new Encoder(LAUNCH_ENCODER_TOP_CHANNEL_A, LAUNCH_ENCODER_TOP_CHANNEL_B, false, Encoder::k1X);
		//launchEncoderBottom = new Encoder(LAUNCH_ENCODER_BOTTOM_CHANNEL);
		stick0 = new Joystick(JOYSTICK_0_PORT);
		stick1 = new Joystick(JOYSTICK_1_PORT);
		stick2 = new Joystick(JOYSTICK_2_PORT);
		ultrasonic = new AnalogChannel(7);
		dashboardLCD = DriverStationLCD::GetInstance();
		//shooterControl = new PIDController(1.0f,0.000f,0.0f,launchEncoderTop,topLauncher);
		SetPeriod( 0.0 );
		launchEncoderTop->SetDistancePerPulse(1.0);
		test = 0;
		switch0 = new DigitalInput(1);
	}


	//New Methods

	//Update the drive motor speeds on the robot from stick0 and stick1 positions
	void Drive(void) {
		float left = stick0-GetY();
		float right = stick1-GetY();
		float turning = stick1-GetX();
		if (stick0-GetRawButton(TURNING_BUTTON)) { //if you hold this button, the robot will only rotate, with stick1x
			leftDrive0-Set(turning);
			leftDrive1-Set(turning);
			rightDrive0-Set(-turning);
			rightDrive1-Set(-turning);
		}
		else {
			leftDrive0-Set(left);
			leftDrive1-Set(left);
			rightDrive0-Set(right);
			rightDrive1-Set(right);
		}
	}

	//Sets the gear shifting servos to the values that they currently aren't
	void ShiftGear() {
		leftDrive0->Set(0.0);
		leftDrive1->Set(0.0);
		rightDrive0->Set(0.0);
		rightDrive1->Set(0.0);
		Wait(0.25); //wait for a quarter of a second to let the drive motors stop
		if (((SHIFT_SERVO_MIN + SHIFT_SERVO_ERROR) > leftShifter->Get() > (SHIFT_SERVO_MIN - SHIFT_SERVO_ERROR)) && ((SHIFT_SERVO_MIN + SHIFT_SERVO_ERROR) > rightShifter->Get() > (SHIFT_SERVO_MIN - SHIFT_SERVO_ERROR))) {
			// are the two servos between the error threshold of where they would be if they were in MIN state? if so, set to MAX state.
			leftShifter->Set(SHIFT_SERVO_MAX);
			rightShifter->Set(SHIFT_SERVO_MAX);
		}
		if (((SHIFT_SERVO_MAX + SHIFT_SERVO_ERROR) > leftShifter->Get() > (SHIFT_SERVO_MAX - SHIFT_SERVO_ERROR)) && ((SHIFT_SERVO_MAX + SHIFT_SERVO_ERROR) > rightShifter->Get() > (SHIFT_SERVO_MAX - SHIFT_SERVO_ERROR))) {
			// are the two servos between the error threshold of where they would be if they were in MAX state? if so, set to MIN state.
			leftShifter->Set(SHIFT_SERVO_MIN);
			rightShifter->Set(SHIFT_SERVO_MIN);
		}
	}

	//Update speed of the launcher from the throttle on stick2
	void SetLauncherSpeed() {
		double throttle = stick2->GetThrottle();
		if(stick2->GetRawButton(2)) {
			if(throttle != launchEncoderTop->GetRate()) {//TODO: Add in bottomLauncher
				topLauncher->Set(throttle);
			}
		}
		else {
			topLauncher->Set(0.0);
		}
	}

	void regulateMotorSpeed( Encoder* motorEncoder, SpeedController* motor, double speed ) {
		double encoderRate = motorEncoder->GetRate();
		if( encoderRate != speed*MAX_MOTOR_RPM ) {
			motor->Set(speed);
		}
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

	}

	void TeleopPeriodic(void) {
		//dashboard->Printf("EncoderRate: %d", launchEncoderTop->GetRate()); //TEST CODE REMOVE
		dashboardLCD->Clear();

		//dashboardLCD->Printf( DriverStationLCD::kUser_Line1, 1, "Encoder Rate: %g",launchEncoderTop->Get() );
		dashboardLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Encoder Count: %d",launchEncoderTop->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Switch: %d",switch0->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line3,1, "Ultrasonic: %d",ultrasonic->GetValue());
		dashboardLCD->UpdateLCD();
		//topLauncher->Set(0.25);
	}

	void TeleopContinuous(void) {
		//Drive();
		//shooterControl->SetSetpoint(stick2->GetY());
		//regulateMotorSpeed( launchEncoderTop, topLauncher, stick2->GetY() );
		//topLauncher->Set(stick2->GetY());
		if(stick1->GetRawButton(2)) topLauncher->Set(0.25);
		else topLauncher->Set(0);
		//topLauncher->Set(stick2->GetY());
		//BottomLauncher->Set(stick2->GetY());
	}
};

START_ROBOT_CLASS(MainRobot);
