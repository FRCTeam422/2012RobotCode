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

const int LEFT_SHIFTER_CHANNEL = 8; //edit these soon
//const int RIGHT_SHIFTER_CHANNEL = ; 

const int TOP_LAUNCHER_CHANNEL = 10;	// TODO: change this back to 7
const int BOTTOM_LAUNCHER_CHANNEL = 9;	// TODO: change this back to 8

const int CONVEYOR_CHANNEL = 6;

//Analog Channels
const int BALL0_SENSOR_PORT = 5; //these interfere with the encoder channels TODO: fix that
const int BALL2_SENSOR_PORT = 7;

const int GYRO_PORT = 4;
//

//Digital Channels
const int LAUNCH_ENCODER_TOP_CHANNEL_A = 7;
const int LAUNCH_ENCODER_TOP_CHANNEL_B = 5;
const int LAUNCH_ENCODER_BOTTOM_CHANNEL = 8;

//Joystick Button Constants
const int TURNING_BUTTON = 2;

//Other Constants
const double MAX_MOTOR_RPM = 2000; //TODO: Get actual value for the maximum RPM
const double PULSES_PER_REVOLUTION = 250.0; //pulses for revolution on the endoders
//const double LAUNCHER_SPEED_INCREMENT = 0.01; // how much 


const float BALL_SENSOR_THRESHOLD = 1.2;
const int TRIGGER_ITERATIONS = 0; //TODO: Put the number of times the ball acquisition code will be run before a ball passes the sensor here

const float SHIFT_SERVO_MIN = 0.1;
const float SHIFT_SERVO_MAX = 0.75;
const float SHIFT_SERVO_ERROR = 0.1; //deprecated

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
	Victor *conveyor;
	
	Jaguar *shiftTestingMotor;
	
	Encoder *launchEncoderTop;
	PIDController *shooterControl;
	//Encoder *launchEncoderBottom;
	Joystick *stick0;
	Joystick *stick1;
	Joystick *stick2;
	DriverStationLCD *dashboardLCD;
	Dashboard *dashboard;
	AnalogChannel *ultrasonic;
	AnalogChannel *ball0;
	AnalogChannel *ball2;
	Gyro *gyro;
	int test;
	DigitalInput *switch0;
	//global variables used to keep track of the number of balls
	int ballsNumber;
	int counter;
	bool hadBall;
public:
	MainRobot(void) {
		leftDrive0 = new Victor(DIGITAL_SIDECAR_PORT,LEFT_DRIVE_0_CHANNEL);
		leftDrive1 = new Victor(DIGITAL_SIDECAR_PORT,LEFT_DRIVE_1_CHANNEL);
		rightDrive0 = new Victor(DIGITAL_SIDECAR_PORT,RIGHT_DRIVE_0_CHANNEL);
		rightDrive1 = new Victor(DIGITAL_SIDECAR_PORT,RIGHT_DRIVE_1_CHANNEL);
		leftShifter = new Servo(DIGITAL_SIDECAR_PORT,LEFT_SHIFTER_CHANNEL);
		//rightShifter = new Servo(DIGITAL_SIDECAR_PORT,LEFT_SHIFTER_CHANNEL);
		
		shiftTestingMotor = new Jaguar(DIGITAL_SIDECAR_PORT,7);
		
		topLauncher = new Jaguar(DIGITAL_SIDECAR_PORT,TOP_LAUNCHER_CHANNEL);
		bottomLauncher = new Jaguar(DIGITAL_SIDECAR_PORT,BOTTOM_LAUNCHER_CHANNEL);
		conveyor = new Victor(DIGITAL_SIDECAR_PORT,CONVEYOR_CHANNEL);
		launchEncoderTop = new Encoder(LAUNCH_ENCODER_TOP_CHANNEL_A, LAUNCH_ENCODER_TOP_CHANNEL_B, false, Encoder::k1X);
		//launchEncoderBottom = new Encoder(LAUNCH_ENCODER_BOTTOM_CHANNEL);
		stick0 = new Joystick(JOYSTICK_0_PORT);
		stick1 = new Joystick(JOYSTICK_1_PORT);
		stick2 = new Joystick(JOYSTICK_2_PORT);
		ultrasonic = new AnalogChannel(7);
		dashboardLCD = DriverStationLCD::GetInstance();
		//shooterControl = new PIDController(1.0f,0.000f,0.0f,launchEncoderTop,topLauncher);
		SetPeriod( 0.0 );
		launchEncoderTop->SetDistancePerPulse(1.0/PULSES_PER_REVOLUTION);
		ball0 = new AnalogChannel(BALL0_SENSOR_PORT);
		ball2 = new AnalogChannel(BALL2_SENSOR_PORT);
		gyro = new Gyro(ANALOG_MODULE_PORT,GYRO_PORT);
		test = 0;
		switch0 = new DigitalInput(1);
		ballsNumber = 0;
		counter = 0;
		hadBall = false;
	}


	//New Methods

	//Update the drive motor speeds on the robot from stick0 and stick1 positions
	void Drive(void) {
		float left = stick0->GetY();
		float right = stick1->GetY();
		float turning = stick1->GetX();
		
		if (stick0->GetTrigger() || stick1->GetTrigger()) {
			ShiftGear();
		}
		
		if (stick0->GetRawButton(TURNING_BUTTON)) { //if you hold this button, the robot will only rotate, with stick1x
			leftDrive0->Set(turning);
			leftDrive1->Set(turning);
			rightDrive0->Set(-turning);
			rightDrive1->Set(-turning);
		}
		else {
			leftDrive0->Set(left);
			leftDrive1->Set(left);
			rightDrive0->Set(right);
			rightDrive1->Set(right);
		}
	}

	//Sets the gear shifting servos to the values that they currently aren't
	void ShiftGear() {
		//leftDrive0->Set(0.0);
		//leftDrive1->Set(0.0);
		//rightDrive0->Set(0.0);
		//rightDrive1->Set(0.0);
		Wait(0.1); //wait for a quarter of a second to let the drive motors stop
		if (((leftShifter->Get() >= SHIFT_SERVO_MIN - SHIFT_SERVO_ERROR) && (leftShifter->Get() <= SHIFT_SERVO_MIN + SHIFT_SERVO_ERROR))/* && (rightShifter->Get() == SHIFT_SERVO_MIN)*/) {
			leftShifter->Set(SHIFT_SERVO_MAX);
			//rightShifter->Set(SHIFT_SERVO_MAX);
		}
		else if (((leftShifter->Get() >= SHIFT_SERVO_MAX - SHIFT_SERVO_ERROR) && (leftShifter->Get() <= SHIFT_SERVO_MAX + SHIFT_SERVO_ERROR)))/* && (rightShifter->Get() == SHIFT_SERVO_MIN)*//* && (rightShifter->Get() == SHIFT_SERVO_MAX)*/ {
			leftShifter->Set(SHIFT_SERVO_MIN);
			//rightShifter->Set(SHIFT_SERVO_MIN);
		}
	}
	
	//Update speed of the launcher from the throttle on stick2
	void SetLauncherSpeed() {
		/*double throttle = stick2->GetThrottle();
		if(stick2->GetRawButton(2)) {
			// if the speed it's supposed to be running is less than  current rate from encoder, increment the speed by LAUNCHER_SPEED_INCREMENT
			if(throttle < (launchEncoderTop->GetRate() / MAX_MOTOR_RPM)) { // both numbers should range from 0 to 1 //TODO: Add in bottomLauncher
				topLauncher->Set(topLauncher);
			}
			else if(throttle > (launchEncoderTop->GetRate() / MAX_MOTOR_RPM)) { // both numbers should range from 0 to 1 //TODO: Add in bottomLauncher
		}
		else {
			topLauncher->Set(0.0);
		}
		*/
		topLauncher->Set(stick2->GetThrottle() / MAX_MOTOR_RPM);
		
	}
	//This counts balls, and runs the conveyor
	void BallAcquisition() {
		conveyor->Set(stick2->GetY());
		//increments the counter if a ball enters
		if ((ball2->GetVoltage() >= BALL_SENSOR_THRESHOLD) && (hadBall == false)) {
			++ballsNumber;
			hadBall = true;
		}
		else {
			hadBall = false;
		}
		//raises the ballNumber to 2 if both sensors are activated ant the number is under 2
		if(((ball2->GetVoltage() >= BALL_SENSOR_THRESHOLD) || (ball0->GetVoltage() >= BALL_SENSOR_THRESHOLD)) && (ballsNumber < 2)) {
			ballsNumber = 2;
		}
		//sets the ball count to values determined by data from both sensors
		else if ((ball2->GetVoltage() >= BALL_SENSOR_THRESHOLD) && (ball0->GetVoltage() >= BALL_SENSOR_THRESHOLD)) {
			++counter;
		}
		if (ballsNumber < 2) {
			ballsNumber = 2;
		}
		if (counter >= TRIGGER_ITERATIONS) {
			ballsNumber = 3;
			counter = 0;
		}
		else {
			counter = 0;
		}
		//This checks if it's too big.
		if (ballsNumber > 3) {
	    	ballsNumber = 3;
		//This checks if it's too small.
		}
		else if (ballsNumber < 0) {
			ballsNumber = 0;
		}
		//this sends the driver a message if there are three balls
		if (ballsNumber == 3) {
			dashboardLCD->Printf(DriverStationLCD::kUser_Line5,1, "WARNING: In posession three balls.");//SendWarning(); //TODO: replace with real function to send mesage to dashboard
		}
	}
	
	//ballsNumber--;//TODO: put into the firing function after a ball is launched
	
	/*
	void regulateMotorSpeed( Encoder* motorEncoder, SpeedController* motor, double speed ) {
		double encoderRate = motorEncoder->GetRate();
		if( encoderRate != speed*MAX_MOTOR_RPM ) {
			motor->Set(speed);
		}
	}
	*/
	
	void Launch() {
			
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
		leftShifter->Set(SHIFT_SERVO_MIN);
		//rightShifter->Set(SHIFT_SERVO_MAX);

		//shooterControl->Enable();

	}

	void TeleopPeriodic(void) {
		//dashboard->Printf("EncoderRate: %d", launchEncoderTop->GetRate()); //TEST CODE REMOVE
		dashboardLCD->Clear();

		//dashboardLCD->Printf( DriverStationLCD::kUser_Line1, 1, "Encoder Rate: %g",launchEncoderTop->Get() );
		dashboardLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Encoder Count: %d",launchEncoderTop->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Encoder Rate: %g",launchEncoderTop->GetRate());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Ultrasonic: %d",ultrasonic->GetValue());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Gearshift: %g",leftShifter->Get());
		dashboardLCD->Printf(DriverStationLCD::kUser_Line5, 1, "GyroAngle: %g",gyro->GetAngle());

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
		
		shiftTestingMotor->Set(stick2->GetY());
		
		if (stick0->GetTrigger() || stick1->GetTrigger()) {
			ShiftGear();
		}

		//topLauncher->Set(stick2->GetY());
		//BottomLauncher->Set(stick2->GetY());
	}
};

START_ROBOT_CLASS(MainRobot);
