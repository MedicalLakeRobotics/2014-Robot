// **************************************************************************************
//       SimpleTemplate - MyRobot2014.cpp	              	2/1/14		6:00 PM
//
//		version 2014-11 3-30-14	Ellensburg updates; Timeouts added to auton drive routine,
//								camera code commented out. Allow fire even if rod not fully
//								retracted. Updated autonomous drive distance and angle.
//								Added timeout routine to retraction mode.
//
//      version 2014-10 3-22-14 Corrected for new threaded rod, and new hold voltage, and
//                              adjusted camera settings for size and threshold, removed
//                              accelerometer
//
//		version 2014-9	3-13-14	Added I2C Output for LED Display
//
//		version 2014-8	3-8-14	Added encoder for drive train encoder for distance.
//								Simplified angle calc routine, Added I2C accelerometer
//								for a direct angle elevation angle measurement
//
//		version 2014-7	2-17-14	Simplified autonomous drive routines and log displays 
//
//		version 2014-6	2-16-14	Rewrite firing routine dont use parked flag simply look at 
//								retracted plunger and extended plunger to see when object been
//								fired to stop the motor. Also corrected bugs in soft fire.
//								changed timers in firing sequence.
//
//		version 2014-5	2-15-14	Update for autonomouse routine, spilt vision proccessing code
//								into get image and proccess image. Set up timers to time each
//								phase of the autonomous driving and shutdown at approp. time.
//								addded a logging mode for display on console travel speed and
//								distance. Added retract overide to slow fire command.  Added
//                              camera position at begining of autonomouse based on switches
//
//		version 2014-4	2-12-14	Update smart dash for driver/shooter data. Made camera code
//								a callable function. Provided timer for fire controll.
//
//		version 2014-3	2-11-14	Add automatic retact code, slow fire code, timer for plunger
//								speed, 4 test routings for autonomous driving, additional
//								limit switches and configuration  switches
//
//		version 2014-2	2-9-14	Added Ultrasound, Gyrsoscope Temp Sensor and Moved the
//                              instantiation of the camera up to global level.
//
//		version 2014-1	2-1-14	New code for final challenge - Ball Shooter with Vision
//
//
// **************************************************************************************
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include <iostream>
#include <sstream>
#include <string>
#include <Timer.h>

const string kProgramName = "Simple Template - MyRobot 2014 Version 9 (3/13/2014)";

// ------------------------ Various Constants --------------------------------
const float	 kLEDOnTime = 0.3;			// Ring LED On time 
const float	 kpanServoCenter = 0.5;		// Servo speed to center camera
const float	 kpanServoLeft = 0.35;		// Servo speed to left camera
const float	 kpanServoRight = .65;		// Servo speed to right camera
const float	 kholdPlungerSpeed = 0.12;	// Speed to hold plunger from being pulled back
const float	 kretractPlungerSpeed = 1;	// Speed to pull plunger in
const float  kextendPlungerSpeed = -0.6;	// Set speed for soft release
const float	 kfiringDelay = 0.25;		// delay for firing motor to clear the firing pin
const int	 kRaisedAngle = 72;			// When fork lift is fully raised 
const int	 kLoweredAngle = 0;			// Angle when fully lowered
const int 	 kAutoFireAngle = 48;		// Angle to lower fork too during autonomous mode
const int    kAutoFireEncoder = -2918;	// Lowere limit encoder
const float	 kLogInterval = 0.1;		// Interval time to capture motion data 100 ms
const float	 kWheelDiam = 4.0;			// Diameter of drive wheel for distance calculation

const uint8_t	kI2cReg = 0x21;			// I2C Dummy register address

// ------------------------ Autonomous Speed and Time Constants ------------------
const float	 kAutoElevTimeOut = 3.0;	// if elevation motor runs beyond these seconds stop motor
const float  kAutoMinDistance = 60;		// Distance to wall to stop  

const float  kAutoPhase1Speed = 0.6;	// Phase 1 drive speed
const float	 kAutoPhase1Time = 0.5;		// Phase 1 drive time
const float  kAutoPhase1Distance = 3.0; // Phase 1 Distance in inches
const float  kAutoPhase2Speed = 0.8;	// Phase 1 drive speed
const float	 kAutoPhase2Time = 1.5;		// Phase 1 drive time
//const float  kAutoPhase2Distance = 14.0; // Phase 1 Distance in inches
const float  kAutoPhase2Distance = 30.0; // Phase 1 Distance in inches

const float  kAutoPhase3Speed = 0.6;	// Phase 1 drive speed
const float	 kAutoPhase3Time = 0.5;		// Phase 1 drive time
const float  kAutoPhase3Distance = 5.0; // Phase 1 Distance in inches
//const float  kAutoForwardSpeed = .75;	// Speed forward during automation
//const float  kAutoForwardTime = 1.5;	// How long to go forwardnout of start box
//const float	 kAutoSwingTurnSpeed = 1;	// Speed of motor for turning
//const float  kAutoSwingTurnTime = .75;	// How long to go forwardnout of start box
//const float	 kAutoPointTurnSpeed = .75;	// Speed of motor for turning
//const float  kAutoPointTurnTime = 0.75;	// How long to go forwardnout of start box

const float	 kDriveDeadband = 0.28;		// scale constant for Drive deadband zone
const float  kTurnRate = 1.2;			// scale the turn rate by this amount

const UINT32 kLmtSwTrue = 0;			// Limit sw is currently pressed
const UINT32 kLmtSwFalse = 1;			// Limit sw is currently NOT pressed

// --------------------- Right Joystick - Driver --------------------------
const UINT32 kDriveBtn = 1;				// Button 1 (Trigger) R-Joy to drive
const UINT32 kautonConfirmBtn = 2;		// Button to confirm autonomous test
const UINT32 kpanCtrBtn = 3;			// Button to pan camera to center position
const UINT32 kpanLeftBtn = 4;			// Button to pan camera to left position
const UINT32 kpanRightBtn = 5;			// Button to pan camera to right position
const UINT32 kRingLedOnBtn = 8;			// Button to turn on LED Ring Light
const UINT32 kResetGyroBtn = 9;			// Button to reset gyro


const UINT32 kautonTest1Btn = 6;			// Button to test an autonomous routine
const UINT32 kautonTest2Btn = 7;			// Button to test an autonomous routine
const UINT32 kautonTest3Btn = 10;			// Button to test an autonomous routine
const UINT32 kautonTest4Btn = 11;			// Button to test an autonomous routine

// -------------------- Left Joystick - Shooter-----------------------------
const UINT32 kElevBtn = 1;				// Button 1 (Trigger) Raise/Lower Fork Arms
const UINT32 kretractBtn = 6;			// Button to retract plunger
const UINT32 kFireBtn = 3;				// Button to Fire plunger release
const UINT32 ksoftFireBtn = 4;			// Button to SLOW FIRE
const UINT32 kautoAimBtn = 5;			// Button to have elevation done automatically
const UINT32 kcalibrationBtn = 8;		// Button to Trim with throttle the auto adjustment value
const UINT32 kretractOverideBtn = 9;	// Button to Trim with throttle the auto adjustment value



// ------------ Camera constants used for distance calculation ----------------------
#define Y_IMAGE_RES 240					// y Image resolution in pixels, 640 x 480
#define X_IMAGE_RES 320					// X Image resolution in pixels, should be 160, 320 or 640
#define VIEW_ANGLE 48					// Axis M1013 camera arrived at experimentally
//#define VIEW_ANGLE 48					// Axis 206 camera
//#define VIEW_ANGLE 43.5  				// Axis M1011 camera
#define PI 3.141592653

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 40
#define ASPECT_RATIO_LIMIT 55

#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Score limits used for hot target determination
#define TAPE_WIDTH_LIMIT 50
#define VERTICAL_SCORE_LIMIT 50
#define LR_SCORE_LIMIT 50

//Minimum area of particles to be considered
#define AREA_MINIMUM 80

//Maximum number of particles to process
#define MAX_PARTICLES 8


// ****************************************************************************************
// ********************************** Class RobotDemo *************************************
// ****************************************************************************************
class RobotDemo : public SimpleRobot
{
	// -----------------------------------------------------------------------------------
	//                        Varoius Global Objects and Variables
	// -----------------------------------------------------------------------------------
	
	//Structure to represent the scores for the various tests used for target identification
	struct Scores {	
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
		double xEdge;
		double yEdge;
	};

	struct TargetReport {
		int	 	verticalIndex;
		int		horizontalIndex;
		bool	Hot;
		double	totalScore;
		double	leftScore;
		double	rightScore;
		double	tapeWidthScore;
		double	verticalScore;
	};

	struct TargetResult {
		bool hot;
		double distance;
	};
	
	int	  fireMode;		// 0 = nothing to do do, 1 = move motor out of parked position
						// 2 = we are out of parked position now continue on to parked position
	int   softFireMode;	// 0 = nothing to do, 2 = moving forward now
	int   retractMode;	// 0 = hold in place, 1 = command to retract, 2 = retracting now
	int	  elevFlag;		// 0 = elevator is done moving 0 = no 1 = yes
	int	  driveFlag;	// 0 = still moving 1= reached limit and stopped
	int   extendMode;	// 0 = do nothing, 
	int   autonFlag;	// 0 = First time through autonomous 1 = all others
	int   testFlag;		// 0 = First time through test routine 1 = all others
	int   retractOverRideFlag; // flag on fire to indicated if retract overide was also pressed
	int   holdRetractedFlag;	// 1 = Hold retracted state, 0= Shutoff plunger motor
	int	  imageRetFlag;	// 0 = No Image received to process 1 = good image to process
	int	  passFlag;		// 0 = first time into routine 1 or above second pass or more
	float   ultraLastDistance;	// last ultrasound distance for logging
	float   encoderLastDistance;	// last encoder distance for logging
	float  distance;			// running distance based on encoder
	double	lasttime;	// last timestamp for logging
	double lastenc;		// last encoder value for logging
	double lastDriveEnc; // last drive encoder value for logging
	double ultraInches, encoderInches;
	
	float panState;
	int   pictureCnt;					// Counter to limit to 1 snapshot dureing test
	float aThrottle, aSpeed, aTurn;
	float rThrottle, rSpeed, rTurn;
	float lThrottle, lSpeed, lTurn;	
	float hThrottle, hSpeed, hTurn;
	float voltage;
	float gyroAngle;		// Store gyroscope temperature value
	float gyroTemp;			// Store gyroscope returned angle
	float potValue;			// Store potentiometer value
	double xaxis, yaxis, zaxis;
	double timeLast, imageTime, imageTimeTotal;
	double fireTimer, lastFireTime, plungerTime;
	int currentAngle;
	int plungerTmrFlag;		// Flag to start and start timer when plunger is fired
	TargetResult target;	// create a target to receive camera results
	string functionName;	// Store last function name
	string rJoystickMsg;	// Store last right joystick press
	string lJoystickMsg;	// Store last left joystick press
	string cameraPosMsg;	// Store position of pan tile camera
	string msg1;			// Store a message
	string msg2;			// second message
	
	// ================= Device Class Pointers =========================
	Joystick rstick;					// right joystick
	Joystick lstick;	 				// left joystick
	RobotDrive myRobot;					// robot drive system
	Jaguar elevMotor;					// This Motor raises/lwr lift forks
	Jaguar plungerMotor;				// Motor to pull back plunger
	Victor releaseMotor;				// Fire release motor
	Servo panServo;						// Camera Pan Servo
	Relay ringLedSpike;					// Ring LED light for camera
	DigitalInput elevLmtSwHi;			// Test for fork lift raised to top 
	DigitalInput elevLmtSwLo;			// Test for fork lift lowered to bottom
	DigitalInput plungerRetractLmtSw;	// Test for plunger in retracted position
	DigitalInput plungerExtendLmtSw;	// Test for plunger in extended position
	DigitalInput parkedLmtSw;			// Test for Fire release motor in parked position
	DigitalInput configLeftSw;			// Input for left configuration switch
	DigitalInput configRightSw;			// Input for right configuration switch
	Ultrasonic ultra;					// Ultrasonic Range Finder (Tsmt on 10, Rcv 11)
	Encoder liftEncoder;				// Optical Encoder for angle
	Encoder driveEncoder;				// Optical Encoder for drive distance
	AnalogChannel temp;					// Analog input for gyroscope temperature sensor
	Gyro gyro;							// Analog input for Gyroscope angle	
	AnalogChannel pot;					// Analog input for potentiometer value
	
	DriverStationLCD *dsLCD;
//	AxisCamera *camera;
	I2C *m_i2c;
	DigitalModule *module;
	UINT8 arduinoAddress;
	UINT8 LEDCmd[4];
	UINT8 i2c_recbuf[3];
	
	Timer retractTimeOutTmr;			// elevator raise/lowere time out timer	
	Timer elevTimeOutTmr;				// elevator raise/lowere time out timer
	Timer driveTimeOutTmr;				// drive timer fore each phase of the autonomous drive
	Timer logTmr;						// log timer
	Timer driveTotalTmr;				// Drive total time timer to show progress through full
	Timer plungerSpeedTmr;				// Timer to determine plunger speed
	Timer firingMtrTmr;					// Timer to detect runaway fireing motor
	ColorImage *image;
	
public:
	RobotDemo(void):

		// ================= Device Instantiations =========================
		
		// -------------------- Joystick Ports -------------------------
		rstick(1),
		lstick(2),
		
		// ------------------ PWM Ports ---------------------------------
		myRobot(1, 2),			// PWM Output Port Assignments
		elevMotor(3),			// PWM Jaguar to raise/lwr lift forks
		plungerMotor(4),		// PWM - Motor to pull back plunger
		releaseMotor(5),		// Relay - Fire Release Motor
		panServo(6),			// Servo - Pan camera left/ctr/right

		// ------------------- Relay Ports ------------------------------
		ringLedSpike(1),		// RELAY - Ring LED Light

		// ------------------- Digitital IO Ports -----------------------
		elevLmtSwHi(1),			// DIGITAL INPUT - Test for fork lift raised to top 
		elevLmtSwLo(2),			// DIGITAL INPUT - Test for fork lift lowered to bottom
		plungerRetractLmtSw(8),	// DIGITAL INPUT - Test for plunger retracted
		plungerExtendLmtSw(5),	// DIGITAL INPUT - Test for plunger extended
		parkedLmtSw(6),			// DIGITAL INPUT - Test for release motor parked
		configLeftSw(7),		// DIGITAL INPUT - Test for left configuration switch position 
		configRightSw(3),		// DIGITAL INPUT - Test for right configuration switch position 
		ultra(9,10),			// Ultrasonic Range finder
		liftEncoder(11, 12),	// Optical Encoder
		driveEncoder(13,14, true),	// Drive Encoder set for reverse direction

		// -------------------- Analog Ports ----------------------------
		temp(1),				// Gyroscope temperature sensor
		gyro(2),				// Gyroscope
		pot (3)					// potentiometer not used at this time

	{
		myRobot.SetExpiration(1.0f);
		myRobot.SetSafetyEnabled(false);
		GetWatchdog().SetEnabled(false);
		SmartDashboard::init();
		dsLCD = DriverStationLCD::GetInstance();
//		camera = &AxisCamera::GetInstance();		// create camera object
		module = DigitalModule::GetInstance(1);
		arduinoAddress = 168;
		m_i2c = module->GetI2C(arduinoAddress);
		LEDCmd[0] = 0;
		
		elevTimeOutTmr.Reset();
		currentAngle = kRaisedAngle;				// Forks are raised at beginning			
		fireMode = 0;			// initialize fire mode to do nothing at this time
		retractMode = 0;		// initialize retract mode to do nothing at this time
		softFireMode = 0;		// initialize fire mode to do nothing aat this time
		panState = kpanServoCenter;
		cameraPosMsg="Center";
		timeLast = 0;
		lastFireTime = 0;
		distance = 0;			// reset running distance to zero
		encoderLastDistance = 0;
		ultraLastDistance = 0;
		plungerTmrFlag  = 0;
		target.hot = false;
		target.distance = 0;
		
		functionName = "Startup";
		msg1 = " ";
		msg2 = " ";
		lJoystickMsg = " ";
		rJoystickMsg = " ";
		printf("\nThis is initialization");
		updateSmartDash();
	}
	
	
	/*************************************************************
	* Initialize is run immediately when the robot is powered on
	* regardless of the field mode.
	*************************************************************/
/*	void Initialize(void)
	{
		panServo.Set(kpanServoRight); // these dont seem to execute her maybe motors are disabled?
		Wait(2.0);
		panServo.Set(kpanServoLeft);
		Wait(2.0);
		panServo.Set(kpanServoCenter);
		Wait(2.0);
		printf("This is the init routine !!!!!");
		
		int i;
	    for (i = 0; i < 10; i++)
	    {
	        printf("Initialize %d\r", i);
	        Wait(500);
	    }
	}
*/

	// ****************************************************************
	// *                     TELEOPERATOR MODE                        *
	// ****************************************************************
	void OperatorControl(void)
	{
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Teleoperator Mode Running");
		dsLCD->UpdateLCD();		
		gyro.Reset();
		liftEncoder.Start();
		liftEncoder.Reset();
		driveEncoder.Start();
		driveEncoder.Reset();
		ultra.SetAutomaticMode(true);
		
		holdRetractedFlag = 1;		// start with hold in retracted position
		retractOverRideFlag = 0;	// start with retract overide flag disabled
		retractMode = 0;			// start with retraction of plunger
		softFireMode = 0;			// start with no soft fire set
		fireMode = 0;				// start with no fire state
		distance = 0;			// reset running distance to zero
		encoderLastDistance = 0;
		ultraLastDistance = 0;

		
		printf("Entering TeleOperator Mode: %f \n",Timer::GetFPGATimestamp());
		
		while (IsOperatorControl() && IsEnabled())
		{
			updateDrive();				// drive robot
			updateAutonTest();			// test various autonomous drive routines
			updateRetractPlunger();		// Check if plunger retract button pressed
			updateElev();				// Check if change in elevation
			updateSoftFire();			// Check if soft fire button has been pressed
			updateFire();				// Check if fire button has been pressed
			checkResetGyro();			// Check if button pressed to reset gyro to 0
			updateCameraPan();			// update camera pan command
			checkRingLEDBtn();			// check if Ring LED button pressed	
			if (Timer::GetFPGATimestamp() - timeLast > .1)	//Update display 10 times a second
			{
				updateSmartDash();
				timeLast =Timer::GetFPGATimestamp();
			}
		} // ---- end of IsOperatorControl loop ----
		printf("Leaving TeleOperator Mode: %f \n",Timer::GetFPGATimestamp());
	} // --------end of Operator mode ---------

	// ****************************************************************
	// *                          TEST MODE                           *
	// ****************************************************************

	void Test() {
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Test Mode Running");
		dsLCD->UpdateLCD();
	}

	// ****************************************************************
	// *                   AUTONOMOUS MODE                            *
	// ****************************************************************
	void Autonomous(void)
	{
		int retval = 0;							// returned from getimage and used in process image
		elevFlag = 0;							// elevator flag 0 = still moving 1 = done moving
		autonFlag = 0;							// Used to make loop only done once
		driveFlag = 0;							// This is used to tell if were still driving
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Autonomous Mode Running");
		dsLCD->UpdateLCD();		
		myRobot.SetSafetyEnabled(false);
		printf("\nAutonomous Mode");
		distance = 0;			// reset running distance to zero
		encoderLastDistance = 0;
		ultraLastDistance = 0;
		retractMode = 0;							// reset retract mode flag
		softFireMode = 0;							// reset soft fire mode flag
		fireMode = 0;								// reset fore mode flag
		liftEncoder.Start();						// reset encoder to zero lift should be at top
		liftEncoder.Reset();						// reset encoder to zero lift should be at top
		driveEncoder.Start();						// lets measure distance traveled
		driveEncoder.Reset();						// reset the drive Encoder
		
		while (IsAutonomous() && IsEnabled())
		{
			if (autonFlag > 0)continue;
			autonFlag = 1;							// set flag for one pass only			

			// ************ Phase 0 of autonomouse drive ***********************
			// * Pan Camera, Turn on LED, Retract Plunger, Take Image          *
			// *****************************************************************
			printf ("\nStarting phase 0 Init & Get Image");
			postLog2( 0 );							// print out start line
			ringLedOnOff( 1 );						// Turn ON LED ring
			// Set camera to correct angle to take picture done here to save time in auton. mode
			if ((configLeftSw.Get() == kLmtSwFalse) && (configRightSw.Get() == kLmtSwTrue))
				panServo.Set(kpanServoRight);
			else if ((configLeftSw.Get() == kLmtSwTrue) && (configRightSw.Get() == kLmtSwFalse))
				panServo.Set(kpanServoLeft);
			else
				panServo.Set(kpanServoCenter);	
			logTmr.Start();							// used to capture sample every kLogInterval
			logTmr.Reset();							
			driveTotalTmr.Start();					// start timer to time the whole routine 
			driveTotalTmr.Reset();					 
			driveTimeOutTmr.Start();				// start timer for drive time of each phase
			driveTimeOutTmr.Reset();				 
			elevTimeOutTmr.Start();					// start timer to time out elevation if needed
			elevTimeOutTmr.Reset();
			driveEncoder.Reset();
			retractMode = 1;						// setup command to retract plunger
			holdRetractedFlag=1;					// setup parmeter to hold plunger retracted
			Wait(kLEDOnTime);						// give time for LED and plunger to settle	
			retval = getRobotImage(1);				// parameter 0=file, 1 =camera
			updateRetractPlunger();	// retract plunger
			
			// ************ Phase 1 of autonomouse drive ***********************
			// * start fork down, drive forward while proccessing the image    *
			// *****************************************************************
//			printf ("\nStarting phase 1");
			elevMotor.Set( -1.0 );					// Start elevator moving down !!!!!!!			
			elevTimeOutTmr.Reset();					// reset the elevation time out timer
			driveEncoder.Reset();					// reset the drive Encoder
			myRobot.TankDrive(kAutoPhase1Speed, kAutoPhase1Speed * 0.9);	// Start robot moving !
			driveTimeOutTmr.Reset();				// reset the drive time out timer
			postLog2( 1 );							// print out a test line
//			target = processImage(retval, 0);		// Proccess image , No test outputs   !!!!!!!
			autonDrive(kAutoPhase1Speed,kAutoPhase1Time, kAutoPhase1Distance, 1);
		
			// ************ Phase 2 of autonomouse drive ***********************
			// * continue fork down as needed and driveing  forward needed     *
			// *****************************************************************
//			printf ("\nStarting Phase 2");	
			driveTimeOutTmr.Reset();				// reset the drive time out timer
			driveEncoder.Reset();					// reset the drive Encoder
			autonDrive(kAutoPhase2Speed,kAutoPhase2Time, kAutoPhase2Distance, 2);
			

			// ************ Phase 3 of autonomouse drive ***********************
			// * Slow down to stop, continue lowereing Fork is needed          *
			// *****************************************************************
//			printf ("\nStarting Phase 3");
			driveTimeOutTmr.Reset();				// reset the drive time out timer
			driveEncoder.Reset();					// reset the drive Encoder
			autonDrive(kAutoPhase3Speed,kAutoPhase3Time, kAutoPhase3Distance, 3);
			
			// ************ Phase 4 of autonomouse drive ***********************
			// * stop drive motors, continue lowereing Fork if needed          *
			// *****************************************************************
			myRobot.TankDrive( 0.0, 0.0);			// Stop moving now !!!!!!!
			// Verify that the arm can has reach the target angle in provided time !!!!!!
			// Verify retraction is complete provided time !!!!!!
			while (( retractMode !=0) || (elevFlag == 0)) // I think this is wrong "elevFlag != 0" ???
															// could this cause double retract ???
				{
					updateRetractPlunger();	// retract plunger
					checkElev();			// continue lowereing elev as needed
				}
			elevMotor.Set( 0.0 );					// Stop elevator moving down just in case !!!!
			postLog2( 4 );							// print out a test line
			
			// ************ Phase 5 of autonomouse drive ***********************
			// * Fire Ball and lower forks to ground                           *
			// *****************************************************************
//			printf ("\nStarting Phase 5");
			if (target.hot == false)
				{
					printf("\nWaiting - target not hot now");
					while (driveTotalTmr.Get() <= 4.5)	Wait(0.1);	// wait till time =8 seconds	
				}
			postLog2( 5 );							// print out a test line
			printf("\nFire Now!");					// add code to fire
			fireMode = 2;							// Mode to fire
			while (fireMode != 0) updateFire();		// This will loop untill ball is fired				
			postLog2( 6 );							// print out a test line
			Wait (0.25);							// Give time for fire
			postLog2( 7 );							// print out a test line
			// lets lower the arms for teleoperator mode
			while (elevLmtSwLo.Get() == kLmtSwFalse){	// Are we fully down ?
				elevMotor.Set( -1.0 );				// Start motor moving back to open position	
			}
			postLog2( 8 );							// print out a test line
			elevMotor.Set(0.0);						// stop motor
			liftEncoder.Reset();					// reset encoder
	
		}// end of loop for while autonomous and enabled
	}// end of autonomous block
	
	
	// -------------------------------------------------------------------
	// ----------------- Autonomous Drive Routine -----------------------------------
	// -------------------------------------------------------------------
	
	void autonDrive(float speed, float time, float distance, int phaseNum)
	{
		float distanceTemp;
		printf ("\nStarting Phase %d", phaseNum);
		myRobot.TankDrive( speed, speed );		// Start robot moving !!!!!!!
		driveFlag = 0;							// reset drive flag
//		while (driveTimeOutTmr.Get() <= time)	// Continue driving till timeout
//			{
//				checkUltrasound();				// check if we are TOO close to the wall
//				checkElev();					// check to see if we need to stop elevator
//			}

		distanceTemp = (driveEncoder.Get() * (PI * kWheelDiam) / 250); // in inches
		encoderLastDistance = distanceTemp;
		while (distanceTemp < distance)			// Continue driving till reached in
			{
				if (driveTimeOutTmr.Get() > time)	// timeout exceeded
				{
					printf("\nTIMED OUT auton Drive");
					break;						// jump out of while
				}
				updateRetractPlunger();			// retract plunger
				checkUltrasound();				// check if we are TOO close to the wall
				checkElev();					// check to see if we need to stop elevator
				distanceTemp = (driveEncoder.Get() * (PI * kWheelDiam) / 250); // in inches
			}
		postLog2( phaseNum );							// print out a test line
	}
	
	
	// ------------------------- check lift angle of Elevation -------------------------------
	void checkElev()
	{
		if (elevFlag > 0) return;				// elevator has completed it runs get out
		if (elevTimeOutTmr.Get() >= kAutoElevTimeOut)
			{
			elevFlag=1;
			printf ("\nElevator has timed out!");
			elevMotor.Set( 0.0 );				// stop elevator
			return;
			}
		if (elevLmtSwLo.Get() == kLmtSwTrue)
			{	// we have have hit bottom
			elevFlag=1;
			printf ("\nElevator lower has hit bottom!");
			elevMotor.Set( 0.0 );				// stop elevator
			return;
			}
		// calculate current angle from top value to offset by encoder value
		
		if (liftEncoder.Get() > kAutoFireEncoder )		//	Have we come down to the the target angle?
			elevMotor.Set( -1.0);				// No so keep moving down 
		else
		{	// yes we have reached the target angle
			elevFlag = 1;
			printf ("\nElevator has reached correct angle!");
			elevMotor.Set( 0.0 );				// Stop motor
		}
	}

	// ------------------------- check Ultrasound distance to wall -------------------------------
	void checkUltrasound()
	{
		if (driveFlag != 0) return;			// we have stopped already
//		if ((ultra.GetRangeInches() <= kAutoMinDistance))
//			{	// we have exceeded the limit for distance from target
//				driveFlag = 1;
//				printf ("\nUltrasound distance %f inches TOO CLOSE - stopped motor!", ultra.GetRangeInches());
//				myRobot.TankDrive( 0.0, 0.0);		// Stop robot moving !!!!!!!			
//			}
	}

	
	
	// *******************************************************************************
	// *                         Operator Control Routines                           *				  
	// *******************************************************************************

	
	// ---------------------------------------------------------------------------
	// -------------------- DRIVE ROBOT By Joystick & Right Trigger --------------
	// ---------------------------------------------------------------------------

	void updateDrive()
	{
		if (rstick.GetRawButton(kDriveBtn)== true){
			functionName="Drive Robot";				
			rJoystickMsg="Trigger Pushed";
			// -------- scale speed ------
			rThrottle = ((rstick.GetThrottle() - 1)/-2);
			rSpeed = -1*(rstick.GetY() * rThrottle);
			rTurn = (rstick.GetX() * kTurnRate * rThrottle);
			if (rTurn >1) rTurn = 1;
			else if (rTurn < -1) rTurn=-1;
			// calculate a deadband
			if (rSpeed > (- kDriveDeadband * rThrottle) and rSpeed < (kDriveDeadband * rThrottle))
				rSpeed = 0;
			if (rTurn > (-kDriveDeadband * rThrottle) and rTurn < (kDriveDeadband * rThrottle))
				rTurn = 0;
			Wait(0.005); // wait for a motor update time
		}
		else {
			rSpeed = 0.0;
			rTurn = 0.0;
		}
		myRobot.ArcadeDrive(rSpeed, rTurn); // send drive speed to drive class
	} //--------end updateDrive routine -------------
	
	
	// ---------------------------------------------------------------------------
	// -------------------- TEST DRIVE various routines out ----------------------
	// ---------------------------------------------------------------------------

	void updateAutonTest()
	{
		// ---------------------------- Autonomous Test Vision --------------------------------------------
		if ((rstick.GetRawButton(kautonTest1Btn)== true) && (testFlag == 0))
		{
			functionName="autonTest 1";				
			rJoystickMsg="Auton Test 1 Pushed";
			msg1 = "Get and Process an Image Test";
			testFlag = 1;
			updateSmartDash();
			ringLedOnOff( 1 );						// Turn ON LED ring
//			Wait(kLEDOnTime);						// give time for LED and plunger to settle	
//			target = processImage(getRobotImage(1),1);	// getRobotImage parameter 0=file, 1=camera
			updateSmartDash();
			ringLedOnOff( 0 );						// Turn OFF LED ring
		}
		// ------------------------------ Autonomous Test  Driving ------------------------------------------
		else if ((rstick.GetRawButton(kautonTest2Btn)== true) && (testFlag == 0))
		{
			functionName="autonTest 2";				
			rJoystickMsg="Auton Test 2 Log Data";
			msg1 ="Auto Test 2 - distance speed test";
			testFlag = 1;
			updateSmartDash();	
			barLEDCmd(3);	// issue I2C Command
		}
		// --------------------------------------------------------------------------------------
		else if ((rstick.GetRawButton(kautonTest3Btn)== true) && (testFlag == 0))
		{
			functionName="autonTest 3";				
			rJoystickMsg="Auton Test 3 Pushed";
			// go forward to starting section lined up on first target
			msg1 ="Auto Test 3 - Point turn for 0.75 seconds";
			testFlag = 1;
			updateSmartDash();
			barLEDCmd(5);	// issue I2C Command
		}
		// --------------------------------------------------------------------------------------
		else if ((rstick.GetRawButton(kautonTest4Btn)== true) && (testFlag == 0))
		{
			functionName="autonTest 4";				
			rJoystickMsg="Auton Test 4 Pushed";
			msg1 ="Auto Test 4 - Look at switches and combine movement";
			testFlag = 1;
			updateSmartDash();
			barLEDCmd(11);	// issue I2C Command
		}
		// be sure to clear one time flag after all buttons have been released 
		else if ((rstick.GetRawButton(kautonTest1Btn)== false) &&
				(rstick.GetRawButton(kautonTest2Btn)== false) &&
				(rstick.GetRawButton(kautonTest3Btn)== false) &&
				(rstick.GetRawButton(kautonTest4Btn)== false))
		{
			testFlag = 0;	// reset so it can be fired again
		}
	}

	void postLog2(int linenum)
	{
		float ultraDistance, driveDistance, ultraSpeed, driveSpeed;
		float time;
		int gyrotemp;
		float elevEncoder, driveEncoderTmp;
		float accelAngle, elevAngle;

		elevEncoder = liftEncoder.Get();
		elevAngle = encoderCntConvert (elevEncoder);
		accelAngle = 0;
		time = float(driveTotalTmr.Get());
		ultraDistance =  float(ultra.GetRangeInches());
		if (ultraLastDistance == 0)
			ultraSpeed = 0;							// no previous data to compare
		else
			ultraSpeed = (ultraLastDistance - ultraDistance) / kLogInterval;
		ultraLastDistance = ultraDistance;
		driveEncoderTmp = driveEncoder.Get();
		driveDistance = (driveEncoderTmp * (PI * kWheelDiam) / 250); // in inches
		if (encoderLastDistance == 0)
			driveSpeed = 0;							// no previous data to compare
		else
			driveSpeed = (encoderLastDistance - driveDistance) / kLogInterval;
		encoderLastDistance = driveDistance;
		gyrotemp = int(floor(float(gyro.GetAngle())));
		printf ("\nLog%d, tm=%f, USD=%f, USS=%f, DEnc=%f, DDist=%f, DSp=%f, Gyro=%d, Enc=%f, EncA=%f",
				linenum, time, ultraDistance, ultraSpeed, driveEncoderTmp, driveDistance, driveSpeed,
				gyrotemp, elevEncoder, elevAngle);
	} 
		
	
	// -------------------------- Retract Plunger Code ------------------------------
	//                  Pull back if command and stop and hold at end of stroke
	// ---------------------------------------------------------------------------
	void updateRetractPlunger()
	{
		if ((softFireMode != 0) || (fireMode != 0))return;	// exit now, others are in proccess

		if ((lstick.GetRawButton(kretractBtn) == true))
		{	
			functionName="Retract Plunger";				
			lJoystickMsg="Retract Plunger Pushed";
			holdRetractedFlag = 1;							// Hold when rertracted
			retractMode = 1;								// command retraction now
		}
		
		if(retractMode == 1){								// if first time through start timer
			retractTimeOutTmr.Start();
			retractTimeOutTmr.Reset();
			retractMode = 2;
		}
		
		if (retractTimeOutTmr.Get() > 10){							// check for time out and shut down mtr
			if (holdRetractedFlag == 0)
				plungerMotor.Set( 0.0 );
			else
				plungerMotor.Set(kholdPlungerSpeed);
			retractMode = 0;
			return;
		}
		
		if(retractMode == 2)								// Commanded to retract
			if (plungerRetractLmtSw.Get() == kLmtSwFalse)
				plungerMotor.Set(kretractPlungerSpeed);		// Plunger not at back keep going
			else
				retractMode = 0;							// plunger is now fully retracted
		
		if (retractMode == 0)								// Mode 0 do nothing continue to
			if (holdRetractedFlag == 0)
				plungerMotor.Set(0.0);
			else
				plungerMotor.Set(kholdPlungerSpeed);		// Hold plunger in place
	}

	// -------------------------- Extend plunger to soft fire ball ---------------------------
	//                   drive plunger forward till hit extend switch or keu is let up
	//                   then schedule a retraction
	// ---------------------------------------------------------------------------
	void updateSoftFire()
	{	
		if ((retractMode != 0) || (fireMode != 0))return;	// exit now, others are in proccess

		if ((lstick.GetRawButton(ksoftFireBtn) == false) && (softFireMode == 0))
			return; 								// nothing happening just get out

		if ((lstick.GetRawButton(ksoftFireBtn) == true))
		{	// were moving let make sure we dont go too far
			softFireMode = 1;
			if (plungerExtendLmtSw.Get() == kLmtSwFalse)
			{	// Plunger not fully forward keep going	
				plungerMotor.Set(kextendPlungerSpeed);
				return;
			}
			else
			{	// we HAVE hit the limit switch lets stop and look at retracting				
				plungerMotor.Set(0.0);					// Plunger is at front stop motor
				softFireMode = 0;						// our work is done here
				// lets also schedule a retraction now if overide not pushed
				if (lstick.GetRawButton(kretractOverideBtn) == false)
					{
						retractMode = 1;				// NOT overiden so retract now
						holdRetractedFlag = 1;			// hold when retracted
					}
				else
				{
					holdRetractedFlag = 0;				// No retraction and no retract hold
				}
			}
		}
		
		if ((lstick.GetRawButton(ksoftFireBtn) == false) && (softFireMode == 1))
		{	// Button not pushed any more so stop pushing and look at retraction				
			plungerMotor.Set(0.0);						// Button has been released while in motion
			softFireMode = 0;							// are work is done here
			// lets also schedule a retraction now
			if (lstick.GetRawButton(kretractOverideBtn) == false)
			{
				retractMode = 1;						// NOT overiden so retract now
				holdRetractedFlag = 1;					// hold retracted
			}
			else
			{
				holdRetractedFlag = 0;					// No retraction an no retract hold	
			}	
		}
	}


	// ------------------------ Fire Control Code --------------------------------
	//                    Check for fire button pressed
	// ---------------------------------------------------------------------------

	void updateFire() 
	{
		if ((retractMode != 0) || (softFireMode != 0))return;	// exit now, others are in proccess
		
		if ((fireMode == 0) && (lstick.GetRawButton(kFireBtn) == false)) return;	// nothing new
		
		// ******************** disable this code to enable fireing *****************
//		fireMode = 0;
//		return;
		// **************************************************************************
		if (((fireMode == 0 && (lstick.GetRawButton(kFireBtn) == true))) || (fireMode ==2))
			{
//				if (plungerRetractLmtSw.Get() == kLmtSwFalse)
//					return; // plunger not retracted so ignore request to fire nothing to shoot
				fireMode = 1;								// start the fireing motor
				plungerTmrFlag = 0;							// Reset flag
				firingMtrTmr.Start();						// Start run away timer
				firingMtrTmr.Reset();						// reset to zero
				releaseMotor.Set(1.0);						// start motor moving to fire
				if (lstick.GetRawButton(kretractOverideBtn) == false)
					retractOverRideFlag = 0;				// Overide NOT reequested
				else
					retractOverRideFlag = 1;				// Override HAS been requested
			}
		
		// fireMode must be a > 0 were in motion with the motor now

		if ((plungerRetractLmtSw.Get() == kLmtSwFalse) && (plungerTmrFlag == 0))
		{	// the plunger has just now been released and is flying forward
			plungerSpeedTmr.Start();
			plungerSpeedTmr.Reset();					// Start timing for firing speed
			plungerTmrFlag = 1;
			return;
		}
		
		if ((plungerExtendLmtSw.Get() == kLmtSwTrue) && (plungerTmrFlag == 1))
		{	// we have fired the plunger and it has just now struck the end
			lastFireTime = plungerSpeedTmr.Get();
			printf("\nPlunger Speed %f\n", lastFireTime);
			plungerTmrFlag = 2;
			Wait(kfiringDelay);		// wait for motor to go fully past the firing pin
			releaseMotor.Set(0.0);						// stop the fireing motor now
			fireMode = 0;								// Reset fire mode to await next command
			if (retractOverRideFlag == 1)
			{	// Dont retract
				holdRetractedFlag = 0;					// dont retract and dont hold plunger back 
				return;
			}
			else
			{	// Go ahead and request retraction
				holdRetractedFlag = 1;					// hold retracted when plulled back 
				retractMode = 1;						// lets schedule a retraction now
				return;									// time to get out of here
			}
		}

		// Test for run away condition
		if (firingMtrTmr.Get() > 3)
			{	// Fire motor is in run away condition stop everything
				releaseMotor.Set(0);						// stop motor
				fireMode = 0;								// Reset fire mode to await next command
			}
	}

	
	// ------------- Pan Camera Control Code --------------------------
	//          move camera to left center or right
	// ----------------------------------------------------------------
	
	void updateCameraPan() 
	{
		if (rstick.GetRawButton(kpanCtrBtn)== true)
			{
			rJoystickMsg="Pan to Center";
			panState = kpanServoCenter;
			cameraPosMsg="Center";
			}
		else if(rstick.GetRawButton(kpanLeftBtn)== true)
			{
			rJoystickMsg="Pan to Left";
			panState = kpanServoLeft;
			cameraPosMsg="Left";
			}
		else if(rstick.GetRawButton(kpanRightBtn)== true)
			{
			rJoystickMsg="Pan to Right";
			panState = kpanServoRight;
			cameraPosMsg="Right";
			}
		panServo.Set(panState);
	}

	// ----------------------------------------------------------------------------------
	// -----------------------------  Reset Gyroscope -----------------------------------
	// ----------------------------------------------------------------------------------
	void checkResetGyro()
	{	// --------------------- Check joystick for Reset Gyro --------------------------
		if (rstick.GetRawButton(kResetGyroBtn)== true)
		{	functionName="Reset Gyroscope";				
			gyro.Reset();								// Turn On LED ring
		}
	}
	
	
	// ----------------------------------------------------------------------------------
	// -----------------------------  Turn On/Off Ring LEDS -----------------------------
	// ----------------------------------------------------------------------------------
	void checkRingLEDBtn()
	{	// --------------------- Check joystick for Ring LED ----------------------------
		if (rstick.GetRawButton(kRingLedOnBtn)== true)
		{	functionName="Update Ring LED - ON";				
			ringLedOnOff( 1 );								// Turn On LED ring
		}
		if (rstick.GetRawButton(kRingLedOnBtn)== false)
		{	functionName="";
			ringLedOnOff( 0 );								// Turn Off LED ring
		}
	}
	
	// ------------------------ Drive LED Ring on/off -----------------------------	
	void ringLedOnOff( int ledFlag )
	{
		if (ledFlag == 0){
			ringLedSpike.Set(Relay::kOff);
		}
		else	{
			ringLedSpike.Set(Relay::kForward);
		}
	}

	// ------------------------ Drive BAR LED Light Strip ---------------------------	
	void barLEDCmd( UINT8 command )
	{
		LEDCmd[0] = command;
		m_i2c->Transaction(LEDCmd, 1, i2c_recbuf, 0);
	}
		
	
	
	// -----------------------------------------------------------------------------------------
	void updateElev()	// --------- Fork Lift Elevation by Joystick Routine ----------------
	{		
		if (lstick.GetRawButton(kElevBtn)== true)
		{	functionName="Raise Lower Fork Lift";				
			lJoystickMsg="Top Button Pushed";				

			// ------------------------- scale speed -------------------------------------
			lSpeed = lstick.GetY();		

			// ----- check limit switches, speed direction and update motor speed --------
			if (lSpeed == 0.0){								// speed = 0 so stop the motor
				elevMotor.Set(0.0);
			}
			else if (lSpeed < 0){							// speed < 0 therefore move arm FORWARD
				
				if (elevLmtSwLo.Get() == kLmtSwFalse){		// Are we fully forward ?
					elevMotor.Set(lSpeed);				// Start motor moving back to open position	
				}
				else{ // Fork Lift has hit the LOW limit switch so STOP the motor
					Wait(0.1);								// give time to fully seat at limit
					elevMotor.Set(0.0);						// stop motor
					liftEncoder.Reset();					// reset encoder
					msg2 ="Elev stop 2";
					// ******************* add code here to reset the encoder *********************
				}
			}
			else											// speed >0 so move fork up
			{
				if (elevLmtSwHi.Get() == kLmtSwFalse){		// Are we fully back?
					elevMotor.Set(lSpeed);					// No, so we can move backwards		
				}
				else { 										// Fork Lift has hit the High L.S. so STOP
					Wait(0.1);								// give time to fully seat at limit
					elevMotor.Set(0.0);						// stop motor
					// ******************* add code here to reset the encoder *********************
				}
			}
		}
		else
		{// The joystick button is not pushed so stop motors
			elevMotor.Set(0.0);
		}
	} // -------- end of  routine -------------

			
	// ***********************************************************************************
	//                              Display Update Routines                              *
	// ***********************************************************************************

	void updateSmartDash(void)
	{
		SmartDashboard::PutString("Program",kProgramName);		
		// ----------- update function and joystick Display -------------			
		SmartDashboard::PutString("Function",functionName);
		SmartDashboard::PutString("R-Joystick",rJoystickMsg);				
		SmartDashboard::PutString("L-Joystick",lJoystickMsg);

		double gyrotemp, elevangle;
		double tempPlungerSpeed;
		float driveEncoderTmp, driveDistance;
		
		driveEncoderTmp = driveEncoder.Get();
		driveDistance = (driveEncoderTmp * (PI * kWheelDiam) / 250); // in inches

		int recAngle;
		if (lastFireTime != 0)
			tempPlungerSpeed = ((8.75 / 12) / lastFireTime);
		else
			tempPlungerSpeed = 0;
		gyrotemp = gyro.GetAngle();

		elevangle = encoderCntConvert (liftEncoder.Get());
		recAngle = calcAngleRec(ultra.GetRangeInches());
		
		msg2 = " Gyro=" + float_to_string(gyrotemp);
		msg2 += " L-Enc=" + float_to_string(float(liftEncoder.Get()));
		msg2 += " D-Enc=" + float_to_string(float(driveEncoderTmp));
		msg2 += " RetM=" + float_to_string(float(retractMode));
		msg2 += " FireMe=" + float_to_string(float(fireMode));
		msg2 += " SftM=" + float_to_string(float(softFireMode));
		msg2 += " Spd=" + float_to_string(float(lastFireTime));
		
		SmartDashboard::PutString("Msg1",msg1);
		SmartDashboard::PutString("Msg2",msg2);
		
		// -------------- Driver Messages ------------
		SmartDashboard::PutString("Camera Orientation: ", cameraPosMsg);
		SmartDashboard::PutString("Ultrasound Distance"," "+float_to_string(float(int(float(((ultra.GetRangeInches()-17)/12)*100))/100)) + " Feet");

		SmartDashboard::PutString("Gyroscope Heading:  "," "+ float_to_string(float(int(gyro.GetAngle()))));
		SmartDashboard::PutString("Current Elevation Angle:    ",float_to_string(elevangle));
		SmartDashboard::PutString("Recommended Elevation Angle:"," "+ float_to_string(float(recAngle)));
		SmartDashboard::PutString("Drive Distance:    ",float_to_string(driveDistance) + " Feet");
		
		
		// -------------- Shooter Messages --------------
		SmartDashboard::PutString("Current Elevation Angle    ",float_to_string(elevangle));
		SmartDashboard::PutString("Recommended Elevation Angle"," "+ float_to_string(float(recAngle)));
		SmartDashboard::PutString("Ultrasound Distance"," "+float_to_string(float(int(float(((ultra.GetRangeInches()-17)/12)*100))/100)) + " Feet");
		SmartDashboard::PutString("Camera Distance    "," "+ float_to_string(float(target.distance)));
		SmartDashboard::PutString("Gyroscope Heading"," "+ float_to_string(float(int(gyro.GetAngle()))));
		SmartDashboard::PutString("Last Plunger Speed",float_to_string(float(tempPlungerSpeed)) + " fps");
		if (target.hot == true)
			SmartDashboard::PutString("Hot Zone ","YES");
		else
			SmartDashboard::PutString("Hot Zone ","NO");

		// ----------------------- update Limit Switch Display -------------------------
		if (plungerRetractLmtSw.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Retract LS ","Open");
		else
			SmartDashboard::PutString("Retract LS ","Closed");
		if (plungerExtendLmtSw.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Extend LS ","Open");
		else
			SmartDashboard::PutString("Extend LS ","Closed");
		if (configLeftSw.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Config Left ","Open");
		else
			SmartDashboard::PutString("Config Left ","Closed");
		if (configRightSw.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Config Right ","Open");
		else
			SmartDashboard::PutString("Config Right ","Closed");
		if (elevLmtSwHi.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Elev LS Hi ","Open");
		else
			SmartDashboard::PutString("Elev LS Hi ","Closed");
		if (elevLmtSwLo.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Elev LS Lo ","Open");
		else
			SmartDashboard::PutString("Elev LS Lo ","Closed");
		if (parkedLmtSw.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Parked LS ","Open");
		else
			SmartDashboard::PutString("Parked LS ","Closed");
	}	// -------------- End Update SmartDisplay routine -------------------
	
	
	// ***********************************************************************************
	//                        Convert a float number to a string                         *
	// ***********************************************************************************

	string float_to_string (float f)
	{
	    int prec;
		ostringstream s;
		if (f >= 100000 ) prec = 16;
		   else if (f >= 10000) prec = 11;
		        else if (f >= 1000 ) prec = 10;
		             else if (f >= 100) prec = 9;
		                  else if ( f >= 10) prec = 8;
		                       else prec = 7;
		                       
		s.precision(prec);
		s << f;
		return s.str();
	} // end float_to_string

	
	// ***********************************************************************************
	//                        Calculate Recomended angle                                 *
	// ***********************************************************************************

	int calcAngleRec (float distance)
	{
		int retVal, distfoot;
		distfoot = int(((distance - 17) / 12)+0.5);
	    if (distfoot < 5)
	    	retVal = 0;
	    else if((distfoot >=5) && (distfoot <6))
	    	retVal = 51;
	    else if((distfoot >=6) && (distfoot <7))
	    	retVal = 48;
	    else if((distfoot >=7) && (distfoot <9))
	    	retVal = 46;
	    else if((distfoot >=9) && (distfoot <10))
	    	retVal = 47;
	    else if((distfoot >=10) && (distfoot <11))
	    	retVal = 45;
	    else if((distfoot >=11) && (distfoot <13))
	    	retVal = 44;
	    else if((distfoot >=13))
	    	retVal = 0;
	    return retVal;
	}
	    

    
	// ***********************************************************************************
	//                        Convert a encoder count to angle                           *
	// ***********************************************************************************

	int encoderCntConvert2 (double encCnt)
	{
		float valTemp;
		if (encCnt < 0) encCnt = 0;
		if (encCnt > 13878) encCnt = 32878;
		
		for (int i=0; i < 15; i++)
		{
//			if ((encCnt >= encTbl[i]) && (encCnt < encTbl[i+1]))
//			{
//				valTemp = encoderCntCalc( encCnt, encTbl[i], encTbl[i+1], angleTbl[i], angleTbl[i+1]);
				return int(valTemp + 0.5);
//			}
		}
	}
	
	// ***********************************************************************************
	//                        Convert a encoder count to angle                           *
	// ***********************************************************************************

	int encoderCntConvert (double encCnt)
	{
		float valTemp;
	    if (encCnt < 0)
	    	valTemp = 0;

		//  ------ 0 - 5 degrees 0 - 1140 Enc Cnt ------------
	    else if (encCnt < 1140)
	       	valTemp = encoderCntCalc( encCnt, 0, 1140, 0, 5.05);

	    //  ------ 5 - 10 degrees 1140 - 2049 -------------------
	    else if (encCnt < 2049)
	       	valTemp = encoderCntCalc( encCnt, 1140, 2049, 5.05, 10.1);

	    //  ------ 10 - 15 degrees 2049 - 2860 -------------------
	    else if (encCnt < 2860)
	       	valTemp = encoderCntCalc( encCnt, 2367, 2860, 10.1, 15.0);

	    //  ------ 15 - 20 degrees 2860 - 3707 -------------------
	    else if (encCnt < 3707)
	       	valTemp = encoderCntCalc( encCnt, 2860, 3707, 15.0, 20.0);
	    
		//  ------ 20 - 25 degrees   3707 - 4525 Enc Cnt ------------
	    else if (encCnt < 4525)
	       	valTemp = encoderCntCalc( encCnt, 3707, 4525, 20.0, 25.1);

	    //  ------ 25 - 30 degrees 4525 - 5330 -------------------
	    else if (encCnt < 5330)
	       	valTemp = encoderCntCalc( encCnt, 4525, 5330, 25.1, 30.15);

	    //  ----- 30 - 35 degrees 5330 - 6069 -------------------
	    else if (encCnt < 6069)
	       	valTemp = encoderCntCalc( encCnt, 5330, 6069, 30.15, 34.9);

	    //  ------ 35 - 40 degrees 6069 - 6871 -------------------
	    else if (encCnt < 6871)
	       	valTemp = encoderCntCalc( encCnt, 6069, 6871, 34.9, 40.3);

	    //  ------ 40 - 45 degrees 6871 - 7579 -------------------
	    else if (encCnt < 7579)
	       	valTemp = encoderCntCalc( encCnt, 6871, 7579, 40.3, 45.1);
	    
	    //  ------ 45 - 50 degrees 7579 - 8283 -------------------
	    else if (encCnt < 8283)
	       	valTemp = encoderCntCalc( encCnt, 7579, 8283, 45.1, 50.05);
	    
	    //  ------ 50 - 55 degrees 8283 - 8941 -------------------
	    else if (encCnt < 8941)
	       	valTemp = encoderCntCalc( encCnt, 8283, 8941, 50.05, 55.0);
	    
	    //  ------ 55 - 60 degrees 8941 - 9640 -------------------
	    else if (encCnt < 9640)
	       	valTemp = encoderCntCalc( encCnt, 8941, 9640, 55.0, 60.3);
	    
	    //  ------ 60 - 65 degrees 9640 - 10197 -------------------
	    else if (encCnt < 10197)
	       	valTemp = encoderCntCalc( encCnt, 9640, 10197, 60.3, 64.99);
	    
	    //  ------ 65 - 74 degrees 10197 - 11247 -------------------
	    else if (encCnt < 11247)
	       	valTemp = encoderCntCalc( encCnt, 10197, 11247, 64.99, 73.95);
	    
	    else if (encCnt >= 11247)
	    	valTemp = 74;
		return (int(valTemp +0.5));
	}
	
	
	float encoderCntCalc (float encValue, float encLo, float encHi, float angleLo, float angleHi)
	{
		int returnVal;
		returnVal = int(((angleHi - angleLo) / (encHi - encLo)) * (encValue - encLo) + angleLo);
		//printf ("\nEncoder calc enc=%f encLo=%f encHi=%f angleLo%f angHi%f return=%d",encValue, encLo, encHi, angleLo, angleHi, returnVal );
		return returnVal;
	}
	
	
	// ****************************************************************************************	
	// ************************************* Vision Processing Routines ***********************
	// ****************************************************************************************

	int getRobotImage(int imageTest)
	{	// passed parameter 0=get image from file, 1=get image from camera
//		ringLedOnOff( 1 );								// Turn ON LED ring
//		Wait(kLEDOnTime);								// give time for LED	

		
//		if (imageTest == 0)
			image = new RGBImage("/frcimage.jpg");		// use this to load a file to process					
//		else
//			image = camera->GetImage();					// use this to Get a new image from camera
		ringLedOnOff( 0 );								// Turn OFF LED ring
		image->Write("/imagebackup.jpg");		// save image if needed for debugging
	    if (image->GetHeight() == 0 || image->GetWidth()==0) // look for a bad image
	    	return 0;
	    else
	    	return 1;
	}
	
	
	TargetResult processImage(int imageErrorFlag, int imageTestFlag)
	{	// imageErrorFlag 0=indicates good image, 1=bad image jump back out 
		// imageTestFlage 0=no print msgs, 1=give print msgs
		TargetResult temptarget;
	    temptarget.hot = false;
	    temptarget.distance = 0;
	    
		if (imageErrorFlag != 1) return temptarget;		// NO valid image return now

		Scores *scores;							// setup up variable for scores
		TargetReport target;					// setup up a variable for target info
		int verticalTargets[MAX_PARTICLES], horizontalTargets[MAX_PARTICLES];
		int verticalTargetCount, horizontalTargetCount;
		Threshold threshold(9, 248, 9, 179, 190, 236);		// HSV Created to match our camera

//		Threshold threshold(90, 200, 200, 255, 200, 255);		// HSV Created to match our camera
//		Threshold threshold(90, 200, 150, 255, 100, 255);		// HSV Created to match our camera
		//	Threshold threshold(105, 137, 230, 255, 133, 183);	// HSV Threshold for 2014 sample
		//	Threshold threshold(60, 100, 90, 255, 20, 255);     // HSV Threshold for 2013 sample
		
		//Particle filter criteria, used to filter out small particles		
		ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}};
		
		imageTime = Timer::GetFPGATimestamp();
		imageTimeTotal = imageTime;
		
		printf("\nBeginning Vision Proccessing: %f ",imageTime);
		
	//	while (!camera->IsFreshImage()) Wait(0.1);		// wait for camera to become ready

		if (imageTestFlag != 0) printf("\nStep 1 - %f ",Timer::GetFPGATimestamp()- imageTime);	
		imageTime = Timer::GetFPGATimestamp();
		
			// this is indented because it was originally in the autonomous enable loop!
			if (imageTestFlag != 0) printf ("\nStep 2 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			// -----------   get image from camera  --------------------
//			image = camera->GetImage();				// use this to Get a new image from camera
//			image = new RGBImage("/crioimage.jpg");	// use this to load a file to process		
			if (imageTestFlag != 0) printf("\nStep 3 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			if (imageTestFlag != 0) image->Write("/imagebackup.jpg");		// save image if needed for debugging
			ringLedOnOff( 0 );						// Turn Off LED ring were done capturing
			if (imageTestFlag != 0) printf("\nStep 4 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			 // -------- Filter on HSV Threshold to get just the bright green pixels ----------
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold);
			if (imageTestFlag != 0) printf("\nStep 5 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			if (imageTestFlag != 0) thresholdImage->Write("/Threshold.bmp");	// save image for later debugging
			if (imageTestFlag != 0) printf("\nStep 6 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
		
			
  			// ---------- Filter to Remove Small Particles ------------------------------
			BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);
			if (imageTestFlag != 0) printf("\nStep 7 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			if (imageTestFlag != 0)filteredImage->Write("/Filtered.bmp");	// save an image for later debugging
			if (imageTestFlag != 0) printf("\nStep 8 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			// ------------ get a particle analysis report for each particle ------------
			vector<ParticleAnalysisReport> *reports = 
						filteredImage->GetOrderedParticleAnalysisReports();
			if (imageTestFlag != 0) printf("\nStep 9 - %f ",Timer::GetFPGATimestamp()- imageTime);	
			imageTime = Timer::GetFPGATimestamp();
			
			if (imageTestFlag != 0) printf("\nNumber of particles found Report Size= %d\n", reports->size());

			// Iterate through each particle, scoring it, determining whether it is a target or not
			verticalTargetCount = horizontalTargetCount = 0;
			if(reports->size() > 0)
			{
				scores = new Scores[reports->size()];
				for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
					ParticleAnalysisReport *report = &(reports->at(i));
					
					//Score each particle on rectangularity and aspect ratio
					scores[i].rectangularity = scoreRectangularity(report);
					scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
					scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			
					
					//Check if the particle is a horizontal target or if it's a vertical target
					if(scoreCompare(scores[i], false))
					{
						if (imageTestFlag != 0) printf("particle: %d  is a Horizontal Target centerX: %d  centerY: %d \n",
								i, report->center_mass_x, report->center_mass_y);
						//Add particle to target array and increment count
						horizontalTargets[horizontalTargetCount++] = i;
					} else if (scoreCompare(scores[i], true)) {
						if (imageTestFlag != 0) printf("particle: %d  is a Vertical Target centerX: %d  centerY: %d \n",
								i, report->center_mass_x, report->center_mass_y);
						//Add particle to target array and increment count
						verticalTargets[verticalTargetCount++] = i;
					} else {
						if (imageTestFlag != 0) printf("particle: %d  is not a Target centerX: %d  centerY: %d \n"
								, i, report->center_mass_x, report->center_mass_y);
					}
					if (imageTestFlag != 0) printf("Scores rect: %f  ARvert: %f \n",
							scores[i].rectangularity, scores[i].aspectRatioVertical);
					if (imageTestFlag != 0) printf("ARhoriz: %f  \n", scores[i].aspectRatioHorizontal);	
				}

				// Zero out scores and set verticalIndex to first target in case there are no
				// horizontal targets
				target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore =
						target.verticalScore = 0;
				target.verticalIndex = verticalTargets[0];
				for (int i = 0; i < verticalTargetCount; i++)
				{
					ParticleAnalysisReport *verticalReport =
							&(reports->at(verticalTargets[i]));
					for (int j = 0; j < horizontalTargetCount; j++)
					{
						ParticleAnalysisReport *horizontalReport =
								&(reports->at(horizontalTargets[j]));
						double horizWidth, horizHeight, vertWidth, leftScore, rightScore;
						double tapeWidthScore, verticalScore, total;
	
						// Measure equivalent rectangle sides for use in score calculation
						imaqMeasureParticle(filteredImage->GetImaqImage(),
								horizontalReport->particleIndex, 0,
								IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(),
								verticalReport->particleIndex, 0,
								IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(),
								horizontalReport->particleIndex, 0,
								IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);
						
						// Determine if the horizontal target is to the left of the vertical target
						leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - 
								horizontalReport->center_mass_x)/horizWidth);
						
						// Determine if the horizontal target is to the right of vertical target
						rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x -
								verticalReport->boundingRect.left -
								verticalReport->boundingRect.width)/horizWidth);
						
						// Determine if the width of tape on the two targets appears to be the same
						tapeWidthScore = ratioToScore(vertWidth/horizHeight);
						
						// Determine if the vertical location of the horizontal target
						// appears to be correct
						verticalScore = ratioToScore(1-(verticalReport->boundingRect.top -
								horizontalReport->center_mass_y)/(4*horizHeight));
						
						total = leftScore > rightScore ? leftScore:rightScore;
						total += tapeWidthScore + verticalScore;
						
						// If the target is the best detected so far store the information about it
						if(total > target.totalScore)
						{
							target.horizontalIndex = horizontalTargets[j];
							target.verticalIndex = verticalTargets[i];
							target.totalScore = total;
							target.leftScore = leftScore;
							target.rightScore = rightScore;
							target.tapeWidthScore = tapeWidthScore;
							target.verticalScore = verticalScore;
						}
					}
					// Determine if the best target is a Hot target
					target.Hot = hotOrNot(target);
				}

				if (imageTestFlag != 0) printf("\nStep 10 - %f ",Timer::GetFPGATimestamp()- imageTime);	
				imageTime = Timer::GetFPGATimestamp();
				
				if(verticalTargetCount > 0)
				{
					// Information about the target is contained in the "target" structure
					// To get measurement information such as sizes or locations use the
					// horizontal or vertical index to get the particle report as shown below
					ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
					double distance = computeDistance(filteredImage, distanceReport);
					if(target.Hot)
					{
						printf(" **********Hot target located ******** \n");
						printf("Distance: %f \n", distance);
						temptarget.hot = true;
						temptarget.distance = distance;
						msg1 =  "Hot target located Distance:" + float_to_string(distance);
						
					} else {
						printf("********* NO hot target present ******** \n");
						printf("Distance: %f \n", distance);
						temptarget.hot = false;
						temptarget.distance = distance;
						msg1 =  "Hot target NOT Prfesent:" ;	
					}
					if (imageTestFlag != 0) printf("\nCompleted Vision Proccessing: %f \n",Timer::GetFPGATimestamp()-imageTimeTotal);
					if (imageTestFlag != 0) printf("\nStep 11 - %f \n",Timer::GetFPGATimestamp()- imageTime);	
					imageTime = Timer::GetFPGATimestamp() - imageTime;
					
					updateSmartDash();	
				}	
			}
			
			else if (imageTestFlag != 0) printf("\n****** No Reports found !! \n");

			if (imageTestFlag != 0) printf("Step 12 - %f \n",Timer::GetFPGATimestamp()- imageTime);	
			printf("Completed Vision Proccessing: %f \n",Timer::GetFPGATimestamp() - imageTimeTotal);
			
			// be sure to delete images after using them
			// delete filteredImage;
			// delete thresholdImage;
			// delete image;

			// delete allocated reports and Scores objects also
			delete scores;
			delete reports;
		return temptarget;
	} // end of lookForTarget function

	
	
	
	//  ----------------------------------------------------------------------------
	// ---------------------- Compute Distance to Target ---------------------------
	// -----------------------------------------------------------------------------
	 /*
	 * Computes the estimated distance to a target using the height of the particle in the image.
	 * For more information and graphics showing the math behind this approach see the Vision
	 * Processing section of the ScreenStepsLive documentation.
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param outer True if the particle should be treated as an outer target, false to treat it
	 *   as a center target
	 * @return The estimated distance to the target in Inches.
	 */
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report) {
		double rectLong, height;
		int targetHeight;
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
				IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		// using the smaller of the estimated rectangle short side and the bounding rectangle
		// height results in better performance on skewed rectangles
		height = min(report->boundingRect.height, rectLong);
		targetHeight = 32;
		return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}

	
	//  ----------------------------------------------------------------------------
	// ---------------------- Compute Aspect Ratio of Targets ----------------------
	// -----------------------------------------------------------------------------
	/**
	 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target.
	 * This method uses the equivalent rectangle sides to determine aspect ratio as it performs
	 * better as the target gets skewed by moving to the left or right. The equivalent rectangle
	 * is the rectangle with sides x and y where particle area= x*y and particle perimeter= 2x+2y
	 * 
	 * @param image The image containing the particle to score, needed to perform additional
	 *    measurements
	 * @param report The Particle Analysis Report for the particle, used for the width, height,
	 *    and particle number
	 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio
	 *    for the inner target or the outer
	 * @return The aspect ratio score (0-100)
	 */
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical){
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		
		//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
		idealAspectRatio = vertical ? (4.0/32) : (23.5/4);
		
		imaqMeasureParticle(image->GetImaqImage(),
				report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(),
				report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		
		//Divide width by height to measure aspect ratio
		if(report->boundingRect.width > report->boundingRect.height){
			//particle is wider than it is tall, divide long by short
			aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
		} else {
			//particle is taller than it is wide, divide short by long
			aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
		}
		return aspectRatio;		//force to be in range 0-100
	}
	
	
	//  ----------------------------------------------------------------------------
	// ------------------ Compares Scores of Particals to Targets ------------------
	// -----------------------------------------------------------------------------
	/**
	 * Compares scores to defined limits and returns true if the particle appears to be a target
	 * 
	 * @param scores The structure containing the scores to compare
	 * @param outer True if the particle should be treated as an outer target, false to treat it
	 *         as a center target
	 * @return True if the particle meets all limits, false otherwise
	 */
	bool scoreCompare(Scores scores, bool vertical){
		bool isTarget = true;
		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if(vertical){
			isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
		}
		return isTarget;
	}
	
	
	//  ----------------------------------------------------------------------------
	// -------------------- Score Rectangularity of Partical -----------------------
	// -----------------------------------------------------------------------------
	/**
	 * Computes a score (0-100) estimating how rectangular the particle is by comparing
	 * the area of the particle to the area of the bounding box surrounding it. A
	 * perfect rectangle would cover the entire bounding box.
	 * 
	 * @param report The Particle Analysis Report for the particle to score
	 * @return The rectangularity score (0-100)
	 */
	double scoreRectangularity(ParticleAnalysisReport *report){
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
		} else {
			return 0;
		}	
	}
	

	// ----------------------------------------------------------------------
	// ------------  Simple Calculation Routine -----------------------------
	// ----------------------------------------------------------------------
	double ratioToScore(double ratio)
	{
		return (max(0, min(100*(1-fabs(1-ratio)), 100)));
	}
	
	// ----------------------------------------------------------------------
	// ------------  Hot on Not Routine -----------------------------
	// ----------------------------------------------------------------------
	bool hotOrNot(TargetReport target)
	{
		bool isHot = true;
		
		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
				
		return isHot;
	}	
	
}; // end of Class


START_ROBOT_CLASS(RobotDemo);

