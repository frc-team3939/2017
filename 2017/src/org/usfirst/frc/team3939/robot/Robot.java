package org.usfirst.frc.team3939.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;






/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot  implements PIDOutput {
	AHRS ahrs;
	
	String autoSelected;
	Boolean gearSelected, shotSelected;
	
	//SendableChooser<String> DriveType = new SendableChooser<>();
    SendableChooser<Boolean> DelGear = new SendableChooser<>();
    SendableChooser<Double> shooterSpeed = new SendableChooser<>();
    SendableChooser<Boolean> DelShot = new SendableChooser<>();
    SendableChooser<String> atomtype = new SendableChooser<>();
    
	RobotDrive robotDrive;
	
	PowerDistributionPanel pdp;

	// Channels for the wheels
	CANTalon kFrontLeftChannel = new CANTalon(27); 		/* device IDs here (1 of 2) */
	CANTalon kRearLeftChannel = new CANTalon(21);
	CANTalon kFrontRightChannel = new CANTalon(28);
	CANTalon kRearRightChannel = new CANTalon(24);
	
	DigitalOutput shooterlight, gearlight;
	
	DigitalInput lSwitch, rSwitch;
    	
	CANTalon kGearRight = new CANTalon(29);
	CANTalon kGearLeft = new CANTalon(22);
	double ClosedPosition = 0.0;
	double OpenPosition = 0.25;
	
	CANTalon ShooterMotor = new CANTalon(23); 
	double ShooterPower = 0;
	
	CANTalon IntakeMotor = new CANTalon(20); 
	double IntakePower = 0.6;

	Talon RightConveyorMotor, LeftConveyorMotor; 
	double ConveyorPower = 1;

	Talon Climb1Motor, Climb2Motor; 
	double ClimbPower = 1;
	
	Servo ShooterStop;
	double ShooterStopClosedLoc = .5;
	double ShooterStopOpenLoc = .7;
	 
	Joystick stick = new Joystick(0);
	Joystick gamepad = new Joystick(1);
	
	boolean IntakeSetarted = false, ConveyorStarted = false, ShooterStarted = false, ClimbStarted = false;
	
	// turn to angle stuff
	PIDController turnController;
    double rotateToAngleRate;
    double currentRotationRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.05;
    static final double kI = 0.1;
    static final double kD = 0.05;
    static final double kF = 0.0;
    
    static final double kToleranceDegrees = 0.5f;
    // end turn to angle stuff
	
    //drive to distance stuff
    double autodrivepower = .4;
    Encoder drivewheelencoder = new Encoder(0, 1, true, EncodingType.k4X);
    public static final double WHEEL_DIAMETER = 4; //Will need to be set before use
	public static final double PULSE_PER_REVOLUTION = 1440;
	public static final double ENCODER_GEAR_RATIO = 1;
	public static final double GEAR_RATIO = 12.75;
	public static final double FUDGE_FACTOR = 1.0;
	final double distancePerPulse = Math.PI * WHEEL_DIAMETER / PULSE_PER_REVOLUTION;
	// end drive to distance
	
	//Collision Detection 
	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
	final static double kCollisionThreshold_DeltaG = 0.5f;
	// end Collision stuff 
	
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		smartDashBoardBsetup();
		
		initTalonEncoders();
		
		//CameraServer.getInstance().startAutomaticCapture(); //USB Cameras
		
		ShooterStop = new Servo(8);  //ShooterStop
		ShooterStop.set(ShooterStopClosedLoc); // set start location

		shooterlight = new DigitalOutput(9);
		gearlight = new DigitalOutput(8);
		
		lSwitch = new DigitalInput(0);
        rSwitch = new DigitalInput(1);
		
		RightConveyorMotor = new Talon(2); //set PMW Location
		LeftConveyorMotor = new Talon(1); //set PMW Location

		Climb1Motor = new Talon(4); //set PMW Location
		Climb2Motor = new Talon(5); //set PMW Location

		robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert the left side motors
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); // you may need to change or remove this to match your robot
		robotDrive.setExpiration(0.1);
		
		try {
	          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
	          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
	          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
	          ahrs = new AHRS(SPI.Port.kMXP); 
	      } catch (RuntimeException ex ) {
	          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
	      }
		
		// rotate to angle stuff
				turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		        turnController.setInputRange(-180.0f,  180.0f);
		        turnController.setOutputRange(-.4, .4);
		        turnController.setAbsoluteTolerance(kToleranceDegrees);
		        turnController.setContinuous(true);
		        
		        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		        /* tuning of the Turn Controller's P, I and D coefficients.            */
		        /* Typically, only the P value needs to be modified.                   */
		        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		// end rotate to angle 	
		
		drivewheelencoder.setDistancePerPulse(distancePerPulse);
		drivewheelencoder.reset();
		
		shooterlight.set(true);
		gearlight.set(true);
		
		pdp = new PowerDistributionPanel(0);
		
	
  	  
		        
	}

	public void initTalonEncoders(){
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		//int absolutePositionRight = kGearRight.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
        /* use the low level API to set the quad encoder signal */
		kGearRight.setPosition(0);
		kGearRight.setEncPosition(0);
        
        /* choose the sensor and sensor direction */
        kGearRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        kGearRight.changeControlMode(TalonControlMode.PercentVbus);
        kGearRight.set(0);
        kGearRight.reverseSensor(false);
        kGearRight.configEncoderCodesPerRev(497); // if using FeedbackDevice.QuadEncoder
        //kGearRight.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        kGearRight.configNominalOutputVoltage(+0f, -0f);
        kGearRight.configPeakOutputVoltage(+12f, -12f);
        /* set the allowable closed-loop error,
         * Closed-Loop output will be neutral within this range.
         * See Table in Section 17.2.1 for native units per rotation. 
         */
        kGearRight.setAllowableClosedLoopErr(0); /* always servo */
        /* set closed loop gains in slot0 */
        kGearRight.setInverted(true);
        kGearRight.enableForwardSoftLimit(true);
        kGearRight.setForwardSoftLimit(0);
        kGearRight.enableReverseSoftLimit(true);
        kGearRight.setReverseSoftLimit(-.25);
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		//int absolutePositionLeft = kGearRight.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
        /* use the low level API to set the quad encoder signal */
		kGearLeft.setPosition(0);
		kGearLeft.setEncPosition(0);
        
        /* choose the sensor and sensor direction */
        kGearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        kGearLeft.changeControlMode(TalonControlMode.PercentVbus);
        kGearLeft.set(0);
        kGearLeft.reverseSensor(false);
        kGearLeft.configEncoderCodesPerRev(497); // if using FeedbackDevice.QuadEncoder
        //kGearRight.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        kGearLeft.configNominalOutputVoltage(+0f, -0f);
        kGearLeft.configPeakOutputVoltage(+12f, -12f);
        /* set the allowable closed-loop error,
         * Closed-Loop output will be neutral within this range.
         * See Table in Section 17.2.1 for native units per rotation. 
         */
        kGearLeft.setAllowableClosedLoopErr(0); /* always servo */
        /* set closed loop gains in slot0 */
        kGearLeft.enableForwardSoftLimit(true);
        kGearLeft.setForwardSoftLimit(.25);
        kGearLeft.enableReverseSoftLimit(true);
        kGearLeft.setReverseSoftLimit(0);	
	}
	
	public void fixgears(){
		kGearRight.enableForwardSoftLimit(false);
        kGearRight.enableReverseSoftLimit(false);
        kGearLeft.enableForwardSoftLimit(false);
        kGearLeft.enableReverseSoftLimit(false);
        
		while (!lSwitch.get()) {
			kGearLeft.set(-.1); 
		}
		kGearLeft.set(0);
		
		while (!rSwitch.get()) {
			kGearRight.set(-.1); 
		}
		kGearRight.set(0);
		
		initTalonEncoders();
	}

	public void opengears() {
		kGearRight.set(.4); 
		kGearLeft.set(.4); 
	}
	
	public void closegears() {
		kGearRight.set(-.4); 
		kGearLeft.set(-.4); 
	}
	
	public void startintake() {
		IntakeSetarted = true;
		IntakeMotor.set(-IntakePower); 
    }
	
	public void stopintake() {
		IntakeSetarted = false;
		//IntakeMotor.stopMotor();
		IntakeMotor.set(0);
    }
	
	public void reverseintake() {
		IntakeMotor.set(IntakePower); 
    }

	public void startConveyor() { 
		ConveyorStarted = true;
		RightConveyorMotor.set(-ConveyorPower); 
		LeftConveyorMotor.set(ConveyorPower); 
    }
	
	public void stopConveyor() {
		ConveyorStarted = false;
		RightConveyorMotor.stopMotor();
		LeftConveyorMotor.stopMotor();
    }
	
	public void reverseConveyor() {
		RightConveyorMotor.set(ConveyorPower); 
		LeftConveyorMotor.set(-ConveyorPower); 
    }

	public void startClimb() {
		//while (stick.getRawButton(7)) {
			ClimbStarted = true;
			Climb1Motor.set(ClimbPower); 
			Climb2Motor.set(ClimbPower);
		//}
		//stopClimb();
    }
	
	public void stopClimb() {
		ClimbStarted = false;
		Climb1Motor.stopMotor();
		Climb2Motor.stopMotor();
    }
	
	public void reverseClimb() {
		Climb1Motor.set(-ClimbPower); 
		Climb2Motor.set(-ClimbPower); 
    }
	
	public void startshooter() {
		ShooterPower = shooterSpeed.getSelected();
		ShooterStarted = true;
		//ShooterPower = .85; //set shooter power level
		ShooterMotor.set(ShooterPower); 
    	Timer.delay(1);
    	ShooterStop.set(ShooterStopOpenLoc); //Shooter Servo location
    	//need to start conveyor
    }
	
	public void stopshooter() {
		ShooterStarted = false;
		//need to stop conveyor
		ShooterStop.set(ShooterStopClosedLoc); //Shooter Servo location
		//Timer.delay(3.0);
    	//ShooterMotor.stopMotor();			
    	ShooterMotor.set(0);			
	}
	
	public void smartDashBoardBsetup() {
		
		//DriveType.addObject("Field Drive", "Field");
    	//DriveType.addDefault("Normal Drive", "Normal");
    	//SmartDashboard.putData("Drive Type", DriveType);
		
    	DelGear.addObject("Yes", true);
    	DelGear.addDefault("No", false);
    	SmartDashboard.putData("Deliver Gear", DelGear);
    	
    	DelShot.addObject("Yes", true);
    	DelShot.addDefault("No", false);
    	SmartDashboard.putData("Deliver Shot", DelShot);
    	
    	atomtype.addObject("Red 1", "Red1");
    	atomtype.addObject("Red 2", "Red2");
    	atomtype.addObject("Red 3", "Red3");
    	atomtype.addObject("Blue 1", "Blue1");
    	atomtype.addObject("Blue 2", "Blue2");
    	atomtype.addObject("Blue 3", "Blue3");
    	atomtype.addDefault("None", "None");
    	SmartDashboard.putData("Autonomous Type", atomtype);
    	
    	
    	shooterSpeed.addObject("100%", 1.0);
    	shooterSpeed.addObject("95%", 0.95);
    	shooterSpeed.addObject("90%", 0.9);
    	shooterSpeed.addObject("85%", 0.85);
    	shooterSpeed.addDefault("80%", 0.80);
    	shooterSpeed.addObject("75%", 0.75);
    	shooterSpeed.addObject("70%", 0.7);
    	shooterSpeed.addObject("65%", .65);
    	shooterSpeed.addObject("60%", .6);
    	shooterSpeed.addObject("55%", .55);
    	shooterSpeed.addObject("50%", .5);
    	SmartDashboard.putData("Shooter Speed %", shooterSpeed);
    	
    	/*
    	HeightOffset.addObject("160", "160");
    	HeightOffset.addObject("150", "150");
    	HeightOffset.addObject("140", "140");
    	HeightOffset.addObject("130", "130");
    	HeightOffset.addObject("120", "120");
    	HeightOffset.addDefault("110", "110");
    	HeightOffset.addObject("100", "100");
    	HeightOffset.addObject("90", "90");
    	HeightOffset.addObject("80", "80");
    	HeightOffset.addObject("70", "70");
    	HeightOffset.addObject("60", "60");
    	HeightOffset.addObject("50", "50");
    	HeightOffset.addObject("40", "40");
    	HeightOffset.addObject("30", "30");
    	HeightOffset.addObject("20", "20");
    	HeightOffset.addObject("10", "10");
    	SmartDashboard.putData("HeightOffset", HeightOffset);
    	*/
	}

	public void smartDashBoardDisplay() {
        /* Display 6-axis Processed Angle Data                                      */
        //SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        //SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        //SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        //SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        //SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
		/* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        //SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        //SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
        
        //SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        //SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        //SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        //SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        //SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        //SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        //SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
        //SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
        //SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
        //SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        //SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        //SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        //SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        //SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        //SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        //SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
        //SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
        //SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
        //SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
        //SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        //AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        //SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        //SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        //SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        //SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        //SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        //SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        //SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        //SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        //SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
        
        
	}
	
	public void driveforward(double inches){
		drivewheelencoder.reset();
		double encoderDistanceReading = drivewheelencoder.get();
		SmartDashboard.putNumber("driveencoder reading", -encoderDistanceReading);
		double throttle;
		if (inches > 0) { 
			throttle = -autodrivepower;
		} else {
			throttle = autodrivepower;
		}
		
		while (Math.abs(inches)*15 > Math.abs(encoderDistanceReading)){
			encoderDistanceReading = drivewheelencoder.get();
			SmartDashboard.putNumber("driveencoder reading", -encoderDistanceReading);
			kFrontLeftChannel.set(throttle);
			kRearLeftChannel.set(throttle); 
			kFrontRightChannel.set(throttle);
			kRearRightChannel.set(throttle);
			if (stick.getRawButton(12)) {
	        	break;
	        }
			if (DetectCollision()) {
				break;
			}
		} 
		kFrontLeftChannel.stopMotor();
		kRearLeftChannel.stopMotor();
		kFrontRightChannel.stopMotor(); 
		kRearRightChannel.stopMotor();
	}
	
	public void autoshoot(){
		double curangle = ahrs.getAngle();
		double b_angle = SmartDashboard.getDouble("b_angle");
		double shootangle = curangle - 90 + b_angle;
		double turnangle = shootangle;
		turntoangle(turnangle);
		double b_height = SmartDashboard.getDouble("b_heigth");
		double b_distance = (10 * 270)/b_height;   //D = (H x F) / P
		//ShooterPower = shooterSpeed.getSelected();
		double curpower =  pdp.getVoltage();
		ShooterPower = (0.25096*b_distance+54.455065)*(12/curpower); // shotpower = (0.25096*distance+54.455065)*(12/currpower)
		ShooterStarted = true;
		//ShooterPower = .85; //set shooter power level
		ShooterMotor.set(ShooterPower); 
    	Timer.delay(1);
    	ShooterStop.set(ShooterStopOpenLoc); //Shooter Servo location
    	//need to start conveyor
	}
	
	public void autogear(){
		double curangle = ahrs.getAngle();
		double g_angle = SmartDashboard.getDouble("g_angle");
		double shootangle = curangle - 90 + g_angle;
		double turnangle = shootangle;
		turntoangle(turnangle);
		double g_width = SmartDashboard.getDouble("g_width");
		double g_distance = (10 * 270)/g_width;   //D = (H x F) / P
		driveforward(g_distance);
		opengears();
		Timer.delay(0.5);
		driveforward(-18);
		closegears();
	}
			
	public void turntoangle(double angle){
		turnController.setSetpoint(angle);
        turnController.enable();
        double hiloc = angle + kToleranceDegrees;
        double lowloc = angle - kToleranceDegrees;
        SmartDashboard.putNumber("currloc", ahrs.getAngle());
        SmartDashboard.putNumber("rotateToAngleRate", rotateToAngleRate);
        SmartDashboard.putBoolean("onTarget", turnController.onTarget());
        SmartDashboard.putNumber("lowloc", lowloc);
        SmartDashboard.putNumber("hiloc", hiloc);
	       
        while(ahrs.getAngle() < lowloc || ahrs.getAngle() >hiloc){
	        SmartDashboard.putNumber("currloc", ahrs.getAngle());
	        SmartDashboard.putNumber("rotateToAngleRate", rotateToAngleRate);
	        SmartDashboard.putBoolean("onTarget", turnController.onTarget());
	        
	        currentRotationRate = rotateToAngleRate;
	        robotDrive.mecanumDrive_Cartesian(0, 0, -currentRotationRate, ahrs.getAngle());
	        if (stick.getRawButton(12)) {
	        	break;
	        }
		}
	}
	
	public boolean DetectCollision (){
		boolean collisionDetected = false;
        double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        
        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) || ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) { 
            collisionDetected = true;   
        }
        SmartDashboard.putBoolean(  "CollisionDetected", collisionDetected);
        return collisionDetected;
	}
	
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	@Override
	public void autonomousInit() {
		autoSelected = atomtype.getSelected();
		gearSelected = DelGear.getSelected();
		shotSelected = DelShot.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		switch (autoSelected) {
		case "Red1":
			ahrs.reset();
			driveforward(105); // drive forward 48 inches ?
			Timer.delay(1.0);		    //    for 2 seconds
			turntoangle(30f);
			Timer.delay(1.0);
			driveforward(49);  // 50 for gear
			if (gearSelected) {
				opengears();
	  		  	Timer.delay(0.5);
	  		  	driveforward(-18);
	  		  	closegears();
			}
			break;
			
		case "Red2":
			ahrs.reset();
			driveforward(109);
			if (gearSelected) {
				opengears();
		  	  	Timer.delay(0.5);
		  	  	driveforward(-18);
		  	  	closegears();
			}
			if (shotSelected) {
				ahrs.reset();
				turntoangle(10f);
				autoshoot();
			}
			break;

		case "Red3":
			ahrs.reset();
			driveforward(118);
			Timer.delay(1.0);
			turntoangle(-30f);
			Timer.delay(1.0);
			driveforward(22);
			if (gearSelected) {
				opengears();
		  	  	Timer.delay(0.5);
		  	  	driveforward(-18);
		  	  	closegears();
			}
			if (shotSelected) {
				ahrs.reset();
				turntoangle(110f);
				autoshoot();
			}
			break;

		case "Blue1":
			ahrs.reset();
			driveforward(118); // drive forward 48 inches ?
			Timer.delay(1.0);		    //    for 2 seconds
			turntoangle(30f);
			Timer.delay(1.0);
			driveforward(22);  // 50 for gear
			if (gearSelected) {
				opengears();
		  	  	Timer.delay(0.5);
		  	  	driveforward(-18);
		  	  	closegears();
			}
			if (shotSelected) {
				ahrs.reset();
				turntoangle(80f);
				autoshoot();
			}
			break;

		case "Blue2":
			ahrs.reset();
			driveforward(109);
			if (gearSelected) {
				opengears();
		  	  	Timer.delay(0.5);
		  	  	driveforward(-18);
		  	  	closegears();
			}
			if (shotSelected) {
				ahrs.reset();
				turntoangle(170f);
				autoshoot();
			}
			break;

		case "Blue3":
			ahrs.reset();
			driveforward(105); 
			Timer.delay(1.0);
			turntoangle(-30f);
			Timer.delay(1.0);
			driveforward(49);
			if (gearSelected) {
				opengears();
		  	  	Timer.delay(0.5);
		  	  	driveforward(-18);
		  	  	closegears();
			}
			break;

		case "None":
		default:
			// Do Nothing
			break;
		}
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {	

	}

	/**
	 * This function is called periodically during operator control 
	 */
	@Override
	public void teleopPeriodic() {
		robotDrive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			
           
			// Use the joystick X axis for lateral movement, Y axis for forward
			// movement, and Z axis for rotation.
			// This sample does not use field-oriented drive, so the gyro input
			// is set to zero.
			
			//switch (DriveType.getSelected()) {
			//case "Field":
				//field drive
				//if ( stick.getTrigger() ) {//trigger same as button 1
		        //      ahrs.zeroYaw();
		        //}
			//	robotDrive.mecanumDrive_Cartesian(-stick.getX(), -stick.getY(), -stick.getZ(), ahrs.getAngle());
			//	break;
			//case "Normal":
			//default:
				//regular drive
				robotDrive.mecanumDrive_Cartesian(-stick.getX(), -stick.getY(), (-stick.getZ()*.5), 0);
			//	break;
			//}
						
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
						

	          smartDashBoardDisplay();
	          
	          if (stick.getRawButton(1)) {
	        	  startshooter();
	          }
	          if (stick.getRawButton(2)) {
	        	  stopshooter();
	          }
	          if (stick.getRawButton(3)) {
	        	  startintake();
	        	  startConveyor();
	          }
	          if (stick.getRawButton(4)) {
	        	  stopConveyor();
	          }
        	  if (stick.getRawButton(5)) { 
        		  opengears();
        		  Timer.delay(0.5);
        		  driveforward(-18);
        		  closegears();
        	  }
	          if (stick.getRawButton(6)) {
	        	  stopintake();
	          }
	          if (stick.getRawButton(7)) {
	        	  startClimb(); 
		      }
	          if (stick.getRawButton(8)) {
	        	  stopClimb();
	          }
	          if (stick.getRawButton(9)) {
	        	  autoshoot();
	          }
	          if (stick.getRawButton(10)) {
	        	  autogear();
	          }
	          if (stick.getRawButton(11)) {
	        	  reverseintake();
	          }
	          if (stick.getRawButton(12)) {
	        	  // used for abort code
	          }
	          
	          
	          
	          if (gamepad.getRawButton(1)) {
	        	  stopshooter();
	          }
	          if (gamepad.getRawButton(2)) { //purge button
	        	  reverseintake();
		          reverseConveyor();
	          }
	          if (gamepad.getRawButton(3)) {
	        	  stopintake();
	          }
	          if (gamepad.getRawButton(4)) {
	        	  reverseintake();
	          }
        	  if (gamepad.getRawButton(5)) { 
	        	  stopConveyor();
        	  }
	          if (gamepad.getRawButton(6)) {
		          //fixgears();  // need limit switches installed
	          }
	          if (gamepad.getRawButton(7)) {
	        	  stopClimb();
		      }
	          if (gamepad.getRawButton(8)) {
	        	  reverseClimb();  
	          }
	          if (gamepad.getRawButton(9)) {
	        	  opengears();
	          }
	          if (gamepad.getRawButton(10)) {
        		  closegears();
	          }

	          
        	  //double encoderDistanceReading = drivewheelencoder.get(); 
      		  SmartDashboard.putNumber("Drive encoder reading", -drivewheelencoder.get());
      		  //System.out.println(encoderDistanceReading);
      		  SmartDashboard.putNumber("kGearRight", kGearRight.getPosition() );
      		  SmartDashboard.putNumber("kGearLeft", kGearLeft.getPosition() );
      		  SmartDashboard.putNumber("currloc", ahrs.getAngle());
  	       
		}  
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override 
	public void testPeriodic() { 
	}

	@Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
	
}