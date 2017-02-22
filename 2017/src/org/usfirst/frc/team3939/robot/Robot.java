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
public class Robot extends IterativeRobot  implements PIDOutput{
	AHRS ahrs;
	
	String autoSelected;
	SendableChooser<String> DriveType = new SendableChooser<>();
    //SendableChooser<String> MaxSpeed = new SendableChooser<>();
    //SendableChooser<String> shooterSpeed = new SendableChooser<>();
    //SendableChooser<String> HeightOffset = new SendableChooser<>();
    SendableChooser<String> atomtype = new SendableChooser<>();
    
	RobotDrive robotDrive, tankDrive;

	// Channels for the wheels
	CANTalon kFrontLeftChannel = new CANTalon(27); 		/* device IDs here (1 of 2) */
	CANTalon kRearLeftChannel = new CANTalon(21);
	CANTalon kFrontRightChannel = new CANTalon(28);
	CANTalon kRearRightChannel = new CANTalon(24);
	
	DigitalOutput shooterlight, gearlight;
	
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
	double ShooterStopClosedLoc = .55;
	double ShooterStopOpenLoc = .7;
	 
	Joystick stick = new Joystick(0);
	
	boolean IntakeSetarted = false, ConveyorStarted = false, ShooterStarter = false, ClimbStarted = false;
	
	// turn to angle stuff
	PIDController turnController;
    double rotateToAngleRate;
    double currentRotationRate;
    
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;
    // end turn to angle stuff
	
    //drive to distance stuff
    Encoder drivewheelencoder = new Encoder(0, 1, true, EncodingType.k4X);
    public static final double WHEEL_DIAMETER = 4; //Will need to be set before use
	public static final double PULSE_PER_REVOLUTION = 1440;
	public static final double ENCODER_GEAR_RATIO = 0;
	public static final double GEAR_RATIO = 12.75 / 1;
	public static final double FUDGE_FACTOR = 1.0;
	final double distancePerPulse = Math.PI * WHEEL_DIAMETER / PULSE_PER_REVOLUTION / ENCODER_GEAR_RATIO / GEAR_RATIO * FUDGE_FACTOR;
	// end drive to distance
	
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
        kGearRight.enableForwardSoftLimit(true);
        kGearRight.setForwardSoftLimit(OpenPosition);
        kGearRight.enableReverseSoftLimit(true);
        kGearRight.setReverseSoftLimit(ClosedPosition);
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
        kGearLeft.setForwardSoftLimit(ClosedPosition);
        kGearLeft.enableReverseSoftLimit(true);
        kGearLeft.setReverseSoftLimit(-OpenPosition);	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		smartDashBoardBsetup();
		
		initTalonEncoders();
		
		CameraServer.getInstance().startAutomaticCapture(); //USB Cameras
		
		ShooterStop = new Servo(8);  //ShooterStop
		ShooterStop.set(ShooterStopClosedLoc); // set start location

		shooterlight = new DigitalOutput(9);
		gearlight = new DigitalOutput(8);
		
		RightConveyorMotor = new Talon(2); //set PMW Location
		LeftConveyorMotor = new Talon(1); //set PMW Location

		Climb1Motor = new Talon(4); //set PMW Location
		Climb2Motor = new Talon(5); //set PMW Location

		robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert the
																	// left side
																	// motors
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); // you may need
																// to change or
																// remove this
																// to match your
																// robot
		robotDrive.setExpiration(0.1);
		
		tankDrive = new RobotDrive(kRearLeftChannel, kRearRightChannel);
		
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
		        turnController.setOutputRange(-1.0, 1.0);
		        turnController.setAbsoluteTolerance(kToleranceDegrees);
		        turnController.setContinuous(true);
		        
		        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		        /* tuning of the Turn Controller's P, I and D coefficients.            */
		        /* Typically, only the P value needs to be modified.                   */
		        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		// end rotate to angle 	
		
		drivewheelencoder.setDistancePerPulse(distancePerPulse);
		drivewheelencoder.reset();
  	  
		        
	}

	public void opengears() {
		kGearRight.set(-.4); 
		kGearLeft.set(.4); 
	}
	
	public void closegears() {
		kGearRight.set(.4); 
		kGearLeft.set(-.4); 
	}
	
	public void startintake() {
		IntakeMotor.set(-IntakePower); 
    }
	public void stopintake() {
		//IntakeMotor.stopMotor();
		IntakeMotor.set(0);
    }
	public void reverseintake() {
		IntakeMotor.set(IntakePower); 
    }

	public void startConveyor() { 
		RightConveyorMotor.set(-ConveyorPower); 
		LeftConveyorMotor.set(ConveyorPower); 
    }
	public void stopConveyor() {
		RightConveyorMotor.stopMotor();
		LeftConveyorMotor.stopMotor();
    }
	public void reverseConveyor() {
		RightConveyorMotor.set(ConveyorPower); 
		LeftConveyorMotor.set(-ConveyorPower); 
    }

	public void startClimb() {
		Climb1Motor.set(ClimbPower); 
		Climb2Motor.set(ClimbPower); 
    }
	public void stopClimb() {
		Climb1Motor.stopMotor();
		Climb2Motor.stopMotor();
    }
	public void reverseClimb() {
		Climb1Motor.set(-ClimbPower); 
		Climb2Motor.set(-ClimbPower); 
    }
	
	public void startshooter() {
		ShooterPower = .85; //set shooter power level
		ShooterMotor.set(ShooterPower); 
    	Timer.delay(1);
    	ShooterStop.set(ShooterStopOpenLoc); //Shooter Servo location
    	//need to start conveyor
    }
	
	public void stopshooter() {
		//need to stop conveyor
		ShooterStop.set(ShooterStopClosedLoc); //Shooter Servo location
		//Timer.delay(3.0);
    	//ShooterMotor.stopMotor();			
    	ShooterMotor.set(0);			
	}
	
	
	public void smartDashBoardBsetup() {
		
		DriveType.addObject("Field Drive", "Field");
    	DriveType.addDefault("Normal Drive", "Normal");
    	SmartDashboard.putData("Drive Type", DriveType);
		
    	//MaxSpeed.addDefault("100%", "1");
    	//MaxSpeed.addObject("90%", ".9");
    	//MaxSpeed.addObject("80%", ".8");
    	//MaxSpeed.addObject("70%", ".7");
    	//MaxSpeed.addObject("60%", ".6");
    	//MaxSpeed.addObject("50%", ".5");
    	//MaxSpeed.addObject("40%", ".4");
    	//MaxSpeed.addObject("30%", ".3");
    	//MaxSpeed.addObject("20%", ".2");
    	//SmartDashboard.putData("Max Speed %", MaxSpeed);
    	
    	atomtype.addObject("Red 1", "Red1");
    	atomtype.addObject("Red 2", "Red2");
    	atomtype.addObject("Red 3", "Red3");
    	atomtype.addObject("Blue 1", "Blue1");
    	atomtype.addObject("Blue 2", "Blue2");
    	atomtype.addObject("Blue 3", "Blue3");
    	atomtype.addDefault("None", "None");
    	SmartDashboard.putData("Autonomous Type", atomtype);
    	
    	/*
    	shooterSpeed.addObject("100%", "1");
    	shooterSpeed.addObject("90%", ".9");
    	shooterSpeed.addDefault("80%", ".8");
    	shooterSpeed.addObject("70%", ".7");
    	shooterSpeed.addObject("60%", ".6");
    	shooterSpeed.addObject("50%", ".5");
    	shooterSpeed.addObject("40%", ".4");
    	shooterSpeed.addObject("30%", ".3");
    	shooterSpeed.addObject("20%", ".2");
    	shooterSpeed.addObject("10%", ".1");
    	SmartDashboard.putData("Shooter Speed %", shooterSpeed);
    	*/
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
		autoSelected = DriveType.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case "Red1":
			// Put custom auto code here
			break;
		case "Red2":
			// Put custom auto code here
			break;
		case "Red3":
			// Put custom auto code here
			break;
		case "Blue1":
			// Put custom auto code here
			break;
		case "Blue2":
			// Put custom auto code here
			break;
		case "Blue3":
			// Put custom auto code here
			break;
		case "None":
		default:
			// Put default auto code here
			break;
		}
		
		//double encoderDistanceReading = drivewheelencoder.get();
		//SmartDashboard.putNumber("encoder reading", encoderDistanceReading);
		
		//kFrontLeftChannel.set(-.4);
		//kRearLeftChannel.set(-.4); 
		//kFrontRightChannel.set(-.4);
		//kRearRightChannel.set(-.4);
		//if (encoderDistanceReading < -644) {
			//kFrontLeftChannel.stopMotor();
	//		kRearLeftChannel.stopMotor();
		//	kFrontRightChannel.stopMotor(); 
	//		kRearRightChannel.stopMotor();
		//	kFrontLeftChannel.reset();
//			kRearLeftChannel.reset();
	//		kFrontRightChannel.reset();
		//	kRearRightChannel.reset();
			
		//}
	}

	/**
	 * This function is called periodically during operator control 
	 */
	@Override
	public void teleopPeriodic() {
		robotDrive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			
			boolean rotateToAngle = false;
            
			// Use the joystick X axis for lateral movement, Y axis for forward
			// movement, and Z axis for rotation.
			// This sample does not use field-oriented drive, so the gyro input
			// is set to zero.
			
			switch (DriveType.getSelected()) {
			case "Field":
				//field drive
				if ( stick.getTrigger() ) {//trigger same as button 1
		              ahrs.zeroYaw();
		        }
				robotDrive.mecanumDrive_Cartesian(-stick.getX(), -stick.getY(), -stick.getZ(), ahrs.getAngle());
				break;
			case "Normal":
			default:
				//regular drive
				robotDrive.mecanumDrive_Cartesian(-stick.getX(), -stick.getY(), (-stick.getZ()*.5), 0);
				break;
			}
						
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
			
			

	          smartDashBoardDisplay();
	          
	          if (stick.getRawButton(1)) {
	        	  startshooter();
	          }
	          if (stick.getRawButton(2)) {
	        	  stopshooter();
	          }
	          if (stick.getRawButton(5)) {
	        	  startintake();
	          }
	          if (stick.getRawButton(6)) {
	        	  stopintake();
	          }
	          if (stick.getRawButton(11)) {
	        	  reverseintake();
	          }
	          if (stick.getRawButton(3)) {
	        	  startConveyor();
	          }
	          if (stick.getRawButton(4)) {
	        	  stopConveyor();
	          }
	          if (stick.getRawButton(6)) {
	        	//  reverseConveyor();
	          }
	          if (stick.getRawButton(7)) {
	        	//  startClimb();
	            //shooterlight.set(true);
	        	  //turnController.setSetpoint(0.0f);
	        	  turnController.setSetpoint(90.0f);
	        	  //turnController.setSetpoint(179.9f);
	        	 // turnController.setSetpoint(-90.0f);
	                rotateToAngle = true;
	                turnController.enable();
	                currentRotationRate = rotateToAngleRate;
	                robotDrive.mecanumDrive_Cartesian(0, 0, currentRotationRate, ahrs.getAngle());
					
	          }
	          if (stick.getRawButton(8)) {
	        	  double encoderDistanceReading = drivewheelencoder.get();
	      		  SmartDashboard.putNumber("encoder reading", encoderDistanceReading);
	      		    while (drivewheelencoder.get() > -644) {
	      		    	SmartDashboard.putNumber("encoder reading", drivewheelencoder.get());
	  	      		  
	      			  robotDrive.mecanumDrive_Cartesian(0, .5, 0, 0);
	      		  }
					
	        	  
	        	  //  stopClimb();
	            //shooterlight.set(false);
	          }
	          if (stick.getRawButton(9)) {
	        	gearlight.set(true);
	        	  
	        	  //  reverseClimb();
	          }
        	  if (stick.getRawButton(10)) {
        		  opengears();
        	  }
        	  if (stick.getRawButton(12)) {
        		  closegears();
        	  }
	          
        	  double encoderDistanceReading = drivewheelencoder.get(); 
      		  SmartDashboard.putNumber("encoder reading", encoderDistanceReading);
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