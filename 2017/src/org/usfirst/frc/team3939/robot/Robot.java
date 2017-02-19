package org.usfirst.frc.team3939.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DriverStation;






/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	AHRS ahrs;
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	SendableChooser<String> DriveType = new SendableChooser();
    SendableChooser<String> MaxSpeed = new SendableChooser();
    SendableChooser<String> shooterSpeed = new SendableChooser();
    SendableChooser<String> HeightOffset = new SendableChooser();
    SendableChooser<String> atomtype = new SendableChooser();
    
	RobotDrive robotDrive;

	// Channels for the wheels
	CANTalon kFrontLeftChannel = new CANTalon(27); 		/* device IDs here (1 of 2) */
	CANTalon kRearLeftChannel = new CANTalon(21);
	CANTalon kFrontRightChannel = new CANTalon(28);
	CANTalon kRearRightChannel = new CANTalon(24);
	
	CANTalon kGearRight = new CANTalon(22);
	CANTalon kGearLeft = new CANTalon(29);
	
	double OpenPosition = .25; //gear open position
	double ClosedPosition = 0; //gear open position
	
	
	Talon ShooterMotor; 
	int ShooterPower = 0;

	Servo ShooterStop;
	
	// The channel on the driver station that the joystick is connected to
	final int kJoystickChannel = 0;
	
	Joystick stick = new Joystick(kJoystickChannel);
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		smartDashBoardBsetup();
		
		//ShooterStop = new Servo(1);  //ShooterStop
	//	ShooterStop.set(.9); // set start location

		ShooterMotor = new Talon(0); 
		
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
		
		try {
	          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
	          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
	          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
	          ahrs = new AHRS(SPI.Port.kMXP); 
	      } catch (RuntimeException ex ) {
	          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
	      }
		
		initTalonEncoders();
		
	}

	
	public void initTalonEncoders(){
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		int absolutePositionRight = kGearRight.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
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
		int absolutePositionLeft = kGearRight.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
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
        kGearLeft.setForwardSoftLimit(OpenPosition);
        kGearLeft.enableReverseSoftLimit(true);
        kGearLeft.setReverseSoftLimit(ClosedPosition);	}
	
	public void opengears() {
		kGearRight.set(.4); 
		kGearLeft.set(-.4); 
	}
	
	public void closegears() {
		kGearRight.set(-.4); 
		kGearLeft.set(.4); 
	}
	
	public void startshooter() {
		ShooterPower = 1; //set shooter power level
		ShooterMotor.set(ShooterPower); 
    	//Timer.delay(3.0);
    	ShooterStop.set(0.5); //Shooter Servo location
    	//need to start conveyor
    }
	
	public void stopshooter() {
		//need to stop conveyor
		ShooterStop.set(0.9); //Shooter Servo location
		//Timer.delay(3.0);
    	ShooterMotor.stopMotor();			
	}
	
	
	public void smartDashBoardBsetup() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		DriveType.addObject("2 Joysticks", "Tank");
    	DriveType.addDefault("1 Joystick", "Arcade");
    	SmartDashboard.putData("Drive Type", DriveType);
		
    	MaxSpeed.addDefault("100%", "1");
    	MaxSpeed.addObject("90%", ".9");
    	MaxSpeed.addObject("80%", ".8");
    	MaxSpeed.addObject("70%", ".7");
    	MaxSpeed.addObject("60%", ".6");
    	MaxSpeed.addObject("50%", ".5");
    	MaxSpeed.addObject("40%", ".4");
    	MaxSpeed.addObject("30%", ".3");
    	MaxSpeed.addObject("20%", ".2");
    	SmartDashboard.putData("Max Speed %", MaxSpeed);
    	
    	atomtype.addObject("Portcullis", "Portcullis");
    	atomtype.addObject("Drawbridge", "Drawbridge");
    	atomtype.addObject("Ramparts", "Ramparts");
    	atomtype.addObject("RockWall", "RockWall");
    	atomtype.addObject("LowBar", "LowBar");
    	atomtype.addObject("ChevaldeFrise", "ChevaldeFrise");
    	atomtype.addObject("Moat", "Moat");
    	atomtype.addObject("Sallyport", "Sallyport");
    	atomtype.addDefault("RoughTerrain", "RoughTerrain");
    	SmartDashboard.putData("Autonomous Type", atomtype);
    	    	
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
        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
		
        SmartDashboard.putNumber("Right Gear Encoder Output", kGearRight.get());
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
		autoSelected = chooser.getSelected();
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
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
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
			
			//regular drive
			//robotDrive.mecanumDrive_Cartesian(-stick.getX(), -stick.getY(), -stick.getZ(), 0);
			
			//field drive
			robotDrive.mecanumDrive_Cartesian(-stick.getX(), -stick.getY(), -stick.getZ(), ahrs.getAngle());

			
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
			
			boolean zero_yaw_pressed = stick.getTrigger();
	          if ( zero_yaw_pressed ) {
	              ahrs.zeroYaw();
	          }

	          smartDashBoardDisplay();
	          
	          if (stick.getRawButton(7)) {
	  	        //	  startClimb();
	        	  opengears();
	  	      }
	  	      if (stick.getRawButton(8)) {
	  	        //	  stopClimb();
	  	    	  closegears();
	  	      }
	          
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}
