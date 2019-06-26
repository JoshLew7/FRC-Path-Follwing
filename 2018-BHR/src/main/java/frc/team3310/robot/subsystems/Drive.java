package frc.team3310.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.utility.Loop;
import frc.team3310.utility.TalonSRXChecker;
import frc.team3310.utility.TalonSRXEncoder;
import frc.team3310.utility.TalonSRXFactory;
import frc.team3310.utility.control.Kinematics;
import frc.team3310.utility.control.Lookahead;
import frc.team3310.utility.control.Path;
import frc.team3310.utility.control.PathFollower;
import frc.team3310.utility.control.RobotState;
import frc.team3310.utility.math.RigidTransform2d;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Twist2d;

public class Drive extends Subsystem implements Loop {
	private static Drive instance;

	public static enum DriveControlMode {
		JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, ADAPTIVE_PURSUIT, VELOCITY_SETPOINT, CAMERA_TRACK
	};

	public static enum DriveSpeedShiftState {
		HI, LO
	};

	// One revolution of the wheel = Pi * D inches = 60/24 revs due to gears * 36/12
	// revs due mag encoder gear on ball shifter * 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (60.0 / 24.0) * 4096.0 / (5.8 * Math.PI);
	public static final double TRACK_WIDTH_INCHES = 24.56; // 26.937;
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private TalonSRXEncoder leftDrive1;
	private TalonSRX leftDrive2;
	private TalonSRX leftDrive3;

	private TalonSRXEncoder rightDrive1;
	private TalonSRX rightDrive2;
	private TalonSRX rightDrive3;
	
	private boolean mIsBrakeMode;
	
	private long periodMs = (long)(Constants.kLooperDt * 1000.0);
	
    protected Rotation2d mAngleAdjustment = Rotation2d.identity();

	private boolean isFinished;
	private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;
	
    private static final int kVelocityControlSlot = 2;

    private PathFollower mPathFollower;
    private Path mCurrentPath = null;
    private RobotState mRobotState = RobotState.getInstance();
	
	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
	
    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlMode state) {
        if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.ADAPTIVE_PURSUIT || state == DriveControlMode.CAMERA_TRACK) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlMode state) {
        if (state == DriveControlMode.MP_STRAIGHT ||
                state == DriveControlMode.MP_TURN ||
                state == DriveControlMode.HOLD) {
            return true;
        }
        return false;
    }

    private Drive() {
		try {
			leftDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);

			rightDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			
			leftDrive1.setSafetyEnabled(false);
			leftDrive1.setSensorPhase(false);  
			
			leftDrive1.setInverted(true);
			leftDrive2.setInverted(true);
			leftDrive3.setInverted(true);
			
			rightDrive1.setSafetyEnabled(false);
			rightDrive1.setSensorPhase(false);  
			
			rightDrive1.setInverted(false);			
			rightDrive2.setInverted(false);
			rightDrive3.setInverted(false);
							
			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);
			
			// m_drive = new BHRDifferentialDrive(leftDrive1, rightDrive1);
			// m_drive.setSafetyEnabled(false);

			gyroPigeon = new PigeonIMU(rightDrive2);
			
			// speedShift = new Solenoid(RobotMap.DRIVETRAIN_SPEEDSHIFT_PCM_ID);
									
			loadGains();
        	setBrakeMode(true);
	}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

    public synchronized void loadGains() {
        leftDrive1.setPIDFIZone(kVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);        
    }

    @Override
	public void initDefaultCommand() {
	}
	
	public synchronized double getGyroAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}
	
	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return  yprPigeon[2];
	}
	public boolean checkPitchAngle() {
		double pitchAngle = Math.abs(getGyroPitchAngle());
		if(pitchAngle > 10) {
			return true;
		}
		return false;
	}
	
	public synchronized void resetGyro() {
		gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
	}
	
    public synchronized Rotation2d getGyroAngle() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
    }

    public synchronized void setGyroAngle(Rotation2d adjustment) {
    	resetGyro();
        mAngleAdjustment = adjustment;
    }

	public synchronized void resetEncoders() {
		rightDrive1.setPosition(0);
		leftDrive1.setPosition(0);
	}
	
    public void zeroSensors() {
        resetEncoders();
        resetGyro();
    }

    public void calibrateGyro() {
		gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
	}
	
	public void endGyroCalibration() {
		if (isCalibrating == true) {
			isCalibrating = false;
		}
	}
	
	public void setGyroOffset(double offsetDeg) {
		gyroOffsetDeg = offsetDeg;
	}
	
    public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		}
		else {
			setControlMode(DriveControlMode.JOYSTICK);
		}
	}
    
    public synchronized void setControlMode(DriveControlMode controlMode) {
 		this.driveControlMode = controlMode;
 		if (controlMode == DriveControlMode.HOLD) {
			// mpStraightController.setPID(mpHoldPIDParams, kLowGearPositionControlSlot);
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.setPosition(0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}
    
    public synchronized DriveControlMode getControlMode() {
    	return driveControlMode;
    }
	
	@Override
	public void onStart(double timestamp) {
        synchronized (Drive.this) {

        }
	}

	@Override
	public void onStop(double timestamp) {
		
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Drive.this) {
			DriveControlMode currentControlMode = getControlMode();

			if (currentControlMode == DriveControlMode.JOYSTICK) {
				// driveWithJoystick();
			}
			else if (!isFinished()) {
				switch (currentControlMode) {			
					case MP_STRAIGHT :
						// setFinished(mpStraightController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case MP_TURN:
						// setFinished(mpTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case PID_TURN:
						// setFinished(pidTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case ADAPTIVE_PURSUIT:
	                    if (mPathFollower != null) {
	                        updatePathFollower(timestamp);
	                    }
	                    return;
					case CAMERA_TRACK:
	                    // updateCameraTrack();
	                    return;
	                default:
	                    System.out.println("Unknown drive control mode: " + currentControlMode);
	                    break;
                }
			}
			else {
				// hold in current state
			}
		}
	}
	
    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */


    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || driveControlMode != DriveControlMode.ADAPTIVE_PURSUIT) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));

            driveControlMode = DriveControlMode.ADAPTIVE_PURSUIT;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
            System.out.println("Oh NOOOO in velocity set point");
        }
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(driveControlMode)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double maxSetpoint = Constants.kDriveLowGearMaxSetpoint;
            final double scale = max_desired > maxSetpoint ? maxSetpoint / max_desired : 1.0;
            
            leftDrive1.setVelocityWorld(left_inches_per_sec * scale);
            rightDrive1.setVelocityWorld(right_inches_per_sec * scale);
//            double command = leftDrive1.convertEncoderWorldToTicks(left_inches_per_sec * scale) * 0.1;
//            System.out.println("vel Com u/s = " + command + ", vel com in/sec= " + left_inches_per_sec * scale + ", scale = " + scale + ", left pos in = " + getLeftPositionInches()  + ", right pos in = " + getRightPositionInches() + ", left vel in/sec = " + getLeftVelocityInchesPerSec() + ", left vel u/s = " + leftDrive1.getSelectedSensorVelocity(0));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftDrive1.set(ControlMode.Velocity, 0);
            rightDrive1.set(ControlMode.Velocity, 0);
        }
    }

    /**
     * Configures talons for velocity control
     */
    public void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(driveControlMode)) {
        	leftDrive1.enableVoltageCompensation(true);
        	leftDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
        	leftDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
        	leftDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

        	rightDrive1.enableVoltageCompensation(true);
        	rightDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
        	rightDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
			rightDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
			
        	System.out.println("configureTalonsForSpeedControl LO");
	    	leftDrive1.selectProfileSlot(kVelocityControlSlot, TalonSRXEncoder.PID_IDX);
        	leftDrive1.configNominalOutputForward(Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	    	leftDrive1.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        leftDrive1.configClosedloopRamp(Constants.kDriveLowGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
	        	    	
	    	rightDrive1.selectProfileSlot(kVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	       	rightDrive1.configNominalOutputForward(Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	     	rightDrive1.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	    	rightDrive1.configClosedloopRamp(Constants.kDriveLowGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
        	}
        }
    

    public synchronized boolean isDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode 1");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode 2, control mode = " + driveControlMode);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode 3. Control mode = " + driveControlMode);
            return false;
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            rightDrive1.setNeutralMode(NeutralMode.Brake);
            rightDrive2.setNeutralMode(NeutralMode.Brake);
            rightDrive3.setNeutralMode(NeutralMode.Brake);
            leftDrive1.setNeutralMode(NeutralMode.Brake);
            leftDrive2.setNeutralMode(NeutralMode.Brake);
            leftDrive3.setNeutralMode(NeutralMode.Brake);
        }
    }

	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
	
	public double getPeriodMs() {
		return periodMs;
	}
	
	public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    public double getRightPositionInches() {
    	return rightDrive1.getPositionWorld();
    }

    public double getLeftPositionInches() {
    	return leftDrive1.getPositionWorld();
    }

    public double getRightVelocityInchesPerSec() {
    	return rightDrive1.getVelocityWorld();
    }

    public double getLeftVelocityInchesPerSec() {
    	return leftDrive1.getVelocityWorld();
    }
    
    public double getAverageLeftCurrent() {
    	return (leftDrive1.getOutputCurrent() + leftDrive2.getOutputCurrent() + leftDrive3.getOutputCurrent()) / 3;
    }

    public double getAverageRightCurrent() {
    	return (rightDrive1.getOutputCurrent() + rightDrive2.getOutputCurrent() + rightDrive3.getOutputCurrent()) / 3;
    }

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Drive Right Position Inches", rightDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Left Position Inches", leftDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Right Velocity InPerSec", rightDrive1.getVelocityWorld());
				SmartDashboard.putNumber("Drive Left Velocity InPerSec", leftDrive1.getVelocityWorld());
				SmartDashboard.putNumber("Drive Left 1 Amps", leftDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left 2 Amps", leftDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left 3 Amps", leftDrive3.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left Average Amps", getAverageLeftCurrent());
				SmartDashboard.putNumber("Drive Right 1 Amps", rightDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 2 Amps", rightDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 3 Amps", rightDrive3.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right Average Amps", getAverageRightCurrent());
				SmartDashboard.putNumber("Yaw Angle Deg", getGyroAngleDeg());
				SmartDashboard.putNumber("Pitch Angle Deg", getGyroPitchAngle());

			}
			catch (Exception e) {
			}
		}
	}	
	
	public static Drive getInstance() {
		if(instance == null) {
			instance = new Drive();
		}
		return instance;
	}

    public boolean checkSystem() {

        boolean leftSide = TalonSRXChecker.CheckTalons(this,

                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {

                    {

                        add(new TalonSRXChecker.TalonSRXConfig("left_master", leftDrive1));

                        add(new TalonSRXChecker.TalonSRXConfig("left_slave", leftDrive2));

                        add(new TalonSRXChecker.TalonSRXConfig("left_slave1", leftDrive3));

                    }

                }, new TalonSRXChecker.CheckerConfig() {

                    {

                        mCurrentFloor = 2;

                        mRPMFloor = 1500;

                        mCurrentEpsilon = 2.0;

                        mRPMEpsilon = 250;

                        mRPMSupplier = () -> leftDrive1.getSelectedSensorVelocity(0);

                    }

                });

        boolean rightSide = TalonSRXChecker.CheckTalons(this,

                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {

                    {

                        add(new TalonSRXChecker.TalonSRXConfig("right_master", rightDrive1));

                        add(new TalonSRXChecker.TalonSRXConfig("right_slave", rightDrive2));

                        add(new TalonSRXChecker.TalonSRXConfig("right_slave1", rightDrive3));

                    }

                }, new TalonSRXChecker.CheckerConfig() {

                    {

                        mCurrentFloor = 2;

                        mRPMFloor = 1500;

                        mCurrentEpsilon = 2.0;

                        mRPMEpsilon = 250;

                        mRPMSupplier = () -> rightDrive1.getSelectedSensorVelocity(0);

                    }

                });

        return leftSide && rightSide;

    }

}