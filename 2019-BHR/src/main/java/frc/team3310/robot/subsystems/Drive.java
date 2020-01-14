package frc.team3310.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.AutoModeSelector.DesiredMode;
import frc.team3310.robot.Constants;
import frc.team3310.robot.RobotMap;
import frc.team3310.robot.RobotState;
import frc.team3310.robot.loops.ILooper;
import frc.team3310.robot.loops.Loop;
import frc.team3310.robot.planners.DriveMotionPlanner;
import frc.team3310.utility.DriveSignal;
import frc.team3310.utility.ReflectingCSVWriter;
import frc.team3310.utility.lib.drivers.TalonSRXChecker;
import frc.team3310.utility.lib.drivers.TalonSRXEncoder;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class Drive extends Subsystem {
	private static Drive mInstance = new Drive();

	public static enum DriveControlMode {
		JOYSTICK, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING, OPEN_LOOP
	};

	// One revolution of the wheel = Pi * D inches = 4096 ticks
	// Track Width Flange to Flange Measurement
	private static final double DRIVE_ENCODER_PPR = 4096.;
	public static final double ENCODER_TICKS_TO_INCHES = DRIVE_ENCODER_PPR
			/ (Constants.kDriveWheelDiameterInches * Math.PI);
	public static final double TRACK_WIDTH_INCHES = 23.92;

	// private static final int kPositionControlSlot = 0;
	private static final int kVelocityControlSlot = 1;

	// private long periodMs = (long) (Constants.kLooperDt * 1000.0);

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();
	private TalonSRXEncoder leftDrive1;
	private TalonSRX leftDrive2;
	private TalonSRX leftDrive3;

	private TalonSRXEncoder rightDrive1;
	private TalonSRX rightDrive2;
	private TalonSRX rightDrive3;

	private DriveControlMode mDriveControlMode = DriveControlMode.JOYSTICK;

	// Pigeon Setup
	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private short[] xyzPigeon = new short[3];
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
	protected Rotation2d mAngleAdjustment = Rotation2d.identity();

	// Hardware states //Poofs
	private PeriodicIO mPeriodicIO;
	// private boolean mAutoShift;
	// private boolean mIsHighGear;
	private boolean mIsBrakeMode;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;
	private boolean isFinished;

	private final Loop mLoop = new Loop() {

		@Override
		public void onStart(double timestamp) {
			synchronized (Drive.this) {
				setOpenLoop(new DriveSignal(0.05, 0.05));
				setBrakeMode(false);
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (Drive.this) {
				DriveControlMode currentControlMode = getControlMode();

				if (currentControlMode == DriveControlMode.JOYSTICK) {
					// driveWithJoystick();
				} else if (!isFinished()) {
					// readPeriodicInputs();
					switch (currentControlMode) {
					case PATH_FOLLOWING:
						updatePathFollower();
						// writePeriodicOutputs();
						break;
					case OPEN_LOOP:
						// writePeriodicOutputs();
						break;
					case MANUAL:
						break;
					case VELOCITY_SETPOINT:
						break;
					default:
						System.out.println("Unknown drive control mode: " + currentControlMode);
						break;
					}
				} else {
					// hold in current state
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
			stopLogging();
		}
	};

	private void configureMaster(TalonSRX talon, boolean left) {
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
		final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
				100); // primary closed-loop, 100 ms timeout
		if (sensorPresent != ErrorCode.OK) {
			DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent,
					false);
		}
		talon.enableVoltageCompensation(true);
		talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
		talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
		talon.configNeutralDeadband(0.04, 0);
	}

	private Drive() {
		try {
			mPeriodicIO = new PeriodicIO();

			leftDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID,
					ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.CTRE_MagEncoder_Relative);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID,
					RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);

			rightDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID,
					ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);

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

			configureMaster(leftDrive1, true);
			configureMaster(rightDrive1, false);

			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);

			mMotionPlanner = new DriveMotionPlanner();

			gyroPigeon = new PigeonIMU(rightDrive2);
			gyroPigeon.configFactoryDefault();
			rightDrive2.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

			reloadGains();

			setOpenLoop(DriveSignal.NEUTRAL);

			mIsBrakeMode = true;
			setBrakeMode(false);
		} catch (Exception e) {
			System.err.println("An error occurred in the Drive constructor");
		}
	}

	public static Drive getInstance() {
		if (mInstance == null) {
			mInstance = new Drive();
		}
		return mInstance;
	}

	// Encoder and Gryo Setup
	private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	// private static double rpmToInchesPerSecond(double rpm) {
	// return rotationsToInches(rpm) / 60;
	// }

	// private static double inchesToRotations(double inches) {
	// return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	// }

	// private static double inchesPerSecondToRpm(double inches_per_second) {
	// return inchesToRotations(inches_per_second) * 60;
	// }

	private static double radiansPerSecondToTicksPer100ms(double rad_s) {
		return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	public double getLeftEncoderRotations() {
		return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
	}

	public double getRightEncoderRotations() {
		return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
	}

	public double getLeftEncoderDistance() {
		return rotationsToInches(getLeftEncoderRotations());
	}

	public double getRightEncoderDistance() {
		return rotationsToInches(getRightEncoderRotations());
	}

	public double getRightVelocityNativeUnits() {
		return mPeriodicIO.right_velocity_ticks_per_100ms;
	}

	public double getRightLinearVelocity() {
		return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
	}

	public double getLeftVelocityNativeUnits() {
		return mPeriodicIO.left_velocity_ticks_per_100ms;
	}

	public double getLeftLinearVelocity() {
		return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
	}

	public double getLinearVelocity() {
		return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
	}

	public double getAngularVelocity() {
		return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
	}

	public synchronized void resetEncoders() {
		rightDrive1.setSelectedSensorPosition(0, 0, 0);
		leftDrive1.setSelectedSensorPosition(0, 0, 0);
		mPeriodicIO = new PeriodicIO();
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

	public synchronized Rotation2d getGyroAngle() {
		return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
	}

	public synchronized void setGyroAngle(Rotation2d adjustment) {
		resetGyro();
		mAngleAdjustment = adjustment;
	}

	public synchronized double getGyroAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}

	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return yprPigeon[2];
	}

	public short getGyroXAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[0];
	}

	public short getGyroYAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[1];
	}

	public short getGyroZAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[2];
	}

	public synchronized Rotation2d getHeading() {
		return mPeriodicIO.gyro_heading;
	}

	public synchronized void setHeading(Rotation2d heading) {
		System.out.println("SET HEADING: " + heading.getDegrees());

		mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).inverse());
		System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

		mPeriodicIO.gyro_heading = heading;
	}

	public boolean checkPitchAngle() {
		double pitchAngle = Math.abs(getGyroPitchAngle());
		if (pitchAngle > 10) {
			return true;
		}
		return false;
	}

	public synchronized void resetGyro() {
		gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
		setHeading(Rotation2d.identity());

	}

	public void zeroSensors() {
		resetEncoders();
		resetGyro();
	}
	// End

	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveControlMode != DriveControlMode.OPEN_LOOP) {
			setBrakeMode(false);

			System.out.println("Switching to open loop");
			System.out.println(signal);
			mDriveControlMode = DriveControlMode.OPEN_LOOP;
			rightDrive1.configNeutralDeadband(0.04, 0);
			leftDrive1.configNeutralDeadband(0.04, 0);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	/**
	 * Configures talons for velocity control
	 */
	public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
		if (mDriveControlMode != DriveControlMode.PATH_FOLLOWING) {
			setBrakeMode(true);
			leftDrive1.selectProfileSlot(kVelocityControlSlot, 0);
			rightDrive1.selectProfileSlot(kVelocityControlSlot, 0);
			leftDrive1.configNeutralDeadband(0.0, 0);
			rightDrive1.configNeutralDeadband(0.0, 0);

			setControlMode(DriveControlMode.PATH_FOLLOWING);

		}

		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = feedforward.getLeft();
		mPeriodicIO.right_feedforward = feedforward.getRight();

	}

	public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);

			setControlMode(DriveControlMode.PATH_FOLLOWING);
		}
	}

	public void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}

	public boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory == true
				|| mDriveControlMode != DriveControlMode.PATH_FOLLOWING;
	}

	private void updatePathFollower() {
		if (mDriveControlMode == DriveControlMode.PATH_FOLLOWING) {
			final double now = Timer.getFPGATimestamp();

			DriveMotionPlanner.Output output = mMotionPlanner.update(now,
					RobotState.getInstance().getFieldToVehicle(now));

			// DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0,
			// demand.right_feedforward_voltage / 12.0);

			mPeriodicIO.error = mMotionPlanner.error();
			mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

			if (!mOverrideTrajectory) {
				setVelocity(
						new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
								radiansPerSecondToTicksPer100ms(output.right_velocity)),
						new DriveSignal(output.left_feedforward_voltage / 12.0,
								output.right_feedforward_voltage / 12.0));

				mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
				mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
			} else {
				setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
				mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
			}
		} else {
			DriverStation.reportError("Drive is not in path following state", false);
		}
	}

	public synchronized void reloadGains() {
		leftDrive1.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
		leftDrive1.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
		leftDrive1.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
		leftDrive1.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
		leftDrive1.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone,
				Constants.kLongCANTimeoutMs);

		rightDrive1.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
		rightDrive1.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone,
				Constants.kLongCANTimeoutMs);
	}

	public void writeToLog() {
	}

	public synchronized void readPeriodicInputs() {
		double prevLeftTicks = mPeriodicIO.left_position_ticks;
		double prevRightTicks = mPeriodicIO.right_position_ticks;
		mPeriodicIO.left_position_ticks = leftDrive1.getSelectedSensorPosition(0);
		mPeriodicIO.right_position_ticks = rightDrive1.getSelectedSensorPosition(0);
		mPeriodicIO.left_velocity_ticks_per_100ms = leftDrive1.getSelectedSensorVelocity(0);
		mPeriodicIO.right_velocity_ticks_per_100ms = rightDrive1.getSelectedSensorVelocity(0);
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).rotateBy(mGyroOffset);

		double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
		if (deltaLeftTicks > 0.0) {
			mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
		}

		double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
		if (deltaRightTicks > 0.0) {
			mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

		}

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}

		// System.out.println("control state: " + mDriveControlState + ", left: " +
		// mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
	}

	public synchronized void writePeriodicOutputs() {
		if (mDriveControlMode == DriveControlMode.OPEN_LOOP) {
			leftDrive1.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
			rightDrive1.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
		} else {
			leftDrive1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
			rightDrive1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
		}
	}
	// End

	// Drive
	public synchronized DriveControlMode getControlMode() {
		return mDriveControlMode;
	}

	public synchronized void setControlMode(DriveControlMode controlMode) {
		this.mDriveControlMode = controlMode;
		if (controlMode == DriveControlMode.HOLD) {
			// mpStraightController.setPID(mpHoldPIDParams, kPositionControlSlot); //Check
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.setPosition(0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}

	public synchronized void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		} else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, speed);
			leftDrive1.set(ControlMode.PercentOutput, speed);
		}
	}

	public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		} else {
			setControlMode(DriveControlMode.JOYSTICK);
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
	// End

	// public double getPeriodMs() {
	// return periodMs;
	// }

	public boolean checkSystem() {
		boolean leftSide = TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
			/**
			 *
			 */
			private static final long serialVersionUID = 8394970743380604366L;

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

		boolean rightSide = TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
			/**
			 *
			 */
			private static final long serialVersionUID = -7059620893865597689L;

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

	public synchronized void startLogging() {
		if (mCSVWriter == null) {
			mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
		}
	}

	public synchronized void stopLogging() {
		if (mCSVWriter != null) {
			mCSVWriter.flush();
			mCSVWriter = null;
		}
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

	@Override
	public void outputTelemetry(DesiredMode operationMode) {
		if (operationMode == DesiredMode.TEST) {
			try {
				SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
				SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
				SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
				SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
				SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
				SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

				SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
				SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
				SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());

			} catch (Exception e) {
			}
		} else if (operationMode == DesiredMode.COMPETITION) {

			if (getHeading() != null) {
				// SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
			}

		}
	}

	public static class PeriodicIO {
		// INPUTS
		public int left_position_ticks;
		public int right_position_ticks;
		public double left_distance;
		public double right_distance;
		public int left_velocity_ticks_per_100ms;
		public int right_velocity_ticks_per_100ms;
		public Rotation2d gyro_heading = Rotation2d.identity();
		public Pose2d error = Pose2d.identity();

		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
				Pose2dWithCurvature.identity());
	}
}
