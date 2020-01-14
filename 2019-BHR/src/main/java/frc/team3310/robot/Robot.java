/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.AutoModeSelector.DesiredMode;
import frc.team3310.robot.auto.AutoModeExecutor;
import frc.team3310.robot.auto.AutoRoutineBase;
import frc.team3310.robot.loops.Looper;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.robot.subsystems.Drive.DriveControlMode;
import frc.team3310.robot.subsystems.RobotStateEstimator;
import frc.team3310.utility.CrashTracker;
import frc.team3310.utility.lib.geometry.Pose2d;

public class Robot extends TimedRobot {
	// public static OI oi;

	// Control looper
	private Looper mEnabledLooper = new Looper();
	private Looper mDisabledLooper = new Looper();

	private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
	private AutoModeExecutor mAutoModeExecutor;
	private DesiredMode mOperationMode;

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(RobotStateEstimator.getInstance(), Drive.getInstance()));

	// Declare subsystems
	private Drive mDrive = Drive.getInstance();
	private RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	public void zeroAllSensors() {
		mDrive.zeroSensors();
	}

	// Called at the start of connection
	@Override
	public void robotInit() {
		// oi = OI.getInstance();
		try {
			// UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
			// MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0",
			// Constants.kCameraStreamPort);
			// cameraServer.setSource(camera);

			CrashTracker.logRobotInit();

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mTrajectoryGenerator.generateTrajectories();
			mAutoModeSelector.updateModeCreator();

			zeroAllSensors();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called every loop for all modes
	public void robotPeriodic() {
		// outputToSmartDashboard();
	}

	// Called once when is disabled
	@Override
	public void disabledInit() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			// mInfrastructure.setIsDuringAuto(true);
			mDrive.zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			// Reset all auto mode state.
			mAutoModeSelector.reset();
			mAutoModeSelector.updateModeCreator();
			mAutoModeExecutor = new AutoModeExecutor();

			mDisabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly when the robot is disabled
	@Override
	public void disabledPeriodic() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			mOperationMode = mAutoModeSelector.getDesiredMode();
			outputToSmartDashboard();
			mAutoModeSelector.updateModeCreator();

			Optional<AutoRoutineBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
				System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}
			System.gc();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called once at the start of auto
	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

		try {
			CrashTracker.logAutoInit();
			mDisabledLooper.stop();

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			mDrive.zeroSensors();

			mAutoModeExecutor.start();

			mEnabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly through auton
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

		outputToSmartDashboard();
		try {

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	// Called once at the start of teleOp
	@Override
	public void teleopInit() {
		SmartDashboard.putString("Match Cycle", "TELEOP");

		try {
			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mDrive.setControlMode(DriveControlMode.JOYSTICK);

			mDrive.endGyroCalibration();

			if (mOperationMode == DesiredMode.COMPETITION) {
				// Robot.elevator.setJoystickPID();
			}

			if (mOperationMode != DesiredMode.COMPETITION) {
				// Scheduler.getInstance().add(new ElevatorAutoZeroSensor());
			}

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			mEnabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly through teleOp
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Match Cycle", "TELEOP");

		try {

			outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public void outputToSmartDashboard() {
		mDrive.outputTelemetry(mOperationMode);
		mRobotStateEstimator.outputTelemetry(mOperationMode);
	}
}
