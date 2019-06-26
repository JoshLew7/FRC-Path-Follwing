/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.utility.Looper;
import frc.team3310.utility.control.RobotState;
import frc.team3310.utility.control.RobotStateEstimator;
import frc.team3310.utility.math.RigidTransform2d;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;

  // Declare subsystems
  public static final Drive drive = Drive.getInstance();

  // Control looper
  public static final Looper controlLoop = new Looper();

  // Choosers
  private SendableChooser<OperationMode> operationModeChooser;
  private Command autonomousCommand;

  public static enum OperationMode {
    TEST, PRACTICE, COMPETITION
  };

  public static OperationMode operationMode = OperationMode.COMPETITION;

  // PDP
  public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

  // State
  private RobotState robotState = RobotState.getInstance();

  public Robot() {
    super(Constants.kLooperDt * 2);
    System.out.println("Main loop period = " + getPeriod());
  }

  public void zeroAllSensors() {
    drive.zeroSensors();
    robotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
    drive.zeroSensors();
  }

  @Override
  public void robotInit() {
    System.out.println("Main loop period = " + getPeriod());

    oi = OI.getInstance();

    controlLoop.register(drive);
    controlLoop.register(RobotStateEstimator.getInstance());

    // Update default at competition!!!
    operationModeChooser = new SendableChooser<OperationMode>();
    operationModeChooser.addOption("Practice", OperationMode.PRACTICE);
    operationModeChooser.setDefaultOption("Competition", OperationMode.COMPETITION);
    operationModeChooser.addOption("Test", OperationMode.TEST);
    SmartDashboard.putData("Operation Mode", operationModeChooser);

    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();

    zeroAllSensors();
  }

  // Called every loop for all modes
  public void robotPeriodic() {
    updateStatus();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    zeroAllSensors();
  }

  @Override
  public void autonomousInit() {
    controlLoop.start();
    zeroAllSensors();

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    operationMode = operationModeChooser.getSelected();

    controlLoop.start();
    drive.endGyroCalibration();
    zeroAllSensors();

    if (operationMode != OperationMode.COMPETITION) {
    } else {
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  public Alliance getAlliance() {
    return m_ds.getAlliance();
  }

  public double getMatchTime() {
    return m_ds.getMatchTime();
  }

  public void updateStatus() {
    drive.updateStatus(operationMode);
    robotState.updateStatus(operationMode);
  }
    
}
