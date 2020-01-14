/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.auto.routines;

import frc.team3310.robot.auto.AutoModeEndedException;
import frc.team3310.robot.auto.AutoRoutineBase;
import frc.team3310.robot.auto.actions.DriveTrajectory;
import frc.team3310.robot.auto.actions.SeriesAction;
import frc.team3310.robot.paths.TrajectoryGenerator;

public class AutoTest extends AutoRoutineBase {
  DriveTrajectory first_path;
  DriveTrajectory second_path;
  DriveTrajectory third_path;

  public AutoTest() {

    first_path = new DriveTrajectory(
        registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveStraight), true);

  }

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new SeriesAction(first_path));

  }
}
