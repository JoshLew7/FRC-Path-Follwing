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

/**
 * Add your docs here.
 */
public class AutoRocketQualification extends AutoRoutineBase {
        DriveTrajectory first_path;
        DriveTrajectory second_path;
        DriveTrajectory third_path;
        DriveTrajectory fourth_path;

        public AutoRocketQualification() {

                first_path = new DriveTrajectory(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketFront), true);

                second_path = new DriveTrajectory(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontToTurn1A), false);

                third_path = new DriveTrajectory(
                                registerTrajectory(
                                                TrajectoryGenerator.getInstance().getTrajectorySet().turn1AToLoading),
                                false);

                fourth_path = new DriveTrajectory(registerTrajectory(
                                TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack), false);
        }

        @Override
        protected void routine() throws AutoModeEndedException {
                runAction(new SeriesAction(first_path));

        }
}
