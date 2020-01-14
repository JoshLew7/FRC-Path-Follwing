/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.auto.actions;

import java.util.ArrayList;

import frc.team3310.robot.auto.AutoModeEndedException;
import frc.team3310.robot.auto.AutoRoutineBase;
import frc.team3310.utility.lib.physics.DriveCharacterization;

public class CharacterizeDrivebase extends AutoRoutineBase {
  ArrayList<DriveCharacterization.VelocityDataPoint> velocityData;
  ArrayList<DriveCharacterization.AccelerationDataPoint> accelerationData;
  private boolean reversed;
  private boolean turn;

  public CharacterizeDrivebase(boolean isReversed, boolean isTurning) {
    velocityData = new ArrayList<DriveCharacterization.VelocityDataPoint>();
    accelerationData = new ArrayList<DriveCharacterization.AccelerationDataPoint>();
    this.reversed = isReversed;
    this.turn = isTurning;

  }

  @Override
  protected void routine() throws AutoModeEndedException {

    runAction(new SeriesAction(new CollectVelocityData(velocityData, reversed, turn)));
    runAction(new WaitAction(10));
    runAction(new SeriesAction(new CollectAccelerationData(accelerationData, reversed, turn)));
    runAction(new WaitAction(10));
    runAction(new SeriesAction(new CalculateCharacterization(velocityData, accelerationData)));
  }
}
