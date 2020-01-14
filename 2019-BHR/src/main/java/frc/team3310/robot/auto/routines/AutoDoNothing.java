package frc.team3310.robot.auto.routines;

import frc.team3310.robot.auto.AutoModeEndedException;
import frc.team3310.robot.auto.AutoRoutineBase;

public class AutoDoNothing extends AutoRoutineBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }
}
