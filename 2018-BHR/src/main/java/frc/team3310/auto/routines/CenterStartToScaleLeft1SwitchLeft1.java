package frc.team3310.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.team3310.auto.PathContainer;
import frc.team3310.auto.paths.CenterStartToScaleLeft;
import frc.team3310.auto.paths.ScaleLeftToSwitchLeft;
import frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import frc.team3310.robot.commands.DriveResetPoseFromPath;

/**
 *
 */
public class CenterStartToScaleLeft1SwitchLeft1 extends CommandGroup {

    public CenterStartToScaleLeft1SwitchLeft1() {
    	// addSequential(new ElevatorSetZero(0));
    	
    	PathContainer path = new CenterStartToScaleLeft();
        addSequential(new DriveResetPoseFromPath(path, true));

    	// addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new WaitForChildren());
        // addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_FAST_SPEED, 1.0));
        // addParallel(new ElevatorSetPositionMP(Elevator.ZERO_POSITION_INCHES));
        
        addSequential(new WaitCommand(0.8));
        // addParallel(new IntakeCubeAndLiftAbortDrive(false));
    	addSequential(new DrivePathAdaptivePursuit(new ScaleLeftToSwitchLeft()));
    	addSequential(new WaitForChildren());
    	
    	// addSequential(new ElevatorSetPositionMP(Elevator.SWITCH_POSITION_INCHES));
    	// addSequential(new DriveStraightMP(12.0, Drive.MP_MEDIUM_VELOCITY_INCHES_PER_SEC, true, false, 0));
        // addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 1.0));
     }
}
