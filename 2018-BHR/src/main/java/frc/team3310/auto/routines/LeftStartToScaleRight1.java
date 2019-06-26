package frc.team3310.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.team3310.auto.PathContainer;
import frc.team3310.auto.paths.Backup5;
import frc.team3310.auto.paths.Forward5;
import frc.team3310.auto.paths.LeftStartToScaleRightSave;
import frc.team3310.auto.paths.ScaleRightToSwitchRight;
import frc.team3310.robot.commands.DrivePathAdaptivePursuit;
import frc.team3310.robot.commands.DriveResetPoseFromPath;

/**
 *
 */
public class LeftStartToScaleRight1 extends CommandGroup {

    public LeftStartToScaleRight1() {
        
    	// Initialize everything at starting position
    	// addSequential(new ElevatorSetZero(0));
    	PathContainer path = new LeftStartToScaleRightSave();
        addSequential(new DriveResetPoseFromPath(path, true));

    	// Drive backwards to scale.  Start raising elevator during the path when "raiseElevator" marker is crossed
    	// addParallel(new RunAfterMarker("raiseElevator", 4.0, new ElevatorSetPositionMP(Elevator.SCALE_HIGH_POSITION_INCHES)));
    	addSequential(new DrivePathAdaptivePursuit(path));
    	addSequential(new WaitForChildren());
        // addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_REAR_EJECT_FAST_SPEED, 0.8));
    	
    	
    	// Drive forwards to switch.  Center on last cube.  
        PathContainer path2 = new ScaleRightToSwitchRight();
    	// addParallel(new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES));
        // addParallel(new IntakeCubeAndLiftAbortDrive(true));
        addSequential(new DriveResetPoseFromPath(path2, false));
    	addSequential(new DrivePathAdaptivePursuit(path2));
    	addSequential(new WaitForChildren());
    	
    	// Eject cube
    	PathContainer backup5 = new Backup5();
        addSequential(new DriveResetPoseFromPath(backup5, false));
    	addSequential(new DrivePathAdaptivePursuit(backup5));
    	
    	// addSequential(new ElevatorSetPositionPID(Elevator.SWITCH_POSITION_INCHES));

    	PathContainer forward5 = new Forward5();
        addSequential(new DriveResetPoseFromPath(forward5, false));
    	addSequential(new DrivePathAdaptivePursuit(forward5));

        // addSequential(new IntakeSetSpeedTimed(Intake.INTAKE_EJECT_SPEED, 0.6));
    	// addParallel(new ParallelDelay(0.5, new ElevatorSetPositionPID(Elevator.MIN_POSITION_INCHES)));
    	// addSequential(new DriveStraightMP(-25.0, Drive.MP_FAST_VELOCITY_INCHES_PER_SEC, true, false, 0));

    }
}
