package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.auto.PathContainer;
import frc.team3310.robot.Robot;
import frc.team3310.utility.control.RobotState;
import frc.team3310.utility.math.RigidTransform2d;

public class DriveResetPoseFromPath extends Command
{
    protected PathContainer pathContainer;
    protected boolean usePathRotation;
    protected RigidTransform2d transform;

    public DriveResetPoseFromPath(PathContainer pathContainer, boolean usePathRotation) {
        this.pathContainer = pathContainer;
        this.transform = null;
        this.usePathRotation = usePathRotation;
		requires(Robot.drive);
	}

    public DriveResetPoseFromPath(RigidTransform2d transform, boolean usePathRotation) {
        this.pathContainer = null;
        this.transform = transform;
        this.usePathRotation = usePathRotation;
		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
        RigidTransform2d startPose = (transform == null) ? pathContainer.getStartPose() : transform;
        
        if (usePathRotation) {
        	Robot.drive.setGyroAngle(startPose.getRotation());
        }
        else {
        	startPose.setRotation(RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
        }
        
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        
        if (usePathRotation) {
        	Robot.drive.setGyroAngle(startPose.getRotation());
        }
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}