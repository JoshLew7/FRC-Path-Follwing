package frc.team3310.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team3310.robot.Robot;

public class DriveWaitForMarker extends Command
{
	private String targetMarker;
	private double timeout;
	private double startTime;
	
	public DriveWaitForMarker(String targetMarker, double timeout) {
		this.targetMarker = targetMarker;
		this.timeout = timeout;
	}

	@Override
	protected void initialize() {
		setTimeout(timeout);
		startTime = System.currentTimeMillis();
		System.out.println("Start marker = " + targetMarker + ", time = " + System.currentTimeMillis());
	}

	@Override
	protected void execute() {		
	}

	@Override
	protected boolean isFinished() {
		if (isTimedOut()) {
			System.out.println("IsTimedOut marker = " + targetMarker);
			return true;
		}
		
		boolean markerPassed = Robot.drive.hasPassedMarker(targetMarker);
		if (markerPassed) {
			System.out.println("Got to marker = " + targetMarker + ", delta time = " + (System.currentTimeMillis() - startTime) + ", time = " + System.currentTimeMillis());
			return true;
		}
		
		return false;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}