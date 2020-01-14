package frc.team3310.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.team3310.robot.AutoModeSelector;
import frc.team3310.robot.AutoModeSelector.StartingPosition;
import frc.team3310.robot.RobotState;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.MirroredTrajectory;
import frc.team3310.utility.lib.trajectory.TimedView;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class DriveTrajectory implements Action {
    private final LazyLoadTrajectory mLazyLoadTrajectory;
    private AutoModeSelector mAutoModeSelector;
    private RobotState mRobotState;
    private final boolean mResetPose;
    private final boolean mResetXYPose;
    private Drive mDrive = Drive.getInstance();

    public DriveTrajectory(LazyLoadTrajectory lazyLoadTrajectory, boolean resetPose) {
        mLazyLoadTrajectory = lazyLoadTrajectory;
        mResetPose = resetPose;
        mResetXYPose = false;
    }

    public DriveTrajectory(LazyLoadTrajectory lazyLoadTrajectory, boolean resetPose, boolean resetXYPose) {
        mLazyLoadTrajectory = lazyLoadTrajectory;
        mResetPose = resetPose;
        mResetXYPose = resetXYPose;
    }

    @Override
    public boolean isFinished() {
        if (Drive.getInstance().isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        MirroredTrajectory mirroredTrajectory = mLazyLoadTrajectory.getTrajectory();
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory = null;
        if (mAutoModeSelector.getStartingPosition() == StartingPosition.RIGHT) {
            mTrajectory = new TrajectoryIterator<TimedState<Pose2dWithCurvature>>(
                    new TimedView<>(mirroredTrajectory.right));
        } else {
            mTrajectory = new TrajectoryIterator<TimedState<Pose2dWithCurvature>>(
                    new TimedView<>(mirroredTrajectory.left));
        }
        System.out.println("Starting trajectory on " + mAutoModeSelector.getStartingPosition() + " side! (length="
                + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        } else if (mResetXYPose) {
            Pose2d resetPose = new Pose2d(mTrajectory.getState().state().getPose().getTranslation(),
                    mRobotState.getFieldToVehicle(Timer.getFPGATimestamp()).getRotation());
            mRobotState.reset(Timer.getFPGATimestamp(), resetPose);
        }

        mDrive.setTrajectory(mTrajectory);
    }
}
