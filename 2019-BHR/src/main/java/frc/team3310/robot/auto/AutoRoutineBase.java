package frc.team3310.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team3310.robot.auto.AutoModeEndedException;
import frc.team3310.robot.auto.actions.Action;
import frc.team3310.robot.auto.actions.NoopAction;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto modes (which are routines that do actions).
 */
public abstract class AutoRoutineBase {
    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;

    private ArrayList<LazyLoadTrajectory> trajectories = new ArrayList<LazyLoadTrajectory>();

    public LazyLoadTrajectory registerTrajectory(LazyLoadTrajectory trajectory) {
        trajectories.add(trajectory);
        return trajectory;
    }

    public void activate() {
        for (LazyLoadTrajectory trajectory : trajectories) {
            trajectory.activate();
        }
    };

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void waitForDriverConfirm() throws AutoModeEndedException {
        if (!mIsInterrupted) {
            interrupt();
        }
        runAction(new NoopAction());
    }

    public void interrupt() {
        System.out.println("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mode resumed!");
        mIsInterrupted = false;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);

        // Wait for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();

    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}