package frc.team3310.robot.auto;

import frc.team3310.utility.CrashTrackingRunnable;

/**
 * This class selects, runs, and (if necessary) stops a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
    private static AutoModeExecutor mInstance = null;

    private AutoRoutineBase mAutoMode = null;
    private Thread mThread = null;

    public AutoModeExecutor() {
    }

    public static AutoModeExecutor getInstance() {
        if (mInstance == null) {
            mInstance = new AutoModeExecutor();
        }

        return mInstance;
    }

    public void setAutoMode(AutoRoutineBase new_auto_mode) {
        mAutoMode = new_auto_mode;
        mAutoMode.activate(); // Activates the paths in mAutoMode
        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (mAutoMode != null) {
                    mAutoMode.run();
                }
            }
        });
    }

    public void start() {
        if (mThread != null) {
            mThread.start();
        }
    }

    public boolean isStarted() {
        return mAutoMode != null && mAutoMode.isActive() && mThread != null && mThread.isAlive();
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }

        mAutoMode = null;
    }

    public void stop() {
        if (mAutoMode != null) {
            mAutoMode.stop();
        }

        mThread = null;
    }

    public AutoRoutineBase getAutoMode() {
        return mAutoMode;
    }

    public boolean isInterrupted() {
        if (mAutoMode == null) {
            return false;
        }
        return mAutoMode.getIsInterrupted();
    }

    public void interrupt() {
        if (mAutoMode == null) {
            return;
        }
        mAutoMode.interrupt();
    }

    public void resume() {
        if (mAutoMode == null) {
            return;
        }
        mAutoMode.resume();
    }
}
