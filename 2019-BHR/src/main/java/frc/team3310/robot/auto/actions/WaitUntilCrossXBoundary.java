package frc.team3310.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.team3310.robot.RobotState;

public class WaitUntilCrossXBoundary implements Action {

    public enum MovingXDirection {
        Negative, Positive
    };

    private double mXBoundary = 0;
    private MovingXDirection mMovingDirection;

    public WaitUntilCrossXBoundary(double x, MovingXDirection movingDirecton) {
        mXBoundary = x;
        mMovingDirection = movingDirecton;
    }

    @Override
    public boolean isFinished() {
        System.out.println("X Position"
                + RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x());

        if (mMovingDirection == MovingXDirection.Positive) {
            return RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation()
                    .x() > mXBoundary;
        } else {
            return RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation()
                    .x() < mXBoundary;
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        // System.out.println("Passed X Boundary");
    }

    @Override
    public void start() {

    }
}
