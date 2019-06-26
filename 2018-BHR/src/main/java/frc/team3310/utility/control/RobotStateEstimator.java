package frc.team3310.utility.control;

import frc.team3310.robot.subsystems.Drive;
import frc.team3310.utility.Loop;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Twist2d;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = drive_.getLeftPositionInches();
        right_encoder_prev_distance_ = drive_.getRightPositionInches();
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        final double left_distance = drive_.getLeftPositionInches();
        final double right_distance = drive_.getRightPositionInches();
        final Rotation2d gyro_angle = drive_.getGyroAngle();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                drive_.getRightVelocityInchesPerSec());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }
    
    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
