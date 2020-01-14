package frc.team3310.robot.auto.actions;

import java.util.List;

import frc.team3310.utility.lib.physics.DriveCharacterization;

public class CalculateCharacterization implements Action {

    private List<DriveCharacterization.VelocityDataPoint> velocityData;
    private List<DriveCharacterization.AccelerationDataPoint> accelerationData;

    public CalculateCharacterization(List<DriveCharacterization.VelocityDataPoint> velocityData,
            List<DriveCharacterization.AccelerationDataPoint> accelerationData) {
        this.velocityData = velocityData;
        this.accelerationData = accelerationData;
    }

    @Override
    public void start() {
        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization
                .characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}
