package frc.team3310.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.auto.AutoRoutineBase;
import frc.team3310.robot.auto.actions.CharacterizeDrivebase;
import frc.team3310.robot.auto.routines.AutoCargoElimination;
import frc.team3310.robot.auto.routines.AutoCargoQualification;
import frc.team3310.robot.auto.routines.AutoDoNothing;
import frc.team3310.robot.auto.routines.AutoRocketElimination;
import frc.team3310.robot.auto.routines.AutoRocketQualification;
import frc.team3310.robot.auto.routines.AutoTest;
import frc.team3310.robot.auto.routines.AutoTestControlFlow;

public class AutoModeSelector {

    public enum StartingPosition {
        LEFT, RIGHT, CENTER
    }

    public enum DesiredMode {
        TEST, PRACTICE, COMPETITION
    }

    enum AutoChooser {
        DO_NOTHING, ROCKET, ROCKET_ELIM, CARGO, CARGO_ELIM, TEST, TEST_CONTROL_FLOW, DRIVE_CHARACTERIZATION_STRAIGHT,
        DRIVE_CHARACTERIZATION_TURN
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;
    private AutoChooser mCachedAutoSelected = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;
    private SendableChooser<AutoChooser> mAutoChooser;

    private Optional<AutoRoutineBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<StartingPosition>();
        mStartPositionChooser.setDefaultOption("Left", StartingPosition.LEFT);
        mStartPositionChooser.addOption("Right", StartingPosition.RIGHT);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<DesiredMode>();
        mModeChooser.setDefaultOption("Competition Mode", DesiredMode.COMPETITION);
        mModeChooser.addOption("Test Mode", DesiredMode.TEST);
        mModeChooser.addOption("Practice Mode", DesiredMode.PRACTICE);

        SmartDashboard.putData("Desired mode", mModeChooser);

        mAutoChooser = new SendableChooser<AutoChooser>();

        // No Default because it is reset in DisableInit
        mAutoChooser.addOption("Do Nothing", AutoChooser.DO_NOTHING);

        mAutoChooser.addOption("Rocket Front/Back", AutoChooser.ROCKET);
        mAutoChooser.addOption("Rocket Double Back", AutoChooser.ROCKET_ELIM);

        mAutoChooser.addOption("Cargo Double Front", AutoChooser.CARGO);
        mAutoChooser.addOption("Cargo Double Side", AutoChooser.CARGO_ELIM);

        mAutoChooser.addOption("Test", AutoChooser.TEST);
        mAutoChooser.addOption("Test control flow", AutoChooser.TEST_CONTROL_FLOW);
        mAutoChooser.addOption("Drive Characterization Straight", AutoChooser.DRIVE_CHARACTERIZATION_STRAIGHT);
        mAutoChooser.addOption("Drive Characterization Turn", AutoChooser.DRIVE_CHARACTERIZATION_TURN);

        SmartDashboard.putData("Autonomous", mAutoChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        AutoChooser autoSelected = mAutoChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition
                || autoSelected != mCachedAutoSelected) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + "selected auto ->" + autoSelected.name() + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, autoSelected, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
        mCachedAutoSelected = autoSelected;
    }

    private Optional<AutoRoutineBase> getAutoModeForParams(DesiredMode mode, AutoChooser auto,
            StartingPosition position) {
        switch (auto) {
        case DO_NOTHING:
            return Optional.of(new AutoDoNothing());
        case ROCKET:
            return Optional.of(new AutoRocketQualification());
        case ROCKET_ELIM:
            return Optional.of(new AutoRocketElimination());
        case CARGO:
            return Optional.of(new AutoCargoQualification());
        case CARGO_ELIM:
            return Optional.of(new AutoCargoElimination());
        case TEST:
            return Optional.of(new AutoTest());
        case TEST_CONTROL_FLOW:
            return Optional.of(new AutoTestControlFlow());
        case DRIVE_CHARACTERIZATION_STRAIGHT:
            return Optional.of(new CharacterizeDrivebase(false, false));
        case DRIVE_CHARACTERIZATION_TURN:
            return Optional.of(new CharacterizeDrivebase(false, true));
        default:
            break;
        }

        System.err.println("No valid auto mode found for  " + auto);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
        SmartDashboard.putString("AutoSlected", mCachedAutoSelected.name());
    }

    public Optional<AutoRoutineBase> getAutoMode() {
        return mAutoMode;
    }

    public StartingPosition getStartingPosition() {
        return mStartPositionChooser.getSelected();
    }

    public DesiredMode getDesiredMode() {
        return mModeChooser.getSelected();
    }

}
