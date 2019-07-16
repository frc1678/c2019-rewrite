package com.frc1678.c2019;

import com.frc1678.c2019.auto.AutoModeBase;
import com.frc1678.c2019.auto.creators.AutoModeCreator;
import com.frc1678.c2019.auto.creators.CharacterizeHighGearStraightCreator;
import com.frc1678.c2019.auto.creators.CrossAutoLineCreator;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum StartingPosition {
        LEFT,
        RIGHT,
    }

    enum DesiredMode {
        DO_NOTHING,
        CROSS_AUTO_LINE,
        CHARACTERIZE_DRIVE,
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private Optional<AutoModeCreator> mCreator = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Cross Auto Line", DesiredMode.CROSS_AUTO_LINE);
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Characterize Drive", DesiredMode.CHARACTERIZE_DRIVE);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Right", StartingPosition.RIGHT);
        mStartPositionChooser.addOption("Left", StartingPosition.LEFT);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if(mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", starting position->" + startingPosition.name());
            mCreator = getCreatorForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private Optional<AutoModeCreator> getCreatorForParams(DesiredMode mode, StartingPosition position) {
        boolean startOnLeft = StartingPosition.LEFT == position;
        switch (mode) {
            case CHARACTERIZE_DRIVE:
                return Optional.of(new CharacterizeHighGearStraightCreator());
            case CROSS_AUTO_LINE:
                return Optional.of(new CrossAutoLineCreator());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }


    public void reset() {
        mCreator = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mCreator.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(mCreator.get().getStateDependentAutoMode());
    }
}