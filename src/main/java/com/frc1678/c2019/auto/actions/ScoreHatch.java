package com.frc1678.c2019.auto.actions;

import com.frc1678.c2019.statemachines.HatchIntakeStateMachine;
import com.frc1678.c2019.subsystems.HatchIntake;
import edu.wpi.first.wpilibj.Timer;

public class ScoreHatch implements Action {
    private static final HatchIntake mIntake = HatchIntake.getInstance();
    private static final double kPlaceTime = 0.75;

    private double mStartTime;

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntake.setState(HatchIntakeStateMachine.WantedAction.SCORE);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kPlaceTime;
    }

    @Override
    public void done() {
        mIntake.setState(HatchIntakeStateMachine.WantedAction.NONE);
    }
}
