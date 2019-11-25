package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.loops.ILooper;
import com.frc1678.c2019.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.util.TimeDelayedBoolean;

import java.util.ArrayList;

public class CargoIntake extends Subsystem {
    // Intaking is positive
    public static double kIntakeVoltage = -12.0;
    public static double kHoldingVoltage = -4.0;
    public static double kOuttakeVoltage = 10.0;

    private static CargoIntake mInstance;

    public enum WantedAction {
        NONE, INTAKE, OUTTAKE,
    }

    private enum State {
        INTAKING, OUTTAKING, HOLDING,
    }

    private State mState = State.HOLDING;

    private TimeDelayedBoolean mLastSeenCargo = new TimeDelayedBoolean();
    private boolean mDebouncedCargo = false;

    private boolean mRunningManual = false;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private CarriageCanifier mCanifier = CarriageCanifier.getInstance();

    private final TalonSRX mMaster;
    private final Solenoid mPopoutSolenoid;

    private CargoIntake() {
        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kCargoIntakePopoutSolenoidId);

        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeRollerId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
    }

    public synchronized static CargoIntake getInstance() {
        if (mInstance == null) {
            mInstance = new CargoIntake();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("CargoProxy", mPeriodicIO.has_cargo);
        SmartDashboard.putNumber("MotorSetpoint", mPeriodicIO.demand);
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mRunningManual = false;
                mState = State.HOLDING;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (CargoIntake.this) {
                    if (mRunningManual) {
                        runStateMachine(false);
                        return;
                    } else {
                        runStateMachine(true);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mRunningManual = false;
                mState = State.HOLDING;
            }
        });
    }

    public synchronized boolean getIntakeOut() {
        return mPopoutSolenoid.get()
         && mPeriodicIO.pop_out_solenoid; // Cut latency on the hatch intake
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case INTAKING:
            if (modifyOutputs) {
                mPeriodicIO.demand = kIntakeVoltage;
                mPeriodicIO.pop_out_solenoid = true;
            }
            if (hasCargo()) {
                mState = State.HOLDING;
            }
            break;
        case OUTTAKING:
            if (modifyOutputs) {
                mPeriodicIO.demand = kOuttakeVoltage;
                mPeriodicIO.pop_out_solenoid = true;
            } else if (hasCargo()) {
                mState = State.HOLDING;
            }
            break;
        case HOLDING:
            if (modifyOutputs) {
                mPeriodicIO.demand = hasCargo() ? kHoldingVoltage : 0.0;
                if (hasCargo()) {
                    mPeriodicIO.pop_out_solenoid = false;
                }
            }
            break;
        default:
            System.out.println("Fell through on Cargo Intake states!");
        }
    }

    public void forceIntakeIn() {
        mPeriodicIO.pop_out_solenoid = false;
    }

    public boolean hasCargo() {
        return mDebouncedCargo;
    }

    public synchronized void setOpenLoop(double percentage) {
        mRunningManual = true;
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public void setState(WantedAction wanted_state) {
        mRunningManual = false;
        switch (wanted_state) {
        case NONE:
            mState = State.HOLDING;
            break;
        case INTAKE:
            mState = State.INTAKING;
            break;
        case OUTTAKE:
            mState = State.OUTTAKING;
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mDebouncedCargo = mLastSeenCargo.update(mCanifier.getCargoProxy(), 0.1);
        mPeriodicIO.has_cargo = mDebouncedCargo;
        mPeriodicIO.cargo_proxy = mCanifier.getCargoProxy();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        if (Wrist.getInstance().getWantsPassThrough()) {
            forceIntakeIn();
        }
        mPopoutSolenoid.set(mPeriodicIO.pop_out_solenoid);
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
            {
                add(new TalonSRXChecker.TalonSRXConfig("cargo intake", mMaster));
            }
        }, new TalonSRXChecker.CheckerConfig() {
            {
                mCurrentFloor = 2;
                mCurrentEpsilon = 2.0;
                mRPMSupplier = null;
            }
        });
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean has_cargo;
        public boolean cargo_proxy;

        // OUTPUTS
        public double demand;
        public boolean pop_out_solenoid;
    }
}
