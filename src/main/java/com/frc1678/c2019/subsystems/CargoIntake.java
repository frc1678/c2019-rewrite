package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.loops.ILooper;
import com.frc1678.c2019.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.util.ReflectingCSVWriter;
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

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private CargoIntake() {
        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kCargoIntakePopoutSolenoidId);

        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeRollerId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mMaster.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
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
        SmartDashboard.putNumber("Cargo Current", mPeriodicIO.current);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
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
                // startLogging();
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
                stopLogging();
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
            if (hasCargo()) {
                mPeriodicIO.demand = kHoldingVoltage;
                mPeriodicIO.pop_out_solenoid = false;
                mState = State.HOLDING;
                break;
            }
            
            if (modifyOutputs) {
                mPeriodicIO.demand = kIntakeVoltage;
                mPeriodicIO.pop_out_solenoid = true;
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
        mPeriodicIO.current = mMaster.getOutputCurrent();
        mDebouncedCargo = mLastSeenCargo.update(mCanifier.getCargoProxy(), 0.1);
        mPeriodicIO.has_cargo = mDebouncedCargo;
        mPeriodicIO.cargo_proxy = mCanifier.getCargoProxy();
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
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
        return TalonSRXChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
            private static final long serialVersionUID = 8343060678848936021L;
            {
                add(new MotorChecker.MotorConfig<>("cargo intake", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mCurrentFloor = 2;
                mCurrentEpsilon = 2.0;
                mRPMSupplier = null;
            }
        });
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CARGOINTAKE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;
        public boolean has_cargo;
        public boolean cargo_proxy;

        // OUTPUTS
        public double demand;
        public boolean pop_out_solenoid;
    }
}