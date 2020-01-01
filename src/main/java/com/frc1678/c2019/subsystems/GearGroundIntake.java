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
//import com.team254.lib.util.TimeDelayedBoolean;

import java.util.ArrayList;

public class GearGroundIntake extends Subsystem {
    // Intaking is positive
    public static double kIntakeVoltage = -12.0;
    public static double kHoldingVoltage = -4.0;
    public static double kOuttakeVoltage = 10.0;
    public static double kCarryingVoltage = -1.5;
    public static double kNotCarryingVoltage = 0;
    public static double kScoringVoltage = 12;
    public double PickUpTime; 

    private static GearGroundIntake mInstance;

    public enum WantedAction {
        NONE, INTAKE, OUTTAKE, HOLD, CARRY, NOT_CARRY, SCORE,
    }

    private enum State {
        IDLE, NONE, INTAKING, OUTTAKING, HOLDING, CARRYING, NOT_CARRYING, SCORING,
    }

    private State mState = State.NOT_CARRYING;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonSRX mMaster;
    private final Solenoid mDropSolenoid;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private GearGroundIntake() {
        mDropSolenoid = Constants.makeSolenoidForId(Constants.kCargoIntakePopoutSolenoidId);

        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeRollerId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
    }

    public synchronized static GearGroundIntake getInstance() {
        if (mInstance == null) {
            mInstance = new GearGroundIntake();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("Has Gear", mPeriodicIO.has_gear);
        SmartDashboard.putNumber("Gear Current", mPeriodicIO.current);
        SmartDashboard.putNumber("Gear Voltage", mPeriodicIO.voltage);
        SmartDashboard.putBoolean("Gear Solenoids", mPeriodicIO.drop_solenoid);

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
                mState = State.NOT_CARRYING;
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (GearGroundIntake.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.NOT_CARRYING;
                stopLogging();
            }
        });    
    }

    public synchronized boolean getIntakeOut() {
        return mDropSolenoid.get()
         && mPeriodicIO.drop_solenoid; // Cut latency on the hatch intake
    }

    public boolean hasGear() {
        if (mPeriodicIO.current > 200 && PickUpTime > 2){
                return true;
        } else {
            return false;
        }
        //get current and if current is higher than a certain number for a 
        //certain amount of time, you have a gear :)
    }

    public void runStateMachine() {
        switch (mState) {
        case IDLE:
            break;
        case NONE:
            mPeriodicIO.voltage = 0;
            break;
        case INTAKING:
            mPeriodicIO.drop_solenoid = true;
            mPeriodicIO.voltage = kIntakeVoltage;
            break;
        case OUTTAKING:
            mPeriodicIO.drop_solenoid = true;
            mPeriodicIO.voltage = kOuttakeVoltage;
            break;
        case HOLDING:
            mPeriodicIO.drop_solenoid = true;
            mPeriodicIO.voltage = kHoldingVoltage;
            break;
        case CARRYING:
            mPeriodicIO.drop_solenoid = false;
            mPeriodicIO.voltage = kCarryingVoltage;
            break;
        case NOT_CARRYING:
            mPeriodicIO.drop_solenoid = false;
            mPeriodicIO.voltage = kNotCarryingVoltage;
            break;
        case SCORING:
            mPeriodicIO.drop_solenoid = false;
            mPeriodicIO.voltage = kScoringVoltage;
            break;
        default:
            break;
        }
    }

    public void forceIntakeIn() {
        mPeriodicIO.drop_solenoid = false;
    }

    public synchronized void setOpenLoop(double percentage) {
        boolean mRunningManual = true;
        mPeriodicIO.voltage = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.voltage;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.NONE;
            break;
        case INTAKE:
            mState = State.INTAKING;
            break;
        case OUTTAKE:
            mState = State.OUTTAKING;
            break;
        case HOLD:
            mState = State.HOLDING;
            break;
        case CARRY:
            mState = State.CARRYING;
            break;
        case NOT_CARRY:
            mState = State.NOT_CARRYING;
            break;
        case SCORE:
            mState = State.SCORING;
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.current = mMaster.getOutputCurrent();
        mPeriodicIO.time = Timer.getFPGATimestamp();
        mPeriodicIO.has_gear = hasGear();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.voltage / 12.0);
        mDropSolenoid.set(mPeriodicIO.drop_solenoid);
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
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/GearGroundIntake-LOGS.csv", PeriodicIO.class);
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
        public double current;
        public double time;
        public boolean has_gear;

        // OUTPUTS
        public double voltage;
        public boolean drop_solenoid;
    }
}