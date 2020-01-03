// Arthur's FIRST robot program! :D
package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.loops.ILooper;
import com.frc1678.c2019.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Percentage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.drivers.TalonSRXChecker;

//The FUN Stuff!
/*
public class GroundGearIntake extends Subsystem {

    public enum State {
        NONE, GROUND_INTAKING, INTAKING, HOLDING, SCORING, OUTTAKING
    }
    public enum WantedAction {
        IDLE, INTAKE, SCORE, OUTTAKE
    }
    State mState = State.NONE;

    private static GroundGearIntake mInstance;

    public static GroundGearIntake getInstance() {
        if (mInstance == null) {
            mInstance = new GroundGearIntake();
        }
    }
    private GroundGearIntake(){
        
    }
    private void setState(State newState){
        Boolean stateChanged = false;
        
        if(mState != newState){
            stateChanged = true;
        }

        mState = newState;
    }
    private void undefined(){}

    }
*/
public class GroundGearIntake extends Subsystem {
    // The official skeleton
    // Constants, make sure that you have public static before each of them
    public static double kIntakeVoltage = -12.0;
    public static double kHoldingVoltage = -4.0;
    public static double kOuttakeVoltage = 10.0;
    public static double kCarryingVoltage = 10.0;

    private static GroundGearIntake mInstance;

    public enum WantedAction {
        NONE, INTAKE, SCORE, OUTTAKE, DROP, RISE, STOP_DROPPING_BALLS, START_DROPPING_BALLS
    }

    private enum State {
        NONE, IDLE, GROUND_INTAKING, INTAKING, HOLDING, CARRYING, SCORING, OUTTAKING, PICKING_UP, DROP_BALL_WITH_GEAR, DROP_BALL_WITHOUT_GEAR
    }

    private State mState = State.NONE;

    // Any private variables you may need

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Any motor, solenoid, sensor you need. These will be referred to as
    // actuators(convert signal into energy) and sensors
    private final TalonSRX mMaster;
    private final Solenoid mGearSolenoid;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private GroundGearIntake() {
        // Set each actuator to their ID's
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeRollerId);
        mGearSolenoid = Constants.makeSolenoidForId(Constants.kCargoIntakePopoutSolenoidId);
    }

    public synchronized static GroundGearIntake getInstance() {
        if (mInstance == null) {
            mInstance = new GroundGearIntake();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        // Anything you want displayed on the SmartDashboard

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
                mState = State.NONE;
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (GroundGearIntake.this) {
                    // What happens while the robot is on. This is usually a state machine
                    // Ask Mohammed Here about how this works
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                // mRunningManual = false;
                mState = State.NONE;
                stopLogging();
            }
        });
    }

    public void runStateMachine() {
        // All states: NONE, GROUND_INTAKING, INTAKING, HOLDING, SCORING, OUTTAKING
        switch (mState) {
        case NONE:
            mPeriodicIO.gear_solenoid = false;
            mPeriodicIO.demand = 0;
        case GROUND_INTAKING:
            mPeriodicIO.gear_solenoid = true;
            mPeriodicIO.demand = kIntakeVoltage;
        case INTAKING:
            mPeriodicIO.gear_solenoid = true;
            mPeriodicIO.demand = kIntakeVoltage;
        case HOLDING:
            mPeriodicIO.gear_solenoid = true;
            mPeriodicIO.demand = kHoldingVoltage;
        case SCORING:
            mPeriodicIO.gear_solenoid = false;
            mPeriodicIO.demand = kOuttakeVoltage;
        case OUTTAKING:
            mPeriodicIO.gear_solenoid = true;
            mPeriodicIO.demand = kOuttakeVoltage;
        case CARRYING:
            mPeriodicIO.gear_solenoid = false;
            mPeriodicIO.demand = kCarryingVoltage;
        default:
            System.out.println("Fell through on Ground Gear Intake states!");
        }
    }
    public synchronized void setOpenLoop(double Percent){
        boolean mRunningManual = true;
        mPeriodicIO.demand = Percent;
    }
    public void setState(WantedAction wanted_state) {
        //possible states:         NONE, GROUND_INTAKING, INTAKING, HOLDING, SCORING, OUTTAKING
        switch (wanted_state) {
            case DROP:
            mState = State.INTAKING;
            break;
          case RISE:
            if (mState == State.INTAKING) {
                mState = State.IDLE;
            }
            break;
          case SCORE:
            if (mState == State.CARRYING || mState == State.IDLE) {
                mState = State.SCORING;
            }
            break;
          case NONE:
            if (mState == State.SCORING) {
                mState = State.IDLE;
            }
            break;
          case OUTTAKE:
            mState = State.OUTTAKING;
            break;
          case START_DROPPING_BALLS:
            if (mState == State.PICKING_UP || mState == State.CARRYING) {
              mState = State.DROP_BALL_WITH_GEAR;
            }
            if (mState == State.INTAKING || mState == State.IDLE) {
              mState = State.DROP_BALL_WITHOUT_GEAR;
            }
            break;
          case STOP_DROPPING_BALLS:
            if (mState == State.DROP_BALL_WITHOUT_GEAR) {
              mState = State.IDLE;
            }
            if (mState == State.DROP_BALL_WITH_GEAR) {
              mState = State.CARRYING;
            }
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // Update anything you want to read from sensors or actuators, these are usually
        // in your inputs under periodicIO
        mPeriodicIO.current = mMaster.getOutputCurrent();
        //mPeriodicIO.gear_solenoid = mGearSolenoid.getOutputCurrent();
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
            mGearSolenoid.set(mPeriodicIO.gear_solenoid);
        // Cases for each state and what the actuators should be at those states
        // Use .set on each of your actuators to whatever output you have been setting
        // from periodicIO. This is also a good place to add limits to your code.
    }

    @Override
    public boolean checkSystem() {
        // Use CheckMotor function on any motor you have that uses PID, else just return
        // true in this block
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>(
                    "/home/lvuser//GroundGearIntake/-LOGS.csv", PeriodicIO.class);
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
        // OUTPUTS
        public double demand;
        public boolean gear_solenoid;
        // public boolean solenoidCurrent;
    }
}