// First FRC Code, Version 0.1
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
import java.util.Scanner;
/* 
1. Create the motor for gear intake?
    - Find out if there is a link to the 2017 competition plan/code [Done]
    - 
2. Program for if gear is in intake
    - 
3. Program for if gear isn't in intake
    - 
4. Program for idle code
    - 
*/
public class GroundGearIntake extends Subsystem {
    // Constants, make sure that you have public static before each of them
    public static double kVoltIntake = -7.0;
    public static double kVoltOut = -3.0;
    public static double kVoltHold = -4.0;
    public static double kVoltCarry = -5.0;

    private static GroundGearIntake mInstance;


    public enum WantedAction {
        // Insert the action
        NONE, SCORE, DROP, RISE, INTAKE, OUTTAKE, START_BALLDROP, STOP_BALLDROP
    }

    private enum State {
        // Different types of states
        IDLE, INTAKE, PICKUP, CARRY, SCORING, OUTTAKE, BALLDROP_GEAR, BALLDROP_NOGEAR
    }

    private State mState = State.IDLE;

    // Any private variables you may need
    

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Any motor, solenoid, sensor you need. These will be referred to as actuators(convert signal into energy) and sensors
    private final TalonSRX mMaster;
    private final Solenoid mGearIntSolenoid;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private GroundGearIntake() {
     // Set each actuator to their ID's 
    
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeRollerId);
        mGearIntSolenoid = Constants.makeSolenoidForId(Constants.kCargoIntakePopoutSolenoidId);

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
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (GroundGearIntake.this) {
                    // What happens while the robot is on. This is usually a state machine
                    runStateMachine(); 
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stopLogging();
            }
        }
        );
    }


    public void runStateMachine() {
        switch (mState) {
        // Cases for each state and what the actuators should be at those states
            case IDLE:
                mPeriodicIO.GearIntSolenoid = false;
                mPeriodicIO.demand = 0;
            case INTAKE:
                mPeriodicIO.GearIntSolenoid = true;
                mPeriodicIO.demand = kVoltIntake;
            case PICKUP:
                mPeriodicIO.GearIntSolenoid = false;
                mPeriodicIO.demand = kVoltIntake;
            case CARRY:
                mPeriodicIO.GearIntSolenoid = false;
                mPeriodicIO.demand = 0;
            case SCORING:
                mPeriodicIO.GearIntSolenoid = false;
                mPeriodicIO.demand = kVoltOut;
            case OUTTAKE:
                mPeriodicIO.GearIntSolenoid = false;
                mPeriodicIO.demand = kVoltOut;
            case BALLDROP_GEAR:
                mPeriodicIO.GearIntSolenoid = true;
                mPeriodicIO.demand = 0;
            case BALLDROP_NOGEAR:
                mPeriodicIO.GearIntSolenoid = false;
                mPeriodicIO.demand = 0;

        }
    }


    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        // Cases to switch 
            case DROP:
              mState = State.INTAKE;
              break;
            case RISE:
              if (mState == State.INTAKE) {
                mState = State.IDLE;
              }
              break;
            case SCORE:
              if (mState == State.CARRY || mState == State.IDLE) {
                mState = State.SCORING;
              }
              break;
            case NONE:
              if (mState == State.SCORING) {
                mState = State.IDLE;
              }
              break;
            case OUTTAKE:
              mState = State.OUTTAKE;
              break;
            case START_BALLDROP:
              if (mState == State.PICKUP || mState == State.CARRY) {
                mState = State.BALLDROP_GEAR;
              }
              if (mState == State.INTAKE || mState == State.IDLE) {
                mState = State.BALLDROP_NOGEAR;
              }
              break;
            case STOP_BALLDROP:
              if (mState == State.BALLDROP_NOGEAR) {
                mState = State.IDLE;
              }
              if (mState == State.BALLDROP_NOGEAR) {
                mState = State.CARRY;
              }
              break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // Update anything you want to read from sensors or actuators, these are usually in your inputs under periodicIO
        mPeriodicIO.current = mMaster.getOutputCurrent();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        // Use .set on each of your actuators to whatever output you have been setting from periodicIO. This is also a good place to add limits to your code. 
        // Code below tells us the amount of volts the robot need to go
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        //
        mGearIntSolenoid.set(mPeriodicIO.GearIntSolenoid);


    }

    @Override
    public boolean checkSystem() {
        
       // Use CheckMotor function on any motor you have that uses PID, else just return true in this block
       return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser//*GroundGearIntake, make sure it is in CAPS*/-LOGS.csv", PeriodicIO.class);
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
        public boolean GearIntSolenoid;
    }
}


