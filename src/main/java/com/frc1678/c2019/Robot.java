/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.frc1678.c2019;

import com.frc1678.c2019.auto.AutoModeExecutor;
import com.frc1678.c2019.auto.modes.AutoModeBase;
import com.frc1678.c2019.loops.Looper;
import com.frc1678.c2019.paths.TrajectoryGenerator;
import com.frc1678.c2019.statemachines.*;
import com.frc1678.c2019.statemachines.HatchIntakeStateMachine.WantedAction;
import com.frc1678.c2019.states.SuperstructureConstants;
import com.frc1678.c2019.subsystems.*;
import com.frc1678.c2019.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.Optional;



public class Robot extends TimedRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private boolean had_cargo_ = false;
    private boolean climb_mode = false;

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(RobotStateEstimator.getInstance(), Drive.getInstance(), LimelightManager.getInstance(), Superstructure.getInstance(),
                    HatchIntake.getInstance(), CargoIntake.getInstance(), Wrist.getInstance(), Elevator.getInstance(),
                    Climber.getInstance(), CarriageCanifier.getInstance(), Infrastructure.getInstance()
            ));

    private Drive mDrive = Drive.getInstance();
    private LimelightManager mLLManager = LimelightManager.getInstance();
    private HatchIntake mHatchIntake = HatchIntake.getInstance();
    private CargoIntake mCargoIntake = CargoIntake.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Infrastructure mInfrastructure = Infrastructure.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private Climber mClimber = Climber.getInstance();

    private AutoModeExecutor mAutoModeExecutor;

    private boolean mWantsDriverAuto;

    public Robot() {
        CrashTracker.logRobotConstruction();
        mTrajectoryGenerator.generateTrajectories();
    }

    @Override
    public void robotInit() {
        try {
            /*
             * //init camera stream UsbCamera camera =
             * CameraServer.getInstance().startAutomaticCapture();
             * camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15); MjpegServer
             * cameraServer = new MjpegServer("serve_USB Camera 0",
             * Constants.kCameraStreamPort); cameraServer.setSource(camera);
             */

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mInfrastructure.setIsDuringAuto(true);

            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();
            mInfrastructure.setIsDuringAuto(true);

            mWrist.setRampRate(Constants.kAutoWristRampRate);

            if (!mWantsDriverAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mInfrastructure.setIsDuringAuto(false);
            mWrist.setRampRate(Constants.kWristRampRate);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            mDrive.checkSystem();
            // mCargoIntake.checkSystem();
            // mWrist.checkSystem();
            // mElevator.checkSystem();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        // mLimelight.setStream(2);

        try {
            outputToSmartDashboard();
            mElevator.resetIfAtLimit();
            mWrist.resetIfAtLimit();
            mLLManager.setAllLeds(Limelight.LedMode.OFF);

            mAutoModeSelector.updateModeCreator();
            mWantsDriverAuto = mAutoModeSelector.isDriveByCamera();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
                System.gc();
            }

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
        try {
            if (mWantsDriverAuto || mAutoModeExecutor.isInterrupted()) {
                manualControl();
            } else if (mControlBoard.getInterruptAuto()) {
                mAutoModeExecutor.interrupt();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private void manualControl() {
        double timestamp = Timer.getFPGATimestamp();
        boolean vision = mControlBoard.getStartVision();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        if (vision && mLLManager.getHasTarget() && mLLManager.getLimelightOK()) {
            mDrive.updateVisionPID(mControlBoard.getStartVisionPressed());
        } else {
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(), false));
        }
       
        outputToSmartDashboard();

        final boolean cargo_preset = mCargoIntake.hasCargo();
        double desired_height = Double.NaN;
        double desired_angle = Double.NaN;
        HatchIntakeStateMachine.WantedAction idle_hatch_intake = mHatchIntake.hasHatch() ? WantedAction.PREP_SCORE
                : (mCargoIntake.hasCargo() ? WantedAction.NONE : WantedAction.INTAKE);
        if (!climb_mode) {
            if (mControlBoard.goToGround()) {
                desired_height = SuperstructureConstants.kGroundHeight;
                desired_angle = SuperstructureConstants.kGroundAngle;
                mHatchIntake.setState(idle_hatch_intake);
            } else if (mControlBoard.goToStow()) {
                desired_height = SuperstructureConstants.kStowHeight;
                desired_angle = SuperstructureConstants.kStowAngle;
                mHatchIntake.setState(WantedAction.HOLD);
            } else if (mControlBoard.goToShip()) {
                desired_height = cargo_preset ? SuperstructureConstants.kCargoShipForwardsHeight
                        : SuperstructureConstants.kHatchShipForwardsHeight;
                desired_angle = cargo_preset ? SuperstructureConstants.kCargoShipForwardsAngle
                        : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(idle_hatch_intake);
            } else if (mControlBoard.goToFirstLevel() && !mControlBoard.goToFirstLevelBackwards()) {
                desired_height = cargo_preset ? SuperstructureConstants.kCargoRocketFirstHeight
                        : SuperstructureConstants.kHatchRocketFirstHeight;
                desired_angle = cargo_preset ? SuperstructureConstants.kCargoRocketFirstAngle
                        : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(idle_hatch_intake);
            } else if (mControlBoard.goToSecondLevel()) {
                desired_height = cargo_preset ? SuperstructureConstants.kCargoRocketSecondHeight
                        : SuperstructureConstants.kHatchRocketSecondHeight;
                desired_angle = cargo_preset ? SuperstructureConstants.kCargoRocketSecondAngle
                        : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(idle_hatch_intake);
            } else if (mControlBoard.goToThirdLevel()) {
                desired_height = cargo_preset ? SuperstructureConstants.kCargoRocketThirdHeight
                        : SuperstructureConstants.kHatchRocketThirdHeight;
                desired_angle = cargo_preset ? SuperstructureConstants.kCargoRocketThirdAngle
                        : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(idle_hatch_intake);
            } else if (mControlBoard.goToFirstLevelBackwards()/* && !cargo_preset */) {
                desired_height = SuperstructureConstants.kHatchRocketBackwardsHeight;
                desired_angle = SuperstructureConstants.kHatchBackwardsAngle;
                mHatchIntake.setState(idle_hatch_intake);
            } else if (mControlBoard.getScoreHatch()) {
                mHatchIntake.setState(WantedAction.SCORE);
            } else {
                mHatchIntake.setState(WantedAction.NONE);
            }

            if (mControlBoard.getRunIntake()) {
                mCargoIntake.setState(CargoIntake.WantedAction.INTAKE);
            } else if (mControlBoard.getRunOuttake()) {
                mCargoIntake.setState(CargoIntake.WantedAction.OUTTAKE);
            } else {
                mCargoIntake.setState(CargoIntake.WantedAction.NONE);
            }

            if (mCargoIntake.hasCargo() && !had_cargo_ && !mControlBoard.getRunOuttake()) {
                if (mElevator.getInchesOffGround() < 5 && mWrist.getAngle() < 5) {
                    desired_height = SuperstructureConstants.kStowHeight;
                    desired_angle = SuperstructureConstants.kStowAngle;
                }
            }
        } else {
            mCargoIntake.forceIntakeIn();
            mHatchIntake.setState(HatchIntakeStateMachine.WantedAction.NONE);
            mCargoIntake.setState(CargoIntake.WantedAction.NONE);

            if (mControlBoard.dropCrawlers()) {
                System.out.println("Attempting DROP");
                desired_height = SuperstructureConstants.kCrawlerHeight;
                desired_angle = 0;
                mClimber.setState(Climber.WantedAction.DROP);
            }

            if (mElevator.getInchesOffGround() >= SuperstructureConstants.kCrawlerHeight - 10 && mControlBoard.Crawl()) {
                System.out.println("Attempting CRAWL");
                desired_height = 0.0;
                desired_angle = SuperstructureConstants.kBustDownAngle;
                mClimber.setState(Climber.WantedAction.CRAWL);
            }
            if (mControlBoard.finishClimb()) {
                System.out.println("Climb done");
                desired_angle = SuperstructureConstants.kBustDownAngle;
                desired_height = SuperstructureConstants.kBustDown;
                mClimber.setState(Climber.WantedAction.DONE);
            }
        }

        if (Double.isNaN(desired_angle) && Double.isNaN(desired_height)) {
            mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.IDLE);
        } else if (Double.isNaN(desired_angle)) {
            mSuperstructure.setDesiredHeight(desired_height);
        } else if (Double.isNaN(desired_height)) {
            mSuperstructure.setDesiredAngle(desired_angle);
        } else if (!Double.isNaN(desired_angle) && !Double.isNaN(desired_height)) {
            mSuperstructure.setDesiredAngle(desired_angle);
            mSuperstructure.setDesiredHeight(desired_height);
        }

        had_cargo_ = mCargoIntake.hasCargo();

        if (mControlBoard.climbMode()) {
            climb_mode = true;
            System.out.println("climb mode");
            desired_height = 0;
        }

        double elevator_jog = mControlBoard.getJogElevatorThrottle();
        if (Math.abs(elevator_jog) > Constants.kJoystickJogThreshold) {
            elevator_jog = (elevator_jog - Math.signum(elevator_jog) * Constants.kJoystickJogThreshold)
                    / (1.0 - Constants.kJoystickJogThreshold);
            mSuperstructure.setElevatorJog(elevator_jog * SuperstructureConstants.kElevatorJogThrottle);
        }

        double wrist_jog = mControlBoard.getJogWristThrottle();
        if (Math.abs(wrist_jog) > Constants.kJoystickJogThreshold) {
            wrist_jog = (wrist_jog - Math.signum(wrist_jog) * Constants.kJoystickJogThreshold)
                    / (1.0 - Constants.kJoystickJogThreshold);
            mSuperstructure.setWristJog(wrist_jog * SuperstructureConstants.kWristJogThrottle);
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            manualControl();
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        //Drive.getInstance().outputTelemetry();
        Wrist.getInstance().outputTelemetry();
        //CargoIntake.getInstance().outputTelemetry();
        //HatchIntake.getInstance().outputTelemetry();
        //Elevator.getInstance().outputTelemetry();
        //Infrastructure.getInstance().outputTelemetry();
        //LimelightManager.getInstance().outputTelemetry();
        mEnabledLooper.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }
}
