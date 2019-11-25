package com.frc1678.c2019.teleop;

import com.frc1678.c2019.Constants;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class Teleop {
	private static Teleop mInstance = null;

	public static Teleop getInstance() {
		if (mInstance == null) {
			mInstance = new Teleop();
		}

		return mInstance;
	}

	private XboxController mJoystick;
	private final Joystick mThrottleStick;
	private final Joystick mTurnStick;

	private Teleop() {
		mJoystick = new XboxController(Constants.kButtonGamepadPort);
		mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
		mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
	}

	// Elevator-Wrist combos
	public boolean goToGround() {
		return mJoystick.getPOV() == 180; // South
	}

	public boolean goToStow() {
		return mJoystick.getPOV() == 0;
	}

	public boolean goToFirstLevel() {
		return mJoystick.getAButton();
	}

	public boolean goToSecondLevel() {
		return mJoystick.getBButton();
	}

	public boolean goToThirdLevel() {
		return mJoystick.getYButton();
	}

	public boolean goToFirstLevelBackwards() {
		return mJoystick.getRawAxis(1) < -Constants.kJoystickThreshold && goToFirstLevel();
	}

	public boolean goToShip() {
		return mJoystick.getXButton();
	}

	// Elevator
	public double getJogElevatorThrottle() {
		return - mJoystick.getRawAxis(5);
	}

	// Wrist
	public double getJogWristThrottle() {
		return - mJoystick.getRawAxis(4);
	}

	// Cargo Intake
	public boolean getRunIntake() {
		return mJoystick.getTriggerAxis(Hand.kRight) > Constants.kJoystickThreshold;
	}

	public boolean getRunOuttake() {
		return mJoystick.getTriggerAxis(Hand.kLeft) > Constants.kJoystickThreshold;
	}

	public boolean getScoreHatch() {
		return mJoystick.getBumper(Hand.kLeft);
	}

	public void setRumble(boolean on) {
		mJoystick.setRumble(RumbleType.kRightRumble, on ? 1.0 : 0.0);
	}

	// climb
	public boolean climbMode() {
		return mJoystick.getBumper(Hand.kLeft) && mJoystick.getBumper(Hand.kRight) && (mJoystick.getTriggerAxis(Hand.kLeft) > Constants.kJoystickThreshold) && (mJoystick.getTriggerAxis(Hand.kRight) > Constants.kJoystickThreshold);
	}

	public boolean dropCrawlers() {
		return mJoystick.getAButton();
	}

	public boolean Crawl() {
		return mJoystick.getBButton();
	}

	public boolean finishClimb() {
		return mJoystick.getYButton();
	}

	public boolean exitClimbMode() {
		return mJoystick.getPOV() == 180;
	}

	public double getThrottle() {
		return mThrottleStick.getRawAxis(1);
	}

	public double getTurn() {
		return - mTurnStick.getRawAxis(0);
	}

	public boolean getQuickTurn() {
		return mTurnStick.getRawButton(5);
	}

	public boolean getStartVision() {
		return mThrottleStick.getRawButton(1);
	}

	public boolean getStartVisionPressed() {
		return mThrottleStick.getRawButtonPressed(1);
	}

	public boolean getInterruptAuto() {
		return mThrottleStick.getRawButton(3);
	}

}