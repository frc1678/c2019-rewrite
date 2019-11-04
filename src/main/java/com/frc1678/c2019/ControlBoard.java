package com.frc1678.c2019;

import com.frc1678.c2019.controlboard.*;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private IDriveControlBoard mDriveControlBoard;
    private IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getInterruptAuto() {
        return mDriveControlBoard.getInterruptAuto();
    }

    @Override
    public boolean goToGround() {
        return mButtonControlBoard.goToGround();
    }

    @Override
    public boolean goToStow() {
        return mButtonControlBoard.goToStow();
    }

    @Override
    public boolean goToFirstLevel() {
        return mButtonControlBoard.goToFirstLevel();
    }

    @Override
    public boolean goToSecondLevel() {
        return mButtonControlBoard.goToSecondLevel();
    }

    @Override
    public boolean goToThirdLevel() {
        return mButtonControlBoard.goToThirdLevel();
    }
    
    @Override
    public boolean goToFirstLevelBackwards() {
        return mButtonControlBoard.goToFirstLevelBackwards();
    }

    @Override
    public boolean goToShip() {
        return mButtonControlBoard.goToShip();
    }

    // Elevator
    @Override
    public double getJogElevatorThrottle() {
        return mButtonControlBoard.getJogElevatorThrottle();
    }

    // Wrist
    @Override
    public double getJogWristThrottle() {
        return mButtonControlBoard.getJogWristThrottle();
    }

    // Cargo Intake
    @Override
    public boolean getRunIntake() {
        return mButtonControlBoard.getRunIntake();
    }
    
    @Override
    public boolean getRunOuttake() {
        return mButtonControlBoard.getRunOuttake();
    }

    // Hatch Intake
    @Override
    public boolean getScoreHatch() {
        return mButtonControlBoard.getScoreHatch();
    }

    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    // Climbing
    @Override
    public boolean climbMode() {
        return mButtonControlBoard.climbMode();
    }

    @Override
    public boolean exitClimbMode() {
        return mButtonControlBoard.exitClimbMode();
    }

    @Override
    public boolean dropCrawlers() {
          return mButtonControlBoard.dropCrawlers();
    }
    
    @Override
    public boolean finishClimb() {
          return mButtonControlBoard.finishClimb();
    }
    
    @Override
    public boolean Crawl() {
        return mButtonControlBoard.Crawl();
    }
 /*   
    if we add forks
    @Override
    public boolean dropForks() {
        return mButtonControlBoard.dropForks();
    }    
*/
}

