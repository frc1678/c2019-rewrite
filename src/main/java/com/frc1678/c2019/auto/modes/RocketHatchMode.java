package com.frc1678.c2019.auto.modes;

import com.frc1678.c2019.auto.AutoModeEndedException;
import com.frc1678.c2019.auto.actions.*;
import com.frc1678.c2019.paths.TrajectoryGenerator;

public class RocketHatchMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectory mHab1ToRocket;

    public RocketHatchMode(boolean left) {
        mHab1ToRocket = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().hab1ToRocket.get(left), true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(mHab1ToRocket);
    }
}
