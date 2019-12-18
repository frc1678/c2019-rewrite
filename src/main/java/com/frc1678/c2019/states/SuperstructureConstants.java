package com.frc1678.c2019.states;

public class SuperstructureConstants {
    public static double kWristMinAngle = 0.0;
    public static double kWristMaxAngle = 180.0;

    public static double kElevatorMinHeight = 0.0;
    public static double kElevatorMaxHeight = 69.0;

    // Passthru
    public static double kWristSafeForwardsAngle = 75.0;
    public static double kWristSafeBackwardsAngle = 105.0;
    public static double kElevatorSafeHeight = 2.5;

    // Safety
    public static final double kElevatorApproachingThreshold = 1.0;

    // Jog
    // This is in inches / ~20ms
    public final static double kElevatorJogThrottle = 60.0 / 50.0;

    // This is in degrees / ~20ms
    public final static double kWristJogThrottle = 5.0 / 20.0;
    

    // elevator constants
    public static double kHatchShipForwardsHeight = 5;
    public static double kHatchShipBackwardsHeight = 5;
    public static double kHatchRocketFirstHeight = 0.1;
    public static double kHatchRocketBackwardsHeight = 5;
    public static double kHatchRocketSecondHeight = 40;
    public static double kHatchRocketThirdHeight = 70;
    public static double kHatchLoadingStationHeight = 3.5;
    public static double kCargoShipForwardsHeight = 43;
    public static double kCargoShipBackwardsHeight = 5;
    public static double kCargoRocketFirstHeight = 3.5;
    public static double kCargoRocketBackwardsHeight = 5;
    public static double kCargoRocketSecondHeight = 38.5;
    public static double kCargoRocketThirdHeight = 70.5;
    public static double kGroundHeight = 0.;
    public static double kHandoffHeight = 10;
    public static double kSpitHeight = 0.;
    public static double kStowHeight = 0.;
    public static double kKissHeight = 55;
    public static double kClimbHeight = 2.0;
  
    public static double kCrawlerHeight = 65 - 1.5;
    public static double kBustDown = 12.3;
    public static double kElevatorPassThroughHeight = 2;
    public static double kElevatorHandoffTolerance = 2e-3;
    public static double kElevatorWristHorizHeight = 2;
    public static double kElevatorBoardHeight = 56;
    public static double kElevatorRezeroCurrentThreshold = 3;  // tune

    // Limelight height constants

    public static double kSwitchLimelightHeight = 20; // rough estimate

    // wrist constants
    public static double kHatchForwardsAngle = 0.0;
    public static double kHatchBackwardsAngle = 170.0;
    public static double kCargoRocketFirstAngle = 50;
    public static double kCargoRocketSecondAngle = 50;
    public static double kCargoRocketThirdAngle = 50;
    public static double kCargoRocketBackwardsAngle = 180.0;
    public static double kCargoShipForwardsAngle = 0.0;
    public static double kCargoShipBackwardsAngle = 170;
    public static double kGroundAngle = 0.0;
    public static double kHandoffAngle = 170;
    public static double kStowAngle = 75;
    public static double kDefStowAngle = 100;
    public static double kBustDownAngle = 20;
    public static double kWristHandoffTolerance = 3.0;
    public static double kWristRezeroCurrentThreshold = 10;  // tune
}   
