package com.frc1678.lib.util;

import com.team254.lib.util.DriveSignal;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive.WheelState;
import com.team254.lib.util.Util;

public class TractionControlHelper {
    private DCMotorTransmission mLeftTransmission;
    private DCMotorTransmission mRightTransmission;
    private double kMaxTorque;

    public TractionControlHelper(DCMotorTransmission left, DCMotorTransmission right, double max_torque) {
        mLeftTransmission = left;
        mRightTransmission = right;
        kMaxTorque = max_torque;
    }

    public final DriveSignal tractionLimit(DriveSignal signal, WheelState current_speed) {
        final double left = Util.limit(signal.getLeft(), mLeftTransmission.getVoltageForTorque(current_speed.left, kMaxTorque));
        final double right = Util.limit(signal.getRight(), mRightTransmission.getVoltageForTorque(current_speed.right, kMaxTorque));
        return new DriveSignal(left, right);
    }
}