package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PowerHandler {
    private volatile DcMotor motor;

    public PowerHandler(DcMotor motor) {
        this.motor = motor;
    }

    public void updatePower(double minPower, double maxPower, int startDistance, int endDistance) {
        // Get the difference
        int targetDifference = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        // Check if out of range
        if (targetDifference > startDistance) {
            motor.setPower(maxPower);
            return;
        }
        // Check if minimum distance
        if (targetDifference < endDistance) {
            motor.setPower(minPower);
            return;
        }
        // Calculate damping
        int distanceRange = startDistance - endDistance;
        double damping = (double) (targetDifference - endDistance) / distanceRange;
        double powerRange = maxPower - minPower;
        double power =  powerRange * damping + minPower;
        // Set power
        motor.setPower(power);
    }
}
