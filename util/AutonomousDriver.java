package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonomousDriver {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    Odometry odometry;

    public static final int TOLERANCE = 20;
    public static final double DEGREE_TOLERANCE = 0.5;
    public static final double RADIAN_TOLERANCE = DEGREE_TOLERANCE / 180 * Math.PI;
    public static final double MAX_POWER = 0.5;
    public static final double TURN_POWER = 0.5;

    public AutonomousDriver(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, Odometry odometry) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;

        this.odometry = odometry;
    }

    public boolean linear(int x, int y) {
        // Get the current position
        int currentX = (int) odometry.x;
        int currentY = (int) odometry.y;
        // Find the angle between the two points
        int delta_x = x - currentX;
        int delta_y = y - currentY;
        double distance = Math.sqrt(Math.pow(delta_x, 2) + Math.pow(delta_y, 2));

        // Check tolerance
        if (distance < TOLERANCE) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            return false;
        }

        // Calculate angle at which the offset is located
        double theta = Math.atan2(delta_x, delta_y);

        double powerFrontLeft = Math.sin(theta + Math.PI / 4);
        double powerFrontRight = Math.sin(theta - Math.PI / 4);

        int max = 300;
        int min = 150;

        double clampedDistance = Math.max(Math.min(max, distance), min);

        double powerModifier = clampedDistance / max * MAX_POWER;

        frontLeftMotor.setPower(powerFrontLeft * powerModifier);
        frontRightMotor.setPower(powerFrontRight * powerModifier);
        backLeftMotor.setPower(powerFrontRight * powerModifier);
        backRightMotor.setPower(powerFrontLeft * powerModifier);

        return true;
    }

    public boolean turnRad(double targetAngle) {
        // Get current angle
        double current_angle = odometry.theta;
        double distance = targetAngle - current_angle;
        if (Math.abs(distance) < RADIAN_TOLERANCE) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            return false;
        }

        if (distance > 0) {
            frontRightMotor.setPower(TURN_POWER);
            backRightMotor.setPower(TURN_POWER);
            frontLeftMotor.setPower(-TURN_POWER);
            backLeftMotor.setPower(-TURN_POWER);
        } else {
            frontRightMotor.setPower(-TURN_POWER);
            backRightMotor.setPower(-TURN_POWER);
            frontLeftMotor.setPower(TURN_POWER);
            backLeftMotor.setPower(TURN_POWER);
        }

        return true;
    }

    public boolean turnDeg(double targetAngle) {
        return turnRad(targetAngle / 180 * Math.PI);
    }

    public void log (int x, int y, Telemetry telemetry) {
        telemetry.addData("Target position", x + ", " + y);
        telemetry.addData("Current position", odometry.x + ", " + odometry.y);
        telemetry.addData("Robot heading", odometry.theta);

        // Get the current position
        int currentX = (int) odometry.x;
        int currentY = (int) odometry.y;
        // Find the angle between the two points
        int delta_x = x - currentX;
        int delta_y = y - currentY;
        double distance = Math.sqrt(Math.pow(delta_x, 2) + Math.pow(delta_y, 2));

        telemetry.addData("Distance to target", distance);
    }

    public void log (double targetAngle, Telemetry telemetry) {
        telemetry.addData("Target angle", targetAngle);
        telemetry.addData("Current angle", odometry.theta);
        telemetry.addData("Difference", targetAngle - odometry.theta);
    }
}
