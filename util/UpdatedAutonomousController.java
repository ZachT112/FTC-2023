package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConfig;

public abstract class UpdatedAutonomousController extends LinearOpMode {
    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    public static final int DRIVE_TOLERANCE = 3;
    public static final int MOTOR_DAMPENING_DISTANCE = 500;
    public static final double MINIMUM_POWER = 0.15;
    public static final double MAXIMUM_POWER = 1.00;
    public static final double POWER_RATE = 0.03;

    public void resetInternalPositions() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());
    }

    public void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void forward(int ticks) {
        setPower(MINIMUM_POWER);

        frontLeft.setTargetPosition(frontLeft.getTargetPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getTargetPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getTargetPosition() + ticks);
        backRight.setTargetPosition(backRight.getTargetPosition() + ticks);

        runToTarget();
        setPower(MINIMUM_POWER);
    }

    public void forward(int ticks, double power) {
        setPower(power);

        frontLeft.setTargetPosition(frontLeft.getTargetPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getTargetPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getTargetPosition() + ticks);
        backRight.setTargetPosition(backRight.getTargetPosition() + ticks);

        while(getDistanceFromPosition() > DRIVE_TOLERANCE && opModeIsActive());

        resetInternalPositions();
    }

    public void right(int ticks) {
        setPower(MINIMUM_POWER);

        frontLeft.setTargetPosition(frontLeft.getTargetPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getTargetPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getTargetPosition() - ticks);
        backRight.setTargetPosition(backRight.getTargetPosition() + ticks);

        runToTarget();
        setPower(MINIMUM_POWER);
    }

    public void left(int ticks) {
        right(-ticks);
    }

    public void turn(int ticks) {
        setPower(MINIMUM_POWER);

        frontRight.setTargetPosition(frontRight.getTargetPosition() + ticks);
        backRight.setTargetPosition(backRight.getTargetPosition() + ticks);
        frontLeft.setTargetPosition(frontLeft.getTargetPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getTargetPosition() - ticks);

        runToTarget();
        setPower(MINIMUM_POWER);
        sleep(500);
    }

    public void runToTarget() {
        int startingDistance = getDistanceFromPosition();
        double power = MINIMUM_POWER;
        while (getDistanceFromPosition() > startingDistance / 2 && opModeIsActive() && power < MAXIMUM_POWER) {
            power += POWER_RATE;
            power = clamp(power, MINIMUM_POWER, MAXIMUM_POWER);
            setPower(power);
            sleep(15);
        }
        while(getDistanceFromPosition() > MOTOR_DAMPENING_DISTANCE && opModeIsActive());
        setPower(MINIMUM_POWER);
        while(getDistanceFromPosition() > DRIVE_TOLERANCE && opModeIsActive());
        resetInternalPositions();
    }

    public int getDistanceFromPosition() {
        Vec4 vector = Vec4.abs(Vec4.getDifference(getMotorTargets(), getMotorPositions()));
        return vector.getMinimum();
    }

    public void backward(int ticks) {
        forward(-ticks);
    }

    public void backward(int ticks, double power) {
        forward(-ticks, power);
    }

    public Vec4 getMotorPositions() {
        return new Vec4(frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
    }

    public Vec4 getMotorTargets() {
        return new Vec4(frontLeft.getTargetPosition(), frontRight.getTargetPosition(), backLeft.getTargetPosition(), backRight.getTargetPosition());
    }

    public void setup() {
        frontLeft = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_LEFT.getName());
        frontRight = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_RIGHT.getName());
        backLeft = hardwareMap.get(DcMotor.class, RobotConfig.BACK_LEFT.getName());
        backRight = hardwareMap.get(DcMotor.class, RobotConfig.BACK_RIGHT.getName());

        if (RobotConfig.FRONT_LEFT.isReversed()) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (RobotConfig.FRONT_RIGHT.isReversed()) frontRight.setDirection(DcMotor.Direction.REVERSE);
        if (RobotConfig.BACK_LEFT.isReversed()) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (RobotConfig.BACK_RIGHT.isReversed()) backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        backLeft.setTargetPosition(backLeft.getCurrentPosition());
        backRight.setTargetPosition(backRight.getCurrentPosition());

        if (RobotConfig.FRONT_LEFT.isEnabled()) frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (RobotConfig.FRONT_RIGHT.isEnabled()) frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (RobotConfig.BACK_LEFT.isEnabled()) backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (RobotConfig.BACK_RIGHT.isEnabled()) backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private double clamp(double num, double min, double max) {
        return Math.max(Math.min(num, max), min);
    }
}
