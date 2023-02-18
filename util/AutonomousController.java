package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotConfig;

public abstract class AutonomousController extends LinearOpMode {
    public static final int DRIVE_TOLERANCE = 3;
    public static final int START_DAMPING_DISTANCE = 200;
    public static final int END_DAMPING_DISTANCE = 50;
    public static final double MIN_POWER = 0.1;
    public static final double MAX_POWER = 0.5;

    protected volatile boolean useDamping = false;
    protected boolean useResetAfterWait;

    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    protected PowerHandler frontLeftHandler;
    protected PowerHandler frontRightHandler;
    protected PowerHandler backLeftHandler;
    protected PowerHandler backRightHandler;

    protected int frontLeftPosition;
    protected int frontRightPosition;
    protected int backLeftPosition;
    protected int backRightPosition;

    protected void setup() {
        frontLeft = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_LEFT.getName());
        frontRight = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_RIGHT.getName());
        backLeft = hardwareMap.get(DcMotor.class, RobotConfig.BACK_LEFT.getName());
        backRight = hardwareMap.get(DcMotor.class, RobotConfig.BACK_RIGHT.getName());

        frontLeftHandler = new PowerHandler(frontLeft);
        frontRightHandler = new PowerHandler(frontRight);
        backLeftHandler = new PowerHandler(backLeft);
        backRightHandler = new PowerHandler(backRight);

        if (RobotConfig.FRONT_LEFT.isReversed()) frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.FRONT_RIGHT.isReversed()) frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.BACK_LEFT.isReversed()) backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.BACK_RIGHT.isReversed()) backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftPosition = frontLeft.getCurrentPosition();
        frontRightPosition = frontRight.getCurrentPosition();
        backLeftPosition = backLeft.getCurrentPosition();
        backRightPosition = backRight.getCurrentPosition();

        updateTargets();

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void forward(int ticks) {
        frontLeftPosition += ticks;
        frontRightPosition += ticks;
        backLeftPosition += ticks;
        backRightPosition += ticks;
        updateTargets();
    }

    public void backward(int ticks) {
        forward(-ticks);
    }

    public void turn(int ticks) {
        frontLeftPosition -= ticks;
        frontRightPosition += ticks;
        backLeftPosition -= ticks;
        backRightPosition += ticks;
        updateTargets();
    }

    public void right(int ticks) {
        backRightPosition += ticks;
        frontLeftPosition += ticks;
        backLeftPosition -= ticks;
        frontRightPosition -= ticks;
        updateTargets();
    }

    public void left(int ticks) {
        right(-ticks);
    }

    public void updateTargets() {
        frontLeft.setTargetPosition(frontLeftPosition);
        frontRight.setTargetPosition(frontRightPosition);
        backLeft.setTargetPosition(backLeftPosition);
        backRight.setTargetPosition(backRightPosition);
    }

    public void stopMotors() {
        frontLeftPosition = frontLeft.getCurrentPosition();
        frontRightPosition = frontRight.getCurrentPosition();
        backLeftPosition = backLeft.getCurrentPosition();
        backRightPosition = backRight.getCurrentPosition();
        updateTargets();
    }

    public void waitForMotors() {
        if (useDamping) {
            while (!checkLiftTolerance(frontLeft) && opModeIsActive()) {
                frontLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                frontRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
            }
            while (!checkLiftTolerance(frontRight) && opModeIsActive()) {
                frontLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                frontRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
            }
            while (!checkLiftTolerance(backLeft) && opModeIsActive()) {
                frontLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                frontRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
            }
            while (!checkLiftTolerance(backRight) && opModeIsActive()) {
                frontLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                frontRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backLeftHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
                backRightHandler.updatePower(MIN_POWER, MAX_POWER, START_DAMPING_DISTANCE, END_DAMPING_DISTANCE);
            }
        } else {
            while (!checkLiftTolerance(frontLeft) && opModeIsActive());
            while (!checkLiftTolerance(frontRight) && opModeIsActive());
            while (!checkLiftTolerance(backLeft) && opModeIsActive());
            while (!checkLiftTolerance(backRight) && opModeIsActive());
        }

        if (useResetAfterWait) {
            frontLeftPosition = frontLeft.getCurrentPosition();
            frontRightPosition = frontRight.getCurrentPosition();
            backLeftPosition = backLeft.getCurrentPosition();
            backRightPosition = backRight.getCurrentPosition();
            updateTargets();
        }
    }

    public Vec4 getPosition() {
        return new Vec4(frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
    }

    protected boolean checkLiftTolerance(DcMotor motor) {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < DRIVE_TOLERANCE;
    }

    public void setUseDamping(boolean useDamping) {
        this.useDamping = useDamping;
    }

    public void setResetAfterWait(boolean useResetAfterWait) {
        this.useResetAfterWait = useResetAfterWait;
    }
}
