package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Odometry;
import org.firstinspires.ftc.teamcode.util.PID;

@TeleOp(name = "Driver Controlled", group = "2022-2023")
public class DriverControlled extends LinearOpMode {
    private static final double STEERING_SENSITIVITY = 0.5;
    private static final int MAX_LIFT_UP_DISTANCE = 3000;
    private static final int MAX_LIFT_DOWN_DISTANCE = 0;
    private static final double HIGH = 0.80;
    private static final double LOW = 0.50;
    private static final double HIGH_TURN = 0.90;
    private static final double LOW_TURN = 0.70;

    DcMotor front_right;
    DcMotor back_right;
    DcMotor back_left;
    DcMotor front_left;

    DcMotor linear_slide;

    Servo gripper;

    boolean lowPower = false;
    boolean lowPowerSwitch = true;

    boolean useClamping = true;
    boolean useClampingSwitch = true;

    int liftMinimum;
    int liftMaximum;

    Odometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStart();

        linear_slide.setPower(1);

        gripper.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());

        while (opModeIsActive()) {
            // Game pad 1 controls
            // Power controls
            if (gamepad1.x && lowPowerSwitch) {
                lowPowerSwitch = false;
                lowPower = !lowPower;
            } else if (!gamepad1.x && !lowPowerSwitch) {
                lowPowerSwitch = true;
            }

            // Driving controls
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                double power;
                if (gamepad1.dpad_up) {
                    power = 1;
                } else {
                    power = -1;
                }
                power *= lowPower ? LOW : HIGH;
                front_left.setPower(power);
                front_right.setPower(power);
                back_left.setPower(power);
                back_right.setPower(power);
            } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                double power1, power2;
                if (gamepad1.dpad_left) {
                    power1 = -1;
                    power2 = 1;
                } else {
                    power1 = 1;
                    power2 = -1;
                }
                power1 *= lowPower ? LOW : HIGH;
                power2 *= lowPower ? LOW : HIGH;
                front_left.setPower(power1);
                back_right.setPower(power1);
                front_right.setPower(power2);
                back_left.setPower(power2);
            } else if (Math.abs(gamepad1.left_stick_y) == 0 && Math.abs(gamepad1.left_stick_x) == 0 && Math.abs(gamepad1.right_stick_x) != 0) {
                front_right.setPower(-gamepad1.right_stick_x * STEERING_SENSITIVITY * (lowPower ? LOW_TURN : HIGH_TURN));
                back_right.setPower(-gamepad1.right_stick_x * STEERING_SENSITIVITY * (lowPower ? LOW_TURN : HIGH_TURN));
                back_left.setPower(gamepad1.right_stick_x * STEERING_SENSITIVITY * (lowPower ? LOW_TURN : HIGH_TURN));
                front_left.setPower(gamepad1.right_stick_x * STEERING_SENSITIVITY * (lowPower ? LOW_TURN : HIGH_TURN));
            } else if (Math.abs(gamepad1.left_stick_y) != 0) {
                double left_side = -gamepad1.left_stick_y;
                double right_side = -gamepad1.left_stick_y;
                if (gamepad1.dpad_up) {
                    left_side = 1;
                    right_side = 1;
                } else if (gamepad1.dpad_down) {
                    left_side = -1;
                    right_side = -1;
                }
                left_side += gamepad1.right_stick_x * STEERING_SENSITIVITY;
                right_side -= gamepad1.right_stick_x * STEERING_SENSITIVITY;
                double normal = Math.max(Math.abs(left_side), Math.abs(right_side));
                left_side /= normal;
                right_side /= normal;
                left_side *= Math.abs(gamepad1.left_stick_y);
                right_side *= Math.abs(gamepad1.left_stick_y);
                front_right.setPower(right_side * (lowPower ? LOW : HIGH));
                back_right.setPower(right_side * (lowPower ? LOW : HIGH));
                back_left.setPower(left_side * (lowPower ? LOW : HIGH));
                front_left.setPower(left_side * (lowPower ? LOW : HIGH));
            } else if (Math.abs(gamepad1.left_stick_x) != 0) {
                front_right.setPower(-gamepad1.left_stick_x * (lowPower ? LOW : HIGH));
                back_right.setPower(gamepad1.left_stick_x * (lowPower ? LOW : HIGH));
                back_left.setPower(-gamepad1.left_stick_x * (lowPower ? LOW : HIGH));
                front_left.setPower(gamepad1.left_stick_x * (lowPower ? LOW : HIGH));
            } else {
                front_right.setPower(0);
                back_right.setPower(0);
                back_left.setPower(0);
                front_left.setPower(0);
            }

            // Game pad 2 controls

            // Clamp enable/disable
            if (gamepad2.x && useClampingSwitch) {
                useClampingSwitch = false;
                useClamping = !useClamping;
            } else if (!gamepad2.x && !useClampingSwitch) {
                useClampingSwitch = true;
            }

            // Reset minimum
            if (gamepad2.y && !useClamping) {
                liftMinimum = linear_slide.getCurrentPosition();
                liftMaximum = liftMinimum + (MAX_LIFT_UP_DISTANCE - MAX_LIFT_DOWN_DISTANCE);
            }

            // Linear slide controls
            if (Math.abs(gamepad2.left_stick_y) > 0) {
                int deltaPosition = (int) (-gamepad2.left_stick_y * 75);
                int newPosition = linear_slide.getCurrentPosition() + deltaPosition;
                if (useClamping) newPosition = clamp(newPosition, liftMinimum, liftMaximum);
                linear_slide.setTargetPosition(newPosition);
            }
            if (gamepad2.dpad_up) linear_slide.setTargetPosition(liftMaximum);
            if (gamepad2.dpad_down) linear_slide.setTargetPosition(liftMinimum);

            // Gripper controls
            if (gamepad2.a) gripper.setPosition(RobotConfig.GRIP_SERVO.getClosePosition());
            if (gamepad2.b) gripper.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());
            if (gamepad2.right_trigger != 0) gripper.setPosition(RobotConfig.GRIP_SERVO.getClosePosition());
            if (gamepad2.left_trigger != 0) gripper.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());

            // Logging
            telemetry.clear();
            telemetry.addData("Clamping Enabled?", useClamping);
            telemetry.addData("Low Power Drive Enabled?", lowPower);
            telemetry.update();

            sleep(25);
        }
    }

    public void setup() {
        front_right = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_RIGHT.getName());
        back_right = hardwareMap.get(DcMotor.class, RobotConfig.BACK_RIGHT.getName());
        back_left = hardwareMap.get(DcMotor.class, RobotConfig.BACK_LEFT.getName());
        front_left = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_LEFT.getName());

        linear_slide = hardwareMap.get(DcMotor.class, RobotConfig.LINEAR_SLIDE.getName());
        liftMinimum = linear_slide.getCurrentPosition() + MAX_LIFT_DOWN_DISTANCE;
        liftMaximum = linear_slide.getCurrentPosition() + MAX_LIFT_UP_DISTANCE;
        linear_slide.setTargetPosition(liftMinimum);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripper = hardwareMap.get(Servo.class, RobotConfig.GRIP_SERVO.getName());

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (RobotConfig.FRONT_LEFT.isReversed()) front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.FRONT_RIGHT.isReversed()) front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.BACK_LEFT.isReversed()) back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.BACK_RIGHT.isReversed()) back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RobotConfig.LINEAR_SLIDE.isReversed()) linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private int clamp(int num, int min, int max) {
        return Math.max(Math.min(max, num), min);
    }
}
