package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.AutonomousController;
import org.firstinspires.ftc.teamcode.util.Vec4;

@Autonomous(name = "Left", group = "2023")
@Disabled
public class Left extends AutonomousController {
    public static final double DRIVE_FAST = 0.60;
    public static final double DRIVE_MEDIUM = 0.50;
    public static final double DRIVE_SLOW = 0.40;
    public static final int LIFT_TOLERANCE = 6;

    ColorSensor colorSensor;
    DcMotor lift;
    Servo gripper;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStart();

        gripper.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());
        setDrivePower(DRIVE_FAST);
        setResetAfterWait(true);

        // Enable color sensor LED
        colorSensor.enableLed(true);

        // Create color variable
        SENSOR_COLOR color;
        // Go forward
        backward(750);
        waitForMotors();
        color = getColor();
        telemetry.clear();
        telemetry.addData("Color", color.name());
        telemetry.update();

        colorSensor.enableLed(false);

        // Go forward
        backward(2800 - 305 - 1000);
        // Wait for alpha to raise above 250
        // while (colorSensor.alpha() < 750 && opModeIsActive());
        // Check color
        // Finish driving and wait
        waitForMotors();

        //forward(300);
        //waitForMotors();

        setDrivePower(DRIVE_SLOW);
        turn(-1105);
        waitForMotors();
        setDrivePower(DRIVE_FAST);

        // Get first cone

        lift.setPower(1);
        lift.setTargetPosition(lift.getTargetPosition() + 400);

        int position = frontLeft.getCurrentPosition();
        forward(1050);
        while(Math.abs(position - frontLeft.getCurrentPosition()) < 900 && opModeIsActive());
        setDrivePower(DRIVE_MEDIUM);
        waitForMotors();
        setDrivePower(DRIVE_FAST);

        while (!checkLiftTolerance(lift) && opModeIsActive());

        gripper.setPosition(RobotConfig.GRIP_SERVO.getClosePosition());
        sleep(500);

        lift.setTargetPosition(lift.getTargetPosition() + 500);
        while (!checkLiftTolerance(lift) && opModeIsActive());

        lift.setTargetPosition(lift.getTargetPosition() + 1950);
        position = frontLeft.getCurrentPosition();
        backward(2050);
        while(Math.abs(position - frontLeft.getCurrentPosition()) < 900 && opModeIsActive());
        setDrivePower(DRIVE_MEDIUM);
        waitForMotors();

        setDrivePower(DRIVE_SLOW);
        right(755);
        waitForMotors();

        //setDrivePower(DRIVE_SLOW);
        //forward(200);
        //waitForMotors();

        gripper.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());
        sleep(250);

        //backward(200);
        //waitForMotors();
        //setDrivePower(DRIVE_FAST);


        left(760);
        waitForMotors();
        waitForMotors();

        lift.setTargetPosition(lift.getTargetPosition() - (1950 + 500 + 50));

        setDrivePower(DRIVE_FAST);

        forward(2050);
        waitForMotors();

        while(!checkLiftTolerance(lift) && opModeIsActive());

        gripper.setPosition(RobotConfig.GRIP_SERVO.getClosePosition());
        sleep(500);

        lift.setTargetPosition(lift.getTargetPosition() + 500);
        while(!checkLiftTolerance(lift) && opModeIsActive());

        lift.setTargetPosition(lift.getTargetPosition() + (1950 + 50));

        backward(2080);
        waitForMotors();

        while(!checkLiftTolerance(lift) && opModeIsActive());

        setDrivePower(DRIVE_MEDIUM);
        right(710);
        waitForMotors();

        gripper.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());
        sleep(250);

        left(710);
        waitForMotors();

        lift.setTargetPosition(lift.getTargetPosition() - (1950 + 500 + 50 + 350));
        while(!checkLiftTolerance(lift) && opModeIsActive());
        lift.setPower(0);

        int targetParking = color.ordinal();
        forward((2 - targetParking) * 1000);
        waitForMotors();

        while(opModeIsActive());
    }

    @Override
    protected void setup() {
        super.setup();

        gripper = hardwareMap.get(Servo.class, RobotConfig.GRIP_SERVO.getName());
        colorSensor = hardwareMap.get(ColorSensor.class, RobotConfig.COLOR_SENSOR);
        lift = hardwareMap.get(DcMotor.class, RobotConfig.LINEAR_SLIDE.getName());
        if (RobotConfig.LINEAR_SLIDE.isReversed()) lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public SENSOR_COLOR getColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (r < g && r < b) return SENSOR_COLOR.CYAN;
        if (g < r && g < b) return SENSOR_COLOR.MAGENTA;
        if (b < r && b < g) return SENSOR_COLOR.YELLOW;
        return SENSOR_COLOR.NULL;
    }

    private enum SENSOR_COLOR {
        CYAN, YELLOW, MAGENTA, NULL
    }

    public boolean checkLiftTolerance(DcMotor motor) {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < LIFT_TOLERANCE;
    }

    public boolean checkChange(Vec4 position, int tolerance) {
        return Vec4.abs(Vec4.getDifference(position, getPosition())).getAverage() > tolerance;
    }
}
