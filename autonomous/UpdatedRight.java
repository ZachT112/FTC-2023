package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.SimpleMath;
import org.firstinspires.ftc.teamcode.util.UpdatedAutonomousController;

@Autonomous(name = "Updated Right", group = "2023")
public class UpdatedRight extends UpdatedAutonomousController {
    public static final COLORS DEFAULT_COLOR = COLORS.CYAN;
    public static final int LIFT_TOLERANCE = 3;
    private DcMotor lift;
    private Servo servo;
    private ColorSensor colorSensor;
    private int initialLiftPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStart();

        lift.setPower(1);

        backward(750);
        COLORS color = getColor();
        telemetry.clear();
        telemetry.addData("Color", color.name());
        telemetry.update();
        backward(1700);
        forward(245);
        turn(1125);

        setLiftPosition(525);
        forward(1075);
        waitForLift();
        closeServo();
        sleep(1500);
        setLiftPosition(800);
        waitForLift();
        setLiftPosition(3000);
        backward(2050);
        waitForLift();

        left(650);
        forward(150, UpdatedAutonomousController.MINIMUM_POWER);
        openServo();
        sleep(500);
        backward(150, UpdatedAutonomousController.MINIMUM_POWER);
        right(650);

        setLiftPosition(450);
        forward(2200);
        waitForLift();
        closeServo();
        sleep(500);
        setLiftPosition(800);
        waitForLift();
        setLiftPosition(3000);
        backward(2050);

        left(650);
        forward(150, UpdatedAutonomousController.MINIMUM_POWER);
        openServo();
        sleep(500);
        backward(150, UpdatedAutonomousController.MINIMUM_POWER);
        right(650);

        setLiftPosition(0);
        waitForLift();
        forward(1100 * color.ordinal());
    }

    public void setLiftPosition(int ticks) {
        lift.setTargetPosition(initialLiftPosition + ticks);
    }

    public void waitForLift() {
        while (Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) > LIFT_TOLERANCE && opModeIsActive());
    }

    @Override
    public void setup() {
        super.setup();

        lift = hardwareMap.get(DcMotor.class, RobotConfig.LINEAR_SLIDE.getName());
        if (RobotConfig.LINEAR_SLIDE.isReversed()) lift.setDirection(DcMotor.Direction.REVERSE);
        initialLiftPosition = lift.getCurrentPosition();
        lift.setTargetPosition(initialLiftPosition);
        if (RobotConfig.LINEAR_SLIDE.isEnabled()) lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo = hardwareMap.get(Servo.class, RobotConfig.GRIP_SERVO.getName());

        colorSensor = hardwareMap.get(ColorSensor.class, RobotConfig.COLOR_SENSOR);
    }

    public void closeServo() {
        servo.setPosition(RobotConfig.GRIP_SERVO.getClosePosition());
    }

    public void openServo() {
        servo.setPosition(RobotConfig.GRIP_SERVO.getOpenPosition());
    }

    public COLORS getColor() {
        int[] rs = new int[10];
        int[] gs = new int[rs.length];
        int[] bs = new int[rs.length];

        for (int i = 0; i < rs.length; i++) {
            rs[i] = colorSensor.red();
            gs[i] = colorSensor.green();
            bs[i] = colorSensor.blue();
        }

        int r = SimpleMath.Average(rs);
        int g = SimpleMath.Average(gs);
        int b = SimpleMath.Average(bs);

        if (b > r && g > r) return COLORS.CYAN;
        if (g > r && b < r) return COLORS.YELLOW;
        if (b < r && g < r) return COLORS.MAGENTA;
        return DEFAULT_COLOR;
    }

    public enum COLORS {
        CYAN, YELLOW, MAGENTA
    }
}
