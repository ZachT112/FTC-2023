package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.onbotjava.OnBotJavaDelegatingStandardFileManager;
import org.firstinspires.ftc.teamcode.util.Odometry;

@TeleOp(name="Odometry Testing", group = "2023")
public class OdometryTesting extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    DcMotor back;

    Odometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "motor_0");
        right = hardwareMap.get(DcMotor.class, "motor_1");
        back = hardwareMap.get(DcMotor.class, "motor_2");

        odometry = new Odometry();

        waitForStart();

        odometry.setPrevious(left.getCurrentPosition(), right.getCurrentPosition(), back.getCurrentPosition());

        while (opModeIsActive()) {
            odometry.update(left.getCurrentPosition(), right.getCurrentPosition(), back.getCurrentPosition());

            telemetry.clear();
            telemetry.addData("n1", left.getCurrentPosition());
            telemetry.addData("n2", right.getCurrentPosition());
            telemetry.addData("n3", back.getCurrentPosition());
            telemetry.addData("X", odometry.x);
            telemetry.addData("Y", odometry.y);
            telemetry.addData("THETA", odometry.theta);
            telemetry.update();

            sleep(30);
        }
    }
}
