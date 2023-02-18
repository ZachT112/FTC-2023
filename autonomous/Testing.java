package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.SimpleMath;
import org.firstinspires.ftc.teamcode.util.UpdatedAutonomousController;

@Autonomous(name = "Testing", group = "2023")
@Disabled
public class Testing extends UpdatedAutonomousController {
    private ColorSensor colorSensor;
    private int[] reds = new int[1000];
    private int[] greens = new int[1000];
    private int[] blues = new int[1000];


    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStart();

        backward(750);

        for (int i = 0; i < 1000; i++) {
            reds[i] = colorSensor.red();
            greens[i] = colorSensor.green();
            blues[i] = colorSensor.blue();
        }

        telemetry.clear();
        telemetry.addData("Red", SimpleMath.Average(reds));
        telemetry.addData("Green", SimpleMath.Average(greens));
        telemetry.addData("Blue", SimpleMath.Average(blues));
        telemetry.update();

        while(opModeIsActive());
    }

    @Override
    public void setup() {
        super.setup();

        colorSensor = hardwareMap.get(ColorSensor.class, RobotConfig.COLOR_SENSOR);
    }

}
