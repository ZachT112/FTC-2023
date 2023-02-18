package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotConfig {
    public static final MotorInfo FRONT_RIGHT = new MotorInfo("motor_0", true, true);
    public static final MotorInfo BACK_RIGHT = new MotorInfo("motor_1", true, true);
    public static final MotorInfo BACK_LEFT = new MotorInfo("motor_2", true, false);
    public static final MotorInfo FRONT_LEFT = new MotorInfo("motor_3", true, false);

    public static final MotorInfo LINEAR_SLIDE = new MotorInfo("motor_4", true, false);

    public static final ServoInfo GRIP_SERVO = new ServoInfo("servo_0", 0, 1);

    public static final String COLOR_SENSOR = "sensor_4";

    public static class MotorInfo {
        private String name;
        private boolean enabled;
        private boolean reversed;

        public MotorInfo(String name, boolean enabled, boolean reversed) {
            this.name = name;
            this.enabled = enabled;
            this.reversed = reversed;
        }

        public String getName() {
            return name;
        }

        public boolean isEnabled() {
            return enabled;
        }

        public boolean isReversed() {
            return reversed;
        }
    }

    public static class ServoInfo {
        private String name;
        private double openPosition;
        private double closePosition;

        public ServoInfo(String name, double openPosition, double closePosition) {
            this.name = name;
            this.openPosition = openPosition;
            this.closePosition = closePosition;
        }

        public String getName() {
            return name;
        }

        public double getOpenPosition() {
            return openPosition;
        }

        public double getClosePosition() {
            return closePosition;
        }
    }
}
