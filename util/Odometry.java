package org.firstinspires.ftc.teamcode.util;

public class Odometry {
    public double x = 0;
    public double y = 0;
    public double theta = 0;

    public static final int TICKS_PER_REVOLUTION = 8192;
    public static final double TICKS_TO_MM = (double) 60 * Math.PI / (double) TICKS_PER_REVOLUTION;
    public static final double FRONT_RADIUS = 247.65 / 2;
    public static final double BACK_RADIUS = 242.7125;

    int previous_n1;
    int previous_n2;
    int previous_n3;

    public void setPrevious(int n1, int n2, int n3) {
        previous_n1 = n1;
        previous_n2 = n2;
        previous_n3 = n3;
    }

    public void update(int n1, int n2, int n3) {
        int s1 = n1 - previous_n1;
        int s2 = n2 - previous_n2;
        int s3 = n3 - previous_n3;

        setPrevious(n1, n2, n3);

        s1 *= TICKS_TO_MM;
        s2 *= TICKS_TO_MM;
        s3 *= TICKS_TO_MM;

        double robot_delta_x = (-s1 + s2) / 2;
        double robot_delta_theta = (s2 + s1) / (2 * FRONT_RADIUS);
        double robot_delta_y = s3 - robot_delta_theta * BACK_RADIUS;

        theta += robot_delta_theta;

        double alpha = theta - Math.PI / 2;

        double course_delta_x = Math.cos(theta) * robot_delta_x + Math.cos(alpha) * robot_delta_y;
        double course_delta_y = Math.sin(theta) * robot_delta_x + Math.sin(alpha) * robot_delta_y;

        x += course_delta_x;
        y += course_delta_y;
    }

}
