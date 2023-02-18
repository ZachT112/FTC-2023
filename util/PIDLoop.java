package org.firstinspires.ftc.teamcode.util;

public class PIDLoop implements Runnable {
    PID pid;
    private double previous_error = 0, integral = 0;
    private long deltaTime = 10;

    public PIDLoop(PID pid) {
        this.pid = pid;
    }

    @Override
    public void run() {
        while (pid.isActive()) {
            double error = pid.getError();
            double proportional = error;
            integral += error * deltaTime;
            double derivative = (error - previous_error) / deltaTime;
            pid.setOutput(pid.p * proportional + pid.i * integral + pid.d * derivative);
            previous_error = error;
            long waitTime = System.currentTimeMillis() + deltaTime;
            while (waitTime > System.currentTimeMillis() && pid.isActive());
        }
    }
}
