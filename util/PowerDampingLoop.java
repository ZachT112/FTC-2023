package org.firstinspires.ftc.teamcode.util;

public class PowerDampingLoop implements Runnable {
    PowerHandler handler;
    Active active;

    double minPower;
    double maxPower;
    int startDistance;
    int endDistance;

    public PowerDampingLoop (PowerHandler handler, Active active, double minPower, double maxPower, int startDistance, int endDistance) {
        this.handler = handler;
        this.active = active;

        this.minPower = minPower;
        this.maxPower = maxPower;
        this.startDistance = startDistance;
        this.endDistance = endDistance;
    }

    @Override
    public void run() {
        while (active.call()) {
            handler.updatePower(minPower, maxPower, startDistance, endDistance);
        }
    }

    public interface Active {
        boolean call();
    }
}
