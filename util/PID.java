package org.firstinspires.ftc.teamcode.util;

public class PID {
    public double p, i, d;

    private volatile double setpoint;
    private volatile double variable;
    private volatile double output;
    private Active active;

    protected PID(double p, double i, double d, Active active) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.active = active;
    }

    public PID(double p, double i, double d, double setpoint, double variable, Active active) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.setpoint = setpoint;
        this.variable = variable;
        this.active = active;

        Thread pidLoop = new Thread(new PIDLoop(this), "pidLoop");
        pidLoop.start();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setVariable(double variable) {
        this.variable = variable;
    }

    public double getVariable() {
        return variable;
    }

    public void setOutput(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }

    public boolean isActive() {
        return active.call();
    }

    public double getError() {
        return getSetpoint() - getVariable();
    }

    public interface Active {
        boolean call();
    }
}
