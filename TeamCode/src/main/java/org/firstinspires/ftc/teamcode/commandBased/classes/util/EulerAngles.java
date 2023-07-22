package org.firstinspires.ftc.teamcode.commandBased.classes.util;

public class EulerAngles {

    double roll;
    double pitch;
    double yaw;

    public EulerAngles(double roll, double pitch, double yaw) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }
}
