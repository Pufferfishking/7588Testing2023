package org.firstinspires.ftc.teamcode.commandBased.classes.poofypid;

public class PoofyPIDCoefficients {

    private final double kP;
    private final double kI;
    private final double kD;

    public PoofyPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }
}
