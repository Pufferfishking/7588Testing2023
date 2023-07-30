package org.firstinspires.ftc.teamcode.commandBased.classes.poofypid;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

public class PoofyPIDCoefficients {

    public double kP;
    public double kI;
    public double kD;

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
