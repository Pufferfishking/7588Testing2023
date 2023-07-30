package org.firstinspires.ftc.teamcode.commandBased.classes.poofypid;

public class PoofyPIDFCoefficients {

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kV = 0;
    private double kA = 0;
    private double kS = 0;
    private double kF = 0;

    public PoofyPIDFCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PoofyPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD);
        this.kF = kF;
    }

    public PoofyPIDFCoefficients(double kP, double kI, double kD, double kV, double kA) {
        this(kP, kI, kD);
        this.kV = kV;
        this.kA = kA;
    }

    public PoofyPIDFCoefficients(double kP, double kI, double kD, double kV, double kA, double kS, double kF) {
        this(kP, kI, kD, kV, kA);
        this.kS = kS;
        this.kF = kF;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }


    public static class Builder {
        private double kP = 0;
        private double kI = 0;
        private double kD = 0;
        private double kV = 0;
        private double kA = 0;
        private double kS = 0;
        private double kF = 0;

        public Builder addP(double kP) {
            this.kP = kP;
            return this;
        }

        public Builder addI(double kI) {
            this.kI = kI;
            return this;
        }

        public Builder addD(double kD) {
            this.kD = kD;
            return this;
        }

        public Builder addV(double kV) {
            this.kV = kV;
            return this;
        }

        public Builder addA(double kA) {
            this.kA = kA;
            return this;
        }

        public Builder addS(double kS) {
            this.kS = kS;
            return this;
        }

        public Builder addF(double kF) {
            this.kF = kF;
            return this;
        }

        public Builder addPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public Builder addVA(double kV, double kA) {
            this.kV = kV;
            this.kA = kA;
            return this;
        }

        public PoofyPIDFCoefficients build() {
            return new PoofyPIDFCoefficients(kP, kI, kD, kV, kA, kS, kF);
        }
    }

}
