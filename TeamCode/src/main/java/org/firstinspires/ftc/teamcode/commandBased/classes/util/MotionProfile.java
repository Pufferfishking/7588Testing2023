package org.firstinspires.ftc.teamcode.commandBased.classes.util;

public class MotionProfile {

    private final double initialPosition;
    private final double finalPosition;
    private double accelPhase;
    private double cruisePhase;
    private final double totalTime;
    private final double accelStopPosition;
    private final double maxVelocity;
    private final double cruiseStopPosition;
    private double originalPos = 0;
    private boolean flipped = false;

    private final State state = new State();
    private final Constraints constraints;


    public MotionProfile(double initialPosition, double finalPosition, Constraints constraints) {
        if (finalPosition < initialPosition) {
            flipped = true;
            this.originalPos = initialPosition;
            double temp = initialPosition;
            initialPosition = finalPosition;
            finalPosition = temp;
        }
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        double distance = finalPosition - initialPosition;
        this.constraints = constraints;

        accelPhase = constraints.velo / constraints.accel;
        double decelPhase = constraints.velo / constraints.decel;

        cruisePhase = Math.abs(distance) / constraints.velo - (accelPhase + decelPhase) / 2;

        if (cruisePhase < 0) {
            System.out.println("Non-Max Velocity Profile");
            this.cruisePhase = 0;

            // this math utilizes a negative deceleration constant. either negate from the passed in value,
            // or just add a negatation symbol prior to the variable.
            double a = (constraints.accel / 2) * (1 - constraints.accel / -constraints.decel);
            double c = -distance;

            // acceleration phase
            accelPhase = Math.sqrt(-4 * a * c) / (2 * a);

            // deceleration phase
            decelPhase = -(constraints.accel * accelPhase) / -constraints.decel;

            // ending accel position (middle peak)
            accelStopPosition = (constraints.accel * Math.pow(accelPhase, 2)) / 2;

            // ending accel velocity (middle peak)
            maxVelocity = constraints.accel * accelPhase;

            cruiseStopPosition = accelStopPosition;
        } else {
            System.out.println("Max Velocity Profile");
            // time constants already calculated

            maxVelocity = constraints.velo;
            accelStopPosition = (constraints.velo * accelPhase) / 2;
            cruiseStopPosition = accelStopPosition + cruisePhase * maxVelocity;
        }

        totalTime = accelPhase + cruisePhase + decelPhase;
    }

    public State calculate(final double time) {
        double position, velocity, acceleration, stage_time;
        if (time <= accelPhase) {
            stage_time = time;
            acceleration = constraints.accel;
            velocity = acceleration * stage_time;
            position = velocity * stage_time / 2;
        } else if (time <= accelPhase + cruisePhase) {
            stage_time = time - accelPhase;
            acceleration = 0;
            velocity = constraints.velo;
            position = accelStopPosition + stage_time * velocity;
        } else if (time <= totalTime) {
            stage_time = time - accelPhase - cruisePhase;
            acceleration = -constraints.decel;
            velocity = maxVelocity - stage_time * constraints.decel;
            position = cruiseStopPosition + (maxVelocity + velocity) / 2 * stage_time;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        if (flipped) {
            state.x = originalPos - position;
        } else {
            state.x = initialPosition + position;
        }
        state.v = velocity;
        state.a = acceleration;
        return this.state;
    }


    public static class Constraints {
        public double velo = 0;
        public double accel = 0;
        public double decel = 0;

        public Constraints() {
        }

        public Constraints(double velo, double accel) {
            this.velo = velo;
            this.accel = accel;
            this.decel = accel;
        }

        public Constraints(double velo, double accel, double decel) {
            this.velo = velo;
            this.accel = accel;
            this.decel = decel;
        }
    }

    public static class State {
        public double x = 0;
        public double v = 0;
        public double a = 0;

        public State() {
        }

        public State(double x, double v, double a) {
            this.x = x;
            this.v = v;
            this.a = a;
        }
    }

//    public static void main(String[] args) {
//        ElapsedTime timer = new ElapsedTime();
//        ElapsedTime loopTimer = new ElapsedTime();
//        boolean done = false;
//        loopTimer.reset();
//        AsymmetricMotionProfile profile = new AsymmetricMotionProfile(0, 10, new Constraints(5, 5, 2));
//        while (!done) {
//            while (timer.seconds() < profile.totalTime && loopTimer.seconds() > 0.1) {
//                System.out.println(profile.calculate(timer.seconds()).a);
//                loopTimer.reset();
//            }
//            if (timer.seconds() > profile.totalTime) {
//                done = true;
//            }
//        }
//
//    }

}
