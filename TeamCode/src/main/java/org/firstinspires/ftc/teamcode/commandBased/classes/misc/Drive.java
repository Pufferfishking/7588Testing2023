package org.firstinspires.ftc.teamcode.commandBased.classes.misc;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commandBased.classes.pid.DeadzonePID;
import org.firstinspires.ftc.teamcode.commandBased.classes.poofypid.PoofyPIDCoefficients;
import org.firstinspires.ftc.teamcode.commandBased.classes.poofypid.PoofyPIDController;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector2d;

public class Drive {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private double maxOutput = 1;

    private final PoofyPIDController xController;
    private final PoofyPIDController yController;
    private final DeadzonePID headingDeadzone;
    private final AngleController thetaController;

    private double turnSpeed;
    private double theta;

    private Pose2d currentPose;

    private double[] speeds = new double[3];

    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        xController = new PoofyPIDController(0, 0, 0);
        yController = new PoofyPIDController(0, 0, 0);
        headingDeadzone = new DeadzonePID(new PIDCoefficientsEx(0, 0, 0, 1, 1, 0), 2);
        thetaController = new AngleController(headingDeadzone);
    }

    public Drive(DcMotorEx frontLeft,
                 DcMotorEx frontRight,
                 DcMotorEx backLeft,
                 DcMotorEx backRight,
                 PIDCoefficientsEx turningCoeffs,
                 PoofyPIDCoefficients xCoeffs,
                 PoofyPIDCoefficients yCoeffs
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        xController = new PoofyPIDController(xCoeffs);
        yController = new PoofyPIDController(yCoeffs);
        headingDeadzone = new DeadzonePID(turningCoeffs, Math.toRadians(2));
        thetaController = new AngleController(headingDeadzone);
        currentPose = new Pose2d(0, 0, 0);
    }

    public void setMaxSpeed(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;

        driveWithMotorPowers(
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        );
    }

    public void driveFieldCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double turnSpeed,
                                  double gyroAngle
    ) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        driveRobotCentric(
                input.getX(),
                input.getY(),
                turnSpeed
        );
    }

    public void drivePointCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double gyroAngle,
                                  Vector2d target,
                                  Pose2d currentPose,
                                  double angleOffset
    ) {

        this.currentPose = currentPose;

        Vector2d difference = target.subtract(currentPose.getVector());

        theta = difference.getAngle() + Math.toRadians(angleOffset);

        turnSpeed = thetaController.calculate(theta, currentPose.getTheta());

        double turnError = Math.abs(theta = currentPose.getTheta());
//
//        strafeSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));
//        forwardSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));

        driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                -turnSpeed,
                gyroAngle
        );
    }

    public void driveFollowTag(Pose2d tagPose,
                               Pose2d targetFollowingPose
    ) {
        xController.setTargetPosition(targetFollowingPose.getX());
        double strafeSpeed = xController.calculate(tagPose.getX());

        yController.setTargetPosition(targetFollowingPose.getY());
        double forwardSpeed = yController.calculate(tagPose.getY());

        double turnSpeed = thetaController.calculate(Math.toRadians(targetFollowingPose.getTheta()), Math.toRadians(tagPose.getTheta()));

        driveRobotCentric(
                -strafeSpeed,
                -forwardSpeed,
                turnSpeed
        );
    }



    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    public double getTurnTarget() {
        return theta;
    }

    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        this.frontLeft
                .setPower(frontLeftSpeed * maxOutput);
        this.frontRight
                .setPower(frontRightSpeed * maxOutput);
        this.backLeft
                .setPower(backLeftSpeed * maxOutput);
        this.backRight
                .setPower(backRightSpeed * maxOutput);
    }

}
