package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.commandBased.classes.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.commandBased.classes.apriltag.AprilTagLocalizerDouble;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandBased.classes.enums.DriveMode;
import org.firstinspires.ftc.teamcode.commandBased.classes.pid.DeadzonePID;
import org.firstinspires.ftc.teamcode.commandBased.classes.misc.Drive;
import org.firstinspires.ftc.teamcode.commandBased.classes.pid.PIDOpenClosed;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

@Config
public class DrivetrainSubsystem extends SubsystemBase {

    private final DcMotorEx fL;
    private final DcMotorEx fR;
    private final DcMotorEx rL;
    private final DcMotorEx rR;

    //DRIVE VARIABLES
    private DriveMode.Mode mode;
    private final double totalSpeed = 0.5;
    private double strafeMultiplier = 1;
    private double turnMultiplier = 1;
    private double forwardMultiplier = 1;
    private double heading;

    private final PIDOpenClosed turnPID;

    private final TwoWheelLocalizer dwLocalizer;
    private final AprilTagLocalizerDouble atLocalizer;
    private final com.acmerobotics.roadrunner.geometry.Pose2d pose = new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0);

    private final Drive drive;
    private LynxModule chub;
    private final IMU imu;

    public DrivetrainSubsystem(final HardwareMap hwMap) {

        //motor setup
        fL = hwMap.get(DcMotorEx.class, "fL");
        fR = hwMap.get(DcMotorEx.class, "fR");
        rL = hwMap.get(DcMotorEx.class, "rL");
        rR = hwMap.get(DcMotorEx.class, "rR");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        rL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new Drive(fL, fR, rL, rR, TURN_COEFFS, X_COEFFS, Y_COEFFS);


        //turning pid
        //TURNING VARIABLES
        PIDCoefficientsEx turningCoeffs = new PIDCoefficientsEx(1.5, 0.4, 0.4, 0.25, 2, 0.5);
        double turningPIDDeadzone = 0.25;
        DeadzonePID turningPID = new DeadzonePID(turningCoeffs, Math.toRadians(turningPIDDeadzone));
        AngleController turningController = new AngleController(turningPID);
        turnPID = new PIDOpenClosed(turningController, 0.2);

        chub = hwMap.getAll(LynxModule.class).get(0); //better ways to do this

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        dwLocalizer = new TwoWheelLocalizer(
                this,
                hwMap,
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90)),
                PARALLEL_ENCODER,
                PERPENDICULAR_ENCODER
        );

        atLocalizer = new AprilTagLocalizerDouble(
                this,
                hwMap,
                CAMERA_POSE,
                CAMERA_1,
                C920_INTRINSICS
        );

    }

    @Override
    public void periodic() {
        heading = getRawExternalHeading();
        atLocalizer.update();
    }


    public Pose3d getTagPose() {
        return atLocalizer.getTagPose();
    }

    public Transform3d getCamToTarget() {
        return atLocalizer.getCamToTarget();
    }

    public Pose3d getCameraPose() {
        return atLocalizer.getCameraPose();
    }

    public AprilTagDetection getTargetTag() {
        return atLocalizer.getTargetTag();
    }


    public Pose3d getTagPose2() {
        return atLocalizer.getTagPose2();
    }

    public Transform3d getCamToTarget2() {
        return atLocalizer.getCamToTarget2();
    }

    public Pose3d getCameraPose2() {
        return atLocalizer.getCamPose2();
    }

    public AprilTagDetection getTargetTag2() {
        return atLocalizer.getTargetTag2();
    }



    public VisionPortal.CameraState getCameraState() {
        return atLocalizer.getCameraState();
    }

    public void setSpeedMultipliers(double strafeMultiplier, double forwardMultiplier, double turnMultiplier) {
        this.strafeMultiplier = strafeMultiplier;
        this.forwardMultiplier = forwardMultiplier;
        this.turnMultiplier = turnMultiplier;
    }

    public void fieldCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveFieldCentric(
                strafeSpeed * strafeMultiplier,
                forwardSpeed * forwardMultiplier,
                turnSpeed * turnMultiplier,
                heading
        );
        mode = DriveMode.Mode.FIELD_CENTRIC;
    }

    public void robotCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(
                strafeSpeed * strafeMultiplier,
                forwardSpeed * forwardMultiplier,
                turnSpeed * turnMultiplier
        );
        mode = DriveMode.Mode.ROBOT_CENTRIC;
    }

    public void pointCentricMode(double strafeSpeed, double forwardSpeed, Vector2d target, Pose2d pose, double angleOffset) {
        drive.drivePointCentric(
                strafeSpeed * strafeMultiplier,
                forwardSpeed * forwardMultiplier,
                heading,
                target,
                pose,
                angleOffset
        );
        mode = DriveMode.Mode.POINT_CENTRIC;
    }

    public void followTagMode(Pose2d followPose) {
        if (getTargetTag() != null) {
            drive.driveFollowTag(
                    new Pose2d(
                            getTargetTag().ftcPose.x,
                            getTargetTag().ftcPose.y,
                            getTargetTag().ftcPose.yaw
                    ),
                    followPose);
        } else {
            drive.driveRobotCentric(0, 0, 0);
        }
        mode = DriveMode.Mode.FOLLOW_TAG;
    }

    public void resetGyro() {
        imu.resetYaw();
    }

    public com.acmerobotics.roadrunner.geometry.Pose2d getPose() {
        return pose;
    }

    public double getTurnSpeed() {
        return drive.getTurnSpeed();
    }

    public double getTurnTarget() {
        return drive.getTurnTarget();
    }

    public Pose2d getPointPose() {
        return drive.getCurrentPose();
    }

    public double getTurnAmount(double stick) {
        return turnPID.calculate(stick, Math.toRadians(getRawExternalHeading()));
    }

    public double getHeading() {
        return heading;
    }

    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public DriveMode.Mode getMode() {
        return mode;
    }

    public Pose2d convertRRPose(com.acmerobotics.roadrunner.geometry.Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }

    public double[] getMotorVelocities() {
        return new double[]{fL.getVelocity(), fR.getVelocity(), rL.getVelocity(), rR.getVelocity()};
    }
}
