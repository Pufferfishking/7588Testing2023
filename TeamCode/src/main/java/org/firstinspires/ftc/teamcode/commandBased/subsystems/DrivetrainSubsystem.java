package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.commandBased.classes.apriltag.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.commandBased.classes.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandBased.classes.enums.DriveMode;
import org.firstinspires.ftc.teamcode.commandBased.classes.pid.DeadzonePID;
import org.firstinspires.ftc.teamcode.commandBased.classes.misc.Drive;
import org.firstinspires.ftc.teamcode.commandBased.classes.pid.PIDOpenClosed;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.commandBased.Constants;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

@Config
public class DrivetrainSubsystem extends SubsystemBase {

    //DRIVE MOTORS
    private DcMotor m_fL;
    private DcMotor m_fR;
    private DcMotor m_rL;
    private DcMotor m_rR;

    //DRIVE VARIABLES
    private DriveMode.Mode mode;
    private final double totalSpeed = 0.5;
    private double strafeMultiplier = 1;
    private double turnMultiplier = 1;
    private double forwardMultiplier = 1;
    private double heading;

    //TURNING VARIABLES
    private PIDCoefficientsEx turningCoeffs;
    private DeadzonePID turningPID;
    private AngleController turningController;
    private PIDOpenClosed turnPID;
    private double turningPIDDeadzone = 0.25;

    private TwoWheelLocalizer dwLocalizer;
    private AprilTagLocalizer atLocalizer;
    private com.acmerobotics.roadrunner.geometry.Pose2d pose = new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0);

    private final Drive drive;
    private LynxModule chub;
    private IMU imu;

    public DrivetrainSubsystem(final HardwareMap hwMap) {

        //motor setup
        Motor fL = new Motor(hwMap, "fL", Constants.DRIVE_MOTOR);
        Motor fR = new Motor(hwMap, "fR", Constants.DRIVE_MOTOR);
        Motor rL = new Motor(hwMap, "rL", Constants.DRIVE_MOTOR);
        Motor rR = new Motor(hwMap, "rR", Constants.DRIVE_MOTOR);

        m_fL = fL.motor;
        m_fR = fR.motor;
        m_rL = rL.motor;
        m_rR = rR.motor;

        m_fL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_rL.setDirection(DcMotorSimple.Direction.REVERSE);

        m_fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new Drive(fL, fR, rL, rR, Constants.TURN_COEFFS);

        //turning pid
        turningCoeffs = new PIDCoefficientsEx(1.5, 0.4, 0.4, 0.25, 2, 0.5);
        turningPID = new DeadzonePID(turningCoeffs, Math.toRadians(turningPIDDeadzone));
        turningController = new AngleController(turningPID);
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

//        atLocalizer = new AprilTagLocalizer(
//                this,
//                hwMap,
//                CAMERA_POSE,
//                CAMERA_1,
//                C920_INTRINSICS
//        );
    }

    @Override
    public void periodic() {
        heading = getRawExternalHeading();
        //atLocalizer.update();
    }


//    public Pose3d getTagPose() {
//        return atLocalizer.getTagPose();
//    }
//
//    public Transform3d getCamToTarget() {
//        return atLocalizer.getCamToTarget();
//    }
//
//    public Pose3d getCameraPose() {
//        return atLocalizer.getCameraPose();
//    }
//
//    public AprilTagDetection getTargetTag() {
//        return atLocalizer.getTargetTag();
//    }
//
//    public VisionPortal.CameraState getCameraState() {
//        return atLocalizer.getCameraState();
//    }

    public void setSpeedMultipliers(double strafeMultiplier, double forwardMultiplier, double turnMultiplier) {
        this.strafeMultiplier = strafeMultiplier;
        this.forwardMultiplier = forwardMultiplier;
        this.turnMultiplier = turnMultiplier;
    }

    public void fieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveFieldCentric(
                strafeSpeed * strafeMultiplier,
                forwardSpeed * forwardMultiplier,
                turnSpeed * turnMultiplier,
                heading
        );
        mode = DriveMode.Mode.FIELD_CENTRIC;
    }

    public void robotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(
                strafeSpeed * strafeMultiplier,
                forwardSpeed * forwardMultiplier,
                turnSpeed * turnMultiplier
        );
        mode = DriveMode.Mode.ROBOT_CENTRIC;
    }

    public void pointCentric(double strafeSpeed, double forwardSpeed, Vector2d target, Pose2d pose, double angleOffset) {
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
}
