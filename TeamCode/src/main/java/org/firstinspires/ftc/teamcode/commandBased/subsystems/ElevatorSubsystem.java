package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.Positions;


public class ElevatorSubsystem extends SubsystemBase {

    private final DcMotorEx eleL;
    private final DcMotorEx eleR;

    //ELEVATOR VARIABLES
    private double elePos;
    private double eleTarget;
    private final double eleEncOffset;

    private PIDFController controller;
    private MotionProfile profile;
    private final ElapsedTime timer;
    private MotionState state;
    private double correction;

    public ElevatorSubsystem(final HardwareMap hwMap){

        //motor setup
        eleL = hwMap.get(DcMotorEx.class, "eleL");
        eleR = hwMap.get(DcMotorEx.class, "eleR");

        eleL.setDirection(DcMotorSimple.Direction.REVERSE);
        eleL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eleEncOffset = Positions.elePosition;

        //pid controller and motion profile setup
        controller = new PIDFController(
                Constants.ELE_COEFFS,
                Constants.ELE_KV,
                Constants.ELE_KA,
                Constants.ELE_KS
        );
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(0, 0, 0),
                inchesToTicks(Constants.ELE_MAX_VEL),
                inchesToTicks(Constants.ELE_MAX_ACCEL)
        );
        state = profile.get(timer.seconds());
    }

    @Override
    public void periodic() {
        //read();

        elePos = getOffsetPos(elePos);

        //motion profiling
        state = profile.get(timer.seconds());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        correction = controller.update(elePos);

        //write();
    }

    public void read() {
        try {
            elePos = ((eleL.getCurrentPosition() + eleR.getCurrentPosition()) / 2.0);
        } catch (Exception e) {

        }
    }

    public void write() {
        try {
            eleL.setPower(correction + Constants.ELE_KG);
        } catch (Exception e) {

        }

        try {
            eleR.setPower(correction + Constants.ELE_KG);
        } catch (Exception e) {

        }

    }

    private double getOffsetPos(double pos) {
        return pos + eleEncOffset;
    }

    public void createNewController() {
        controller = new PIDFController(
                Constants.ELE_COEFFS,
                Constants.ELE_KV,
                Constants.ELE_KA,
                Constants.ELE_KS
        );
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
    }

    public void setProfileTarget(double target) {
        eleTarget = inchesToTicks(target);
        target = eleTarget;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(state.getX(), 0, 0),
                new MotionState(target, 0, 0),
                inchesToTicks(Constants.ELE_MAX_VEL),
                inchesToTicks(Constants.ELE_MAX_ACCEL)
        );
        timer.reset();
    }

    public void moveProfileTarget(double amount) {
        setProfileTarget(getElePos() + amount);
    }

    public boolean isFinished() {
        return eleTarget >= elePos - inchesToTicks(Constants.ELE_DONE_DEADZONE)
            && eleTarget <= elePos + inchesToTicks(Constants.ELE_DONE_DEADZONE);
    }

    public double getElePower() {
        return (correction + Constants.ELE_KG);
    }

    public double getEleTarget() {
        return ticksToInches(eleTarget);
    }

    public double getEleProfileTarget() {
        return ticksToInches(state.getX());
    }

    public double getElePos() {
        return ticksToInches(elePos);
    }

    public double inchesToTicks(double inches) {
        return inches * 79.0426072601;
    }

    public double ticksToInches(double ticks) {
        return ticks / 79.0426072601;
    }
}
