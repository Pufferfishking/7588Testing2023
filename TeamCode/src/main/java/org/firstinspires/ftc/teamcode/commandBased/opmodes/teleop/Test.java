package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.GrabCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.ScoreToIdle;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmIncrementally;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.UpdateArmPID;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.PointCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.ResetGyro;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.SetDriveSpeeds;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorIncrementally;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.UpdateElevatorPID;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.TeleOpMode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

@Disabled
@TeleOp
public class Test extends TeleOpMode {

    //drive commands
    protected FieldCentric fieldCentric;
    protected RobotCentric robotCentric;
    protected PointCentric pointCentric;

    protected SetDriveSpeeds slowMode;
    protected SetDriveSpeeds fastMode;

    protected ResetGyro resetGyro;

    //elevator commands
    protected MoveElevatorToPosition eleLow;
    protected MoveElevatorToPosition eleIdle;
    protected MoveElevatorToPosition eleMid;
    protected MoveElevatorToPosition eleHigh;

    protected MoveElevatorIncrementally eleManualDown;
    protected MoveElevatorIncrementally eleManualUp;

    protected UpdateElevatorPID updateElevatorPID;

    //arm commands
    protected MoveArmToAngle armBack;
    protected MoveArmToAngle armIdle;
    protected MoveArmToAngle armFront;

    protected UpdateArmPID updateArmPID;

    //rotator commands
    protected MoveRotatorToPosition rotatorBack;
    protected MoveRotatorToPosition rotatorFront;

    protected SetRotatorRange rotatorRange;

    //intake commands
    protected SetIntakePower intakeIntake;
    protected SetIntakePower intakeIdle;
    protected SetIntakePower intakeOuttake;

    //macro commands
    protected ScoreToIdle scoreToIdle;
    protected GrabCone grabCone;

    protected LiftMoveRotateArm armFrontMid;
    protected LiftMoveRotateArm armBackMid;
    protected LiftMoveRotateArm armFrontHigh;
    protected LiftMoveRotateArm armBackHigh;

    @Override
    public void initialize() {
        super.initialize();
        initializeAllCommands();


        //arm controls
        gp1(B, 1).whenActive(armFront);
        gp1(A, 1).whenActive(armIdle);
        gp1(X, 1).whenActive(armBack);

        //pid controls
        gp1(LEFT_STICK_BUTTON, 1).whenActive(updateElevatorPID);
        gp1(RIGHT_STICK_BUTTON, 1).whenActive(updateArmPID);

        //rotator controls
        gp1(BACK, 1).whenActive(rotatorBack);
        gp1(START, 1).whenActive(rotatorFront);

        //intake controls
        gp1(LEFT_BUMPER, 1).whenActive(intakeOuttake).whenInactive(intakeIdle);
        gp1(RIGHT_BUMPER, 1).whenActive(intakeIntake).whenInactive(intakeIdle);

        //macro controls
        gp1(Y, 2).whenActive(armBackHigh);
        gp1(X, 2).whenActive(armFrontHigh);
        gp1(B, 2).whenActive(armBackMid);
        gp1(A, 2).whenActive(armFrontMid);

        gp1(DPAD_UP, 2).whenActive(scoreToIdle);
        gp1(DPAD_DOWN, 2).whenActive(grabCone);

        gp1(A, 3).toggleWhenActive(robotCentric, fieldCentric);
        gp1(Y, 3).whenActive(resetGyro);



        gp2(DPAD_DOWN, 1).whenActive(eleLow);
        gp2(DPAD_LEFT, 1).whenActive(eleIdle);
        gp2(DPAD_RIGHT, 1).whenActive(eleMid);
        gp2(DPAD_UP, 1).whenActive(eleHigh);

        gp2(X, 1).whenActive(armFront);
        gp2(A, 1).whenActive(armIdle);
        gp2(B, 1).whenActive(armBack);

        gp2(Y, 1).toggleWhenActive(rotatorBack, rotatorFront);

        gp2(LEFT_BUMPER, 1).whenActive(armBackHigh);
        gp2(RIGHT_BUMPER, 1).whenActive(scoreToIdle);
        gp2(LEFT_TRIGGER, 1).whenActive(grabCone);



        robotCentric.schedule();
        rotatorRange.schedule();
        rotatorFront.schedule();
    }

    @Override
    public void run() {
        super.run();
    }



    protected void initializeAllCommands() {
        initializeDriveCommands();
        initializeElevatorCommands();
        initializeArmCommands();
        initializeRotatorCommands();
        initializeIntakeCommands();
        initializeMacroCommands();
    }

    protected void initializeDriveCommands() {
        fieldCentric = new FieldCentric(
                drivetrainSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );
        robotCentric = new RobotCentric(
                drivetrainSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );
        pointCentric = new PointCentric(
                drivetrainSS,
                () -> driver.getLeftX(),
                () -> -driver.getLeftY(),
                TARGET,
                ANGLE_OFFSET
        );

        slowMode = new SetDriveSpeeds(
                drivetrainSS,
                DRIVE_SLOW_STRAFE,
                DRIVE_SLOW_FORWARD,
                DRIVE_SLOW_TURN
        );
        fastMode = new SetDriveSpeeds(
                drivetrainSS,
                DRIVE_FAST_STRAFE,
                DRIVE_FAST_FORWARD,
                DRIVE_FAST_TURN
        );

        resetGyro = new ResetGyro(drivetrainSS);
    }

    protected void initializeElevatorCommands() {
        eleLow = new MoveElevatorToPosition(elevatorSS, ELE_LOW);
        eleIdle = new MoveElevatorToPosition(elevatorSS, ELE_IDLE);
        eleMid = new MoveElevatorToPosition(elevatorSS, ELE_MID);
        eleHigh = new MoveElevatorToPosition(elevatorSS, ELE_HIGH);

        updateElevatorPID = new UpdateElevatorPID(elevatorSS);

        eleManualDown = new MoveElevatorIncrementally(elevatorSS, ELE_MANUAL_DOWN);
        eleManualUp = new MoveElevatorIncrementally(elevatorSS, ELE_MANUAL_UP);
    }

    protected void initializeArmCommands() {
        armBack = new MoveArmToAngle(armSS, ARM_ANGLE_BACK);
        armIdle = new MoveArmToAngle(armSS, ARM_ANGLE_IDLE);
        armFront = new MoveArmToAngle(armSS, ARM_ANGLE_FRONT);

        updateArmPID = new UpdateArmPID(armSS);
    }

    protected void initializeRotatorCommands() {
        rotatorBack = new MoveRotatorToPosition(rotatorSS, ROTATOR_BACK);
        rotatorFront = new MoveRotatorToPosition(rotatorSS, ROTATOR_FRONT);

        rotatorRange = new SetRotatorRange(rotatorSS, TUNED_RANGE);
    }

    protected void initializeIntakeCommands() {
        intakeIntake = new SetIntakePower(intakeSS, 1);
        intakeIdle = new SetIntakePower(intakeSS, 0);
        intakeOuttake = new SetIntakePower(intakeSS, -0);
    }

    protected void initializeMacroCommands() {
        scoreToIdle = new ScoreToIdle(elevatorSS, armSS, rotatorSS, intakeSS);
        grabCone = new GrabCone(elevatorSS, armSS, rotatorSS, intakeSS);
        armFrontMid = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                ARM_ANGLE_FRONT,
                ELE_MID,
                ROTATOR_FRONT
        );
        armBackMid = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                ARM_ANGLE_BACK,
                ELE_MID,
                ROTATOR_BACK
        );
        armFrontHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                ARM_ANGLE_FRONT,
                ELE_HIGH,
                ROTATOR_FRONT
        );
        armBackHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                ARM_ANGLE_BACK,
                ELE_HIGH,
                ROTATOR_BACK
        );
    }

}
