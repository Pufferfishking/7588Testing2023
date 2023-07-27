package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.GrabCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.ScoreToIdle;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.UpdateArmPID;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.SetDriveSpeeds;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.PointCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;
import org.firstinspires.ftc.teamcode.rr.util.DashboardUtil;


import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@Disabled
@TeleOp(name="Command Based", group="Linear Opmode")
public class RobotExperiment extends BlackOp {

    @Override
    public void go() {

        //cancel all previous commands
        CommandScheduler.getInstance().reset();

        //create subsystem objects
        DrivetrainSubsystem drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        ElevatorSubsystem elevatorSS = new ElevatorSubsystem(hardwareMap);
        ArmSubsystem armSS = new ArmSubsystem(hardwareMap);
        RotatorSubsystem rotatorSS = new RotatorSubsystem(hardwareMap);
        IntakeSubsystem intakeSS = new IntakeSubsystem(hardwareMap);

        //create gamepads
        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
        ReforgedGamepad operator = new ReforgedGamepad(gamepad2);

        //create drivetrain mode commands
        FieldCentric fieldCentric = new FieldCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        RobotCentric robotCentric = new RobotCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        PointCentric pointCentric = new PointCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                Constants.TARGET,
                Constants.ANGLE_OFFSET
        );

        //create drivetrain speed commands
        SetDriveSpeeds slowMode = new SetDriveSpeeds(
                drivetrainSS,
                Constants.DRIVE_SLOW_STRAFE,
                Constants.DRIVE_SLOW_FORWARD,
                Constants.DRIVE_SLOW_TURN
        );
        SetDriveSpeeds fastMode = new SetDriveSpeeds(
                drivetrainSS,
                Constants.DRIVE_FAST_STRAFE,
                Constants.DRIVE_FAST_FORWARD,
                Constants.DRIVE_FAST_TURN
        );

        //create elevator commands
        MoveElevatorToPosition eleLow = new MoveElevatorToPosition(elevatorSS, Constants.ELE_LOW);
        MoveElevatorToPosition eleMidLow = new MoveElevatorToPosition(elevatorSS, Constants.ELE_IDLE);
        MoveElevatorToPosition eleMidHigh = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID);
        MoveElevatorToPosition eleHigh = new MoveElevatorToPosition(elevatorSS, Constants.ELE_HIGH);

        //create arm commands
        MoveArmToAngle armBackward = new MoveArmToAngle(
                armSS,
                Constants.ARM_ANGLE_BACK
        );
        MoveArmToAngle armIdle = new MoveArmToAngle(
                armSS,
                Constants.ARM_ANGLE_IDLE
        );
        MoveArmToAngle armForward = new MoveArmToAngle(
                armSS,
                Constants.ARM_ANGLE_FRONT
        );

        UpdateArmPID updateArmPID = new UpdateArmPID(armSS);

        //create rotator commands
        MoveRotatorToPosition rotatorBack = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_BACK);
        MoveRotatorToPosition rotatorFront = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_FRONT);

        SetRotatorRange rotatorRange = new SetRotatorRange(rotatorSS, Constants.TUNED_RANGE);

        //create intake commands
        SetIntakePower intakeIntake = new SetIntakePower(intakeSS, 1);
        SetIntakePower intakeIdle = new SetIntakePower(intakeSS, 0);
        SetIntakePower intakeOuttake = new SetIntakePower(intakeSS, -1);

        //create group commands
        ScoreToIdle scoreToIdle = new ScoreToIdle(
                elevatorSS,
                armSS,
                rotatorSS,
                intakeSS
        );

        GrabCone grabCone = new GrabCone(
                elevatorSS,
                armSS,
                rotatorSS,
                intakeSS
        );

        LiftMoveRotateArm armFrontMid = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_FRONT,
                Constants.ELE_MID,
                Constants.ROTATOR_FRONT
        );

        LiftMoveRotateArm armBackMid = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_BACK,
                Constants.ELE_MID,
                Constants.ROTATOR_BACK
        );

        LiftMoveRotateArm armFrontHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_FRONT,
                Constants.ELE_HIGH,
                Constants.ROTATOR_FRONT
        );

        LiftMoveRotateArm armBackHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_BACK,
                Constants.ELE_HIGH,
                Constants.ROTATOR_BACK
        );

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        //start robot in field-centric mode
        robotCentric.schedule();
        rotatorRange.schedule();
        rotatorFront.schedule();

        waitForStart();



        Scheduler.launchOnStart(this, () -> {

            //activate scheduler
            CommandScheduler.getInstance().run();

//            //drivetrain speed controls
//            driver.left_bumper.onRise(slowMode::schedule)
//                              .onFall(fastMode::schedule);


            //drivetrain mode controls
            driver.left_trigger.and(driver.a).onRise(() -> {
                pointCentric.cancel();
                fieldCentric.cancel();
                robotCentric.schedule();
            });
            driver.left_trigger.and(driver.b).onRise(() -> {
                pointCentric.cancel();
                robotCentric.cancel();
                fieldCentric.schedule();
            });
            driver.left_trigger.and(driver.x).onRise(() -> {
                fieldCentric.cancel();
                robotCentric.cancel();
                pointCentric.schedule();
            });

            //driver.y.onRise(drivetrainSS::resetGyro);

            //elevator controls
            driver.dpad_down.onRise(eleLow::schedule);
            driver.dpad_left.onRise(eleMidLow::schedule);
            driver.dpad_right.onRise(eleMidHigh::schedule);
            driver.dpad_up.onRise(eleHigh::schedule);

            //arm controls
            driver.b.onRise(armForward::schedule);
            driver.x.onRise(armBackward::schedule);
            driver.a.onRise(armIdle::schedule);

            //rotator controls
            driver.back.onRise(() -> rotatorBack.schedule(true));
            driver.back.onRise(rotatorBack::schedule);
            driver.start.onRise(rotatorFront::schedule);

            driver.left_stick_button.onRise(() -> CommandScheduler.getInstance().cancelAll());
            driver.right_stick_button.onRise(updateArmPID::schedule);

            //intake controls
            driver.left_bumper.onRise(intakeOuttake::schedule)
                              .onFall(intakeIdle::schedule);
            driver.right_bumper.onRise(intakeIntake::schedule)
                               .onFall(intakeIdle::schedule);

            driver.left_trigger(0.5).and(driver.a).onRise(scoreToIdle::schedule);
            driver.left_trigger(0.5).and(driver.right_trigger(0.5)).onRise(grabCone::schedule);

            driver.left_trigger(0.5).and(driver.y).onRise(armBackHigh::schedule);


            // Draw the target on the field
            fieldOverlay.setStroke("#dd2c00");
            fieldOverlay.strokeCircle(Constants.TARGET.getX(), Constants.TARGET.getY(), 3);

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, drivetrainSS.getPose());

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (Constants.DEBUG_DRIVE) {

            }

            if (Constants.DEBUG_ELE) {
                mTelemetry().addData("ele pos", elevatorSS.getElePos());
                mTelemetry().addData("ele profile target", elevatorSS.getEleProfileTarget());
                mTelemetry().addData("ele final target", elevatorSS.getEleTarget());
                mTelemetry().addData("ele power", elevatorSS.getElePower());
            }

            if (Constants.DEBUG_ARM) {
                mTelemetry().addData("arm final encoder target", armSS.getArmTargetEnc());
                mTelemetry().addData("arm final angle target", armSS.getArmTargetAngle());
                mTelemetry().addData("arm profile target", armSS.getArmProfileTarget());
                mTelemetry().addData("arm target", armSS.getArmTargetEnc());
                mTelemetry().addData("arm pos", armSS.getArmPos());
                mTelemetry().addData("arm power", armSS.getArmPower());
                mTelemetry().addData("arm angle", armSS.getArmAngle());
                mTelemetry().addData("arm velocity", armSS.getArmVelocity());
                mTelemetry().addData("arm acceleration", armSS.getArmAcceleration());
                mTelemetry().addData("arm KF", armSS.getCoeffs()[6]);
            }

            if (Constants.DEBUG_ROTATOR) {
                mTelemetry().addData("rotator pos", rotatorSS.getPosition());
                mTelemetry().addData("rotator usFrame", rotatorSS.getPWMRange()[0]);
                mTelemetry().addData("rotator usPulseLower", rotatorSS.getPWMRange()[1]);
                mTelemetry().addData("rotator usPulseUpper", rotatorSS.getPWMRange()[2]);
                mTelemetry().addData("rotator current", rotatorSS.getAverageCurrent());
            }

            if (Constants.DEBUG_INTAKE) {
                mTelemetry().addData("intake power", intakeSS.getPower());
                mTelemetry().addData("intake current", intakeSS.getServoBusCurrent());
                mTelemetry().addData("intake avg current", intakeSS.getAverageCurrent());
            }

//            mTelemetry().addData("back high scheduled", armBackHigh.isScheduled());
//            mTelemetry().addData("grab cone scheduled", grabCone.isScheduled());
//            mTelemetry().addData("score cone scheduled", scoreCone.isScheduled());
//
//            mTelemetry().addData("ele scheduled", eleHigh.isScheduled());
//            mTelemetry().addData("ele triggered", eleHigh.isTriggered());
//            mTelemetry().addData("ele finished", eleHigh.isFinished());
//
//            mTelemetry().addData("arm scheduled", armIdle.isScheduled());
//            mTelemetry().addData("arm triggered", armIdle.isTriggered());
//            mTelemetry().addData("arm finished", armIdle.isFinished());
//
//            mTelemetry().addData("rotator scheduled", rotatorBack.isScheduled());
//            mTelemetry().addData("rotator triggered", rotatorBack.isTriggered());
//            mTelemetry().addData("rotator finished", rotatorBack.isFinished());

            mTelemetry().update();
        });
    }
}
