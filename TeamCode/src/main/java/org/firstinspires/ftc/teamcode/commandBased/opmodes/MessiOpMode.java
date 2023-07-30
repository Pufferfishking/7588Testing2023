package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBased.RobotHardware;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.AllSubsystems;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import java.util.List;

@TeleOp
public class MessiOpMode extends CommandOpModeEx {

    private ElapsedTime loopTimer;
    private int loops;



    private ElevatorSubsystem elevatorSS;

    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();



        robot.init(hardwareMap);
        elevatorSS = new ElevatorSubsystem(hardwareMap);

        robot.enabled = true;

        loopTimer = new ElapsedTime();

        telemetry.addLine("Ready:");
        telemetry.update();
    }

    @Override
    public void runOnce() {
        loopTimer.reset();
    }

    @Override
    public void run() {
        super.run();

        robot.loop(elevatorSS);
        robot.write(elevatorSS);

        loops++;
        telemetry.addData("loop timer", loopTimer.milliseconds());
        telemetry.addData("loops", loops);
        telemetry.addData("loop time", loopTimer.milliseconds() / loops);
        telemetry.update();

        robot.clearBulkCache();
    }
}
