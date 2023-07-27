package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

import java.util.List;

@TeleOp
public class MessiOpMode extends CommandOpModeEx {

    private ElapsedTime loopTimer;
    private int loops;

    private List<LynxModule> hubs;

    private DrivetrainSubsystem driveSS;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driveSS = new DrivetrainSubsystem(hardwareMap);

        telemetry.addLine("Ready:");
        telemetry.update();
    }

    @Override
    public void runOnce() {
        loopTimer = new ElapsedTime();
    }

    @Override
    public void run() {
        super.run();

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        //double[] vels = driveSS.getMotorVelocities();

        loops++;

        telemetry.addData("loop timer", loopTimer.milliseconds());
        telemetry.addData("loops", loops);
        telemetry.addData("loop time", loopTimer.milliseconds() / loops);
        telemetry.update();
    }
}
