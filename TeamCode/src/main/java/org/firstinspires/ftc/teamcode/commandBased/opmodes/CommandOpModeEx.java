package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {

    private boolean runOnce = false;

    public void runOnce() {};

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            if (!runOnce) {
                runOnce();
                runOnce = true;
            }
            run();
        }
        reset();
    }
}
