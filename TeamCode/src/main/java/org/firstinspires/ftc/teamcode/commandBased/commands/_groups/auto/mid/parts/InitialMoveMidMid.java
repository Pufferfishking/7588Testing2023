package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.ScoreCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.ScoreConeStack;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class InitialMoveMidMid extends SequentialCommandGroup {

    public InitialMoveMidMid(
            Subsystems subsystems,
            TrajectorySequence traj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new LiftMoveRotateArm(
                                subsystems.getEle(),
                                subsystems.getArm(),
                                subsystems.getRot(),
                                ARM_ANGLE_IDLE,
                                ELE_MID,
                                ROTATOR_FRONT
                        ),
                        new FollowTrajectorySequenceAsync(subsystems.rrDrive(), traj)
                )
                //new SetIntakePower(subsystems.getIntake(), -1)
        );
    }
}
