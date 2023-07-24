package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class PointCentric extends CommandBase {

    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier forwardSpeed;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Vector2d target;
    private final double angleOffset;

    public PointCentric(DrivetrainSubsystem drivetrainSubsystem,
                        DoubleSupplier strafeSpeed,
                        DoubleSupplier forwardSpeed,
                        Vector2d target,
                        double angleOffset) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.target = target;
        this.angleOffset = angleOffset;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.pointCentricMode(
                strafeSpeed.getAsDouble(),
                forwardSpeed.getAsDouble(),
                target,
                m_drivetrainSubsystem.convertRRPose(m_drivetrainSubsystem.getPose()),
                angleOffset
        );
    }
}
