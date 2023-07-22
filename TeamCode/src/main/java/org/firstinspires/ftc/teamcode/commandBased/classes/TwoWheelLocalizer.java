package org.firstinspires.ftc.teamcode.commandBased.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.rr.util.Encoder;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

import java.util.Arrays;
import java.util.List;

public class TwoWheelLocalizer extends TwoWheelLocalizerAbstract{

    private final Encoder parallelEncoder, perpendicularEncoder;
    private final DrivetrainSubsystem drive;

    public TwoWheelLocalizer(
            DrivetrainSubsystem drive,
            HardwareMap hwMap,
            Pose2d parallelPose,
            Pose2d perpendicularPose,
            String parallelEncoderName,
            String perpendicularEncoderName
    ) {
        super(Arrays.asList(parallelPose, perpendicularPose));

        this.drive = drive;

        parallelEncoder = new Encoder(hwMap.get(DcMotorEx.class, parallelEncoderName));
        perpendicularEncoder = new Encoder(hwMap.get(DcMotorEx.class, perpendicularEncoderName));

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);



    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
