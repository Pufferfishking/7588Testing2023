package org.firstinspires.ftc.teamcode.commandBased.classes;

import static java.lang.Math.cos;

import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector2d;

public class Kinematics {

    public static Pose2d relativeOdometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        double dtheta = robotPoseDelta.getTheta();
        double sineTerm, cosTerm;
        if (dtheta == 0) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - cos(dtheta)) / dtheta;
        }

        Vector2d fieldPositionDelta = new Vector2d(
                sineTerm * robotPoseDelta.getX() - cosTerm * robotPoseDelta.getY(),
                cosTerm * robotPoseDelta.getX() + sineTerm * robotPoseDelta.getY()
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotateBy(fieldPose.getTheta()), robotPoseDelta.getTheta());

        return new Pose2d(
                fieldPose.x + fieldPoseDelta.x,
                fieldPose.y + fieldPoseDelta.y,
                Angle.norm(fieldPose.theta + fieldPoseDelta.theta)
        );
    }
}
