package org.firstinspires.ftc.teamcode.commandBased.classes;

import static java.lang.Math.PI;

public class Angle {
    private static final double TAU = PI * 2;

    public static double norm(double angle) {
        double modifiedAngle = angle % TAU;
        modifiedAngle = (modifiedAngle + TAU) % TAU;
        return modifiedAngle;
    }

    public static double normDelta(double angleDelta) {
        double modifiedAngleDelta = norm(angleDelta);
        if (modifiedAngleDelta > PI) {
            modifiedAngleDelta -= TAU;
        }
        return modifiedAngleDelta;
    }
}
