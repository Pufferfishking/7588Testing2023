package org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.EulerAngles;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.MathEx;

public class Rotation3d {
    private Quaternion q = new Quaternion();

    public Rotation3d() {}

    public Rotation3d(Quaternion q) {
        this.q = q.normalized();
    }

    public Rotation3d(double roll, double pitch, double yaw) {
        q = MathEx.eulerToQuaternion(roll, pitch, yaw);
    }

    public Rotation3d(Vector3d axis, double angleRadians) {
        double norm = axis.getMagnitude();
        if (norm == 0.0) {
            return;
        }

        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Definition
        Vector3d v = axis.scale(1.0 / norm).scale(Math.sin(angleRadians / 2.0));
        q = new Quaternion((float) Math.cos(angleRadians / 2.0), (float) v.getX(), (float)  v.getY(), (float)  v.getZ(), 0);
    }

    public Rotation3d plus(Rotation3d other) {
        return rotateBy(other);
    }

    public Rotation3d minus(Rotation3d other) {
        return rotateBy(other.unaryMinus());
    }

    public Rotation3d unaryMinus() {
        return new Rotation3d(q.inverse());
    }

    public Rotation3d times(double scalar) {
        if (q.w >= 0.0) {
            return new Rotation3d(
                    new Vector3d(q.x, q.y, q.z),
                    2.0 * scalar * Math.acos(q.w));
        } else {
            return new Rotation3d(
                    new Vector3d(-q.x, -q.y, -q.z),
                    2.0 * scalar * Math.acos(-q.w));
        }
    }

    public Rotation3d div(double scalar) {
        return times(1.0 / scalar);
    }

    public Rotation3d rotateBy(Rotation3d other) {
        return new Rotation3d(other.q.multiply(q, 0));
    }

    public double toPose2d() {
        return getZ();
    }

    public double getX() {
        return MathEx.quaternionToEuler(q).getRoll();
    }

    public double getY() {
        return MathEx.quaternionToEuler(q).getPitch();
    }

    public double getZ() {
        return MathEx.quaternionToEuler(q).getYaw();
    }

    public Vector3d getAxis() {
        double norm = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        if (norm == 0.0) {
            return new Vector3d(0, 0, 0);
        } else {
            return new Vector3d(q.z / norm, q.y / norm, q.z / norm);
        }
    }

    public double getAngle() {
        double norm = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        return (2.0 * Math.atan2(norm, q.w));
    }

    public Quaternion getQ() {
        return q;
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("RPY %6.1f %6.1f %6.1f  (deg)", Math.toDegrees(getX()), Math.toDegrees(getY()), Math.toDegrees(getZ()));
    }
}
