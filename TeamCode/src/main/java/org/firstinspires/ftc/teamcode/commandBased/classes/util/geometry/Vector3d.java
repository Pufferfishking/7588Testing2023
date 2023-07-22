package org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class Vector3d {

    private final double x;
    private final double y;
    private final double z;

    public Vector3d() {
        this(0, 0, 0);
    }

    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3d(double distance, Rotation3d angle) {
        final Vector3d rectangular = new Vector3d(distance, 0, 0).rotateBy(angle);
        x = rectangular.getX();
        y = rectangular.getY();
        z = rectangular.getZ();
    }

    public Vector3d(VectorF vectorF) {
        x = vectorF.get(0);
        y = vectorF.get(1);
        z = vectorF.get(2);
    }

    public Vector3d plus(Vector3d other) {
        return new Vector3d(x + other.getX(), y + other.getY(), z + other.getZ());
    }

    public Vector3d minus(Vector3d other) {
        return new Vector3d(x - other.getX(), y - other.getY(), z - other.getZ());
    }

    public Vector3d unaryMinus() {
        return new Vector3d(-x, -y, -z);
    }

    public Vector3d times(double scalar) {
        return new Vector3d(x * scalar, y * scalar, z * scalar);
    }

    public Vector3d div(double scalar) {
        return new Vector3d(x / scalar, y / scalar, z / scalar);
    }

    public Vector3d rotateBy(double pitchAngle, double rollAngle, double yawAngle) {
        Vector3d rotated = new Vector3d(x, y, z);
        rotated = rotated.rotateByPitch(pitchAngle);
        rotated = rotated.rotateByRoll(rollAngle);
        rotated = rotated.rotateByYaw(yawAngle);
        return rotated;
    }

    public Vector3d rotateByPitch(double pitchAngle) {
        Vector2d rotated = new Vector2d(y, z);
        rotated = rotated.rotateBy(pitchAngle);
        return new Vector3d(x, rotated.getX(), rotated.getY());
    }

    public Vector3d rotateByRoll(double rollAngle) {
        Vector2d rotated = new Vector2d(x, z);
        rotated = rotated.rotateBy(rollAngle);
        return new Vector3d(rotated.getX(), y, rotated.getY());
    }

    public Vector3d rotateByYaw(double yawAngle) {
        Vector2d rotated = new Vector2d(x, z);
        rotated = rotated.rotateBy(yawAngle);
        return new Vector3d(rotated.getX(), rotated.getY(), z);
    }

    public Vector3d normalize() {
        return scale(1/getMagnitude());
    }

    public Vector3d scale(double scaleFactor) {
        return new Vector3d(x * scaleFactor, y * scaleFactor, z * scaleFactor);
    }

    public Vector3d rotateBy(Rotation3d other) {
        final Quaternion p = new Quaternion(0, (float) x, (float) y, (float) z, 1);
        final Quaternion qprime = other.getQ().multiply(p, 0).multiply(other.getQ().inverse(), 0);
        return new Vector3d(qprime.x, qprime.y, qprime.z);
    }

    public Vector2d toVector2d() {
        return new Vector2d(x, y);
    }

    public double getDistance(Vector3d other) {
        return Math.sqrt(Math.pow(other.getX() - x, 2) + Math.pow(other.getY() - y, 2) + Math.pow(other.getZ() - z, 2));
    }

    public double getMagnitude() {
        return Math.sqrt((Math.pow(x, 2)) + (Math.pow(y, 2)) + (Math.pow(z, 2)));
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public String toString() {
        return String.format("XYZ %6.1f %6.1f %6.1f", x, y, z);
    }
}
