package org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry;

public class Transform3d {
    private final Vector3d vector3d;
    private final Rotation3d rotation3d;

    public Transform3d(Pose3d initial, Pose3d last) {
        vector3d = last.getVector().minus(initial.getVector()).rotateBy(initial.getRotation().unaryMinus());
        rotation3d = last.getRotation().minus(initial.getRotation());
    }

    public Transform3d(Vector3d vector, Rotation3d rotation) {
        vector3d = vector;
        rotation3d = rotation;
    }

    public Transform3d times(double scalar) {
        return new Transform3d(vector3d.times(scalar), rotation3d.times(scalar));
    }

    public Transform3d div(double scalar) {
        return times(1.0 / scalar);
    }

    public Transform3d plus(Transform3d other) {
        return new Transform3d(new Pose3d(), new Pose3d().transformBy(this).transformBy(other));
    }

    public Transform3d inverse() {
        return new Transform3d(
                getVector().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus()
        );
    }

    public Vector3d getVector() {
        return vector3d;
    }

    public double getX() {
        return vector3d.getX();
    }

    public double getY() {
        return vector3d.getY();
    }

    public double getZ() {
        return vector3d.getZ();
    }

    public Rotation3d getRotation() {
        return rotation3d;
    }

    public String toString() {
        return String.format("Transform3d:\n%s\n%s", vector3d, rotation3d);
    }

}
