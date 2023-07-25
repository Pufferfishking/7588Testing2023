package org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry;

public class Pose3d {

    private final Vector3d vector3d;
    private final Rotation3d rotation3d;

    public Pose3d() {
        vector3d = new Vector3d();
        rotation3d = new Rotation3d();
    }

    public Pose3d(Vector3d vector3d, Rotation3d rotation3d) {
        this.vector3d = vector3d;
        this.rotation3d = rotation3d;
    }

    public Pose3d(double x, double y, double z, Rotation3d rotation) {
        vector3d = new Vector3d(x, y, z);
        rotation3d = rotation;
    }

    public Pose3d plus(Transform3d other) {
        return transformBy(other);
    }

    public Transform3d minus (Pose3d other) {
        final Pose3d pose = this.relativeTo(other);
        return new Transform3d(pose.getVector(), pose.getRotation());
    }

    public Pose3d times(double scalar) {
        return new Pose3d(vector3d.times(scalar), rotation3d.times(scalar));
    }

    public Pose3d div(double scalar) {
        return times(1.0 / scalar);
    }

    public Pose3d transformBy(Transform3d other) {
        return new Pose3d(
                vector3d.plus(other.getVector().rotateBy(rotation3d)),
                other.getRotation().plus(rotation3d)
        );
    }

    public Pose3d relativeTo(Pose3d other) {
        Transform3d transform = new Transform3d(other, this);
        return new Pose3d(transform.getVector(), transform.getRotation());
    }

    public Pose2d toPose2d() {
        return new Pose2d(vector3d.toVector2d(), rotation3d.getZ());
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
        return String.format("Pose3d:\n%s\n%s", vector3d, rotation3d);
    }
}

