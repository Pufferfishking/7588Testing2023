package org.firstinspires.ftc.teamcode.commandBased.classes.util;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class MathEx {

    public static Quaternion eulerToQuaternion(EulerAngles euler) {
        double cr = Math.cos(euler.roll * 0.5);
        double sr = Math.sin(euler.roll * 0.5);
        double cp = Math.cos(euler.pitch * 0.5);
        double sp = Math.sin(euler.pitch * 0.5);
        double cy = Math.cos(euler.yaw * 0.5);
        double sy = Math.sin(euler.yaw * 0.5);

        float w = (float) (cr * cp * cy + sr * sp * sy);
        float x = (float) (sr * cp * cy - cr * sp * sy);
        float y = (float) (cr * sp * cy + sr * cp * sy);
        float z = (float) (cr * cp * sy - sr * sp * cy);

        return new Quaternion(w, x, y, z, 0);
    }

    public static Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
        return eulerToQuaternion(new EulerAngles(roll, pitch, yaw));
    }

    public static EulerAngles quaternionToEuler(Quaternion q) {

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        double roll = Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = Math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
        double cosp = Math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
        double pitch = 2 * Math.atan2(sinp, cosp) - Math.PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = Math.atan2(siny_cosp, cosy_cosp);

        return new EulerAngles(roll, pitch, yaw);
    }
}
