package me.drton.jmavlib.conversion;

import javax.vecmath.Vector3d;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * User: ton Date: 02.06.13 Time: 20:20
 */
public class RotationConversion {
    public static double[] rotationMatrixByEulerAngles(double roll, double pitch, double yaw) {
        return new double[] {
                   cos(pitch) * cos(yaw),
                   sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw),
                   cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw),
                   cos(pitch) * sin(yaw),
                   sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw),
                   cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw),
                   -sin(pitch),
                   sin(roll) * cos(pitch),
                   cos(roll) * cos(pitch)
               };
    }

    public static double[] eulerAnglesByQuaternion(double[] q) {
        return new double[] {
                   Math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])),
                   Math.asin(2 * (q[0] * q[2] - q[3] * q[1])),
                   Math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])),
               };
    }

    public static Float[] quaternionByEulerAngles(Vector3d euler) {
        double cosPhi_2 = Math.cos(euler.x / 2.0);
        double cosTheta_2 = Math.cos(euler.y / 2.0);
        double cosPsi_2 = Math.cos(euler.z / 2.0);
        double sinPhi_2 = Math.sin(euler.x / 2.0);
        double sinTheta_2 = Math.sin(euler.y / 2.0);
        double sinPsi_2 = Math.sin(euler.z / 2.0);
        return new Float[] {
                   (float)(cosPhi_2 * cosTheta_2 * cosPsi_2 +
                           sinPhi_2 * sinTheta_2 * sinPsi_2),
                   (float)(sinPhi_2 * cosTheta_2 * cosPsi_2 -
                           cosPhi_2 * sinTheta_2 * sinPsi_2),
                   (float)(cosPhi_2 * sinTheta_2 * cosPsi_2 +
                           sinPhi_2 * cosTheta_2 * sinPsi_2),
                   (float)(cosPhi_2 * cosTheta_2 * sinPsi_2 -
                           sinPhi_2 * sinTheta_2 * cosPsi_2)
               };
    }

}
