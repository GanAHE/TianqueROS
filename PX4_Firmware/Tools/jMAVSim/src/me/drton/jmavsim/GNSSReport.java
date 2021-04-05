package me.drton.jmavsim;

import me.drton.jmavlib.geo.LatLonAlt;

import javax.vecmath.Vector3d;

/**
 * GNSS Report.
 */
public class GNSSReport {
    public LatLonAlt position;
    public float eph;
    public float epv;
    public Vector3d velocity;
    public int fix;     // 0 = no fix, 1 = time only, 2 = 2D fix, 3 = 3D fix
    public long time;   // UTC time in [us]

    /**
     * Get scalar horizontal speed.
     *
     * @return horizontal ground speed in [m/s]
     */
    public double getSpeed() {
        return Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    }

    /**
     * Get horizontal course over ground.
     *
     * @return horizontal course over ground in [rad]
     */
    public double getCog() {
        return Math.atan2(velocity.y, velocity.x);
    }
}
