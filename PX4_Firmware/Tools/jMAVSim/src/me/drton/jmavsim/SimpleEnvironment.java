package me.drton.jmavsim;

import javax.vecmath.Vector3d;

import java.util.Random;

/**
 * User: ton Date: 28.11.13 Time: 22:40
 */
public class SimpleEnvironment extends Environment {
    public static final double Pb = 101325.0;  // static pressure at sea level [Pa]
    public static final double Tb = 288.15;    // standard temperature at sea level [K]
    public static final double Lb = -0.0065;   // standard temperature lapse rate [K/m]
    public static final double M = 0.0289644;  // molar mass of Earth's air [kg/mol]
    public static final double G = 9.80665;    // gravity
    public static final double R = 8.31432;    // universal gas constant

    private Random random = new Random();
    private long lastTime = 0;

    public SimpleEnvironment(World world) {
        super(world);
        setG(null);
        setMagField(new Vector3d(0.21523, 0.0, 0.42741));
    }

    @Override
    public void setG(Vector3d grav) {
        if (grav == null) {
            g = new Vector3d(0.0, 0.0, G);
        } else {
            g = grav;
        }
    }

    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }
        double dt = lastTime == 0 ? 0.0 : (t - lastTime) / 1000.0;
        lastTime = t;
        Vector3d r = new Vector3d(windDeviation);
        r.scale(random.nextGaussian());
        Vector3d dev = new Vector3d(wind);
        dev.sub(windCurrent);
        dev.scale(1.0 / windT);
        r.add(dev);
        r.scale(dt);
        windCurrent.add(r);
    }


    /**
     * Convert altitude to barometric pressure
     *
     * @param alt        Altitude in meters
     *
     * @return Barometric pressure in Pa
     */
    public static double alt2baro(double alt) {
        if (alt <= 11000.0) {
            return Pb * Math.pow(Tb / (Tb + (Lb * alt)), (G * M) / (R * Lb));
        } else if (alt <= 20000.0) {
            double f = 11000.0;
            double a = alt2baro(f);
            double c = Tb + (f * Lb);
            return a * Math.exp(((-G) * M * (alt - f)) / (R * c));
        }
        return 0.0;
    }


    // Mag declination calculator

    /** set this always to the sampling in degrees for the table below */
    private int SAMPLING_RES      = 10;
    private int SAMPLING_MIN_LAT  = -60;
    private int SAMPLING_MAX_LAT  = 60;
    private int SAMPLING_MIN_LON  = -180;
    private int SAMPLING_MAX_LON  = 180;

    private int[][] declination_table = {
        { 46, 45, 44, 42, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -40, -48, -55, -61, -66, -71, -74, -75, -72, -61, -25, 22, 40, 45, 47, 46 },
        { 30, 30, 30, 30, 29, 29, 29, 29, 27, 24, 18, 11, 3, -3, -9, -12, -15, -17, -21, -26, -32, -39, -45, -51, -55, -57, -56, -53, -44, -31, -14, 0, 13, 21, 26, 29, 30 },
        { 21, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, -40, -44, -45, -44, -40, -32, -22, -12, -3, 3, 9, 14, 18, 20, 21 },
        { 16, 17, 17, 17, 17, 17, 16, 16, 16, 13, 8, 0, -9, -16, -21, -24, -25, -25, -23, -20, -21, -24, -28, -31, -31, -29, -24, -17, -9, -3, 0, 4, 7, 10, 13, 15, 16 },
        { 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -12, -19, -23, -24, -24, -22, -17, -12, -9, -10, -13, -17, -18, -16, -13, -8, -3, 0, 1, 3, 6, 8, 10, 12, 12 },
        { 10, 10, 10, 10, 10, 10, 10, 9, 9, 6, 0, -6, -14, -20, -22, -22, -19, -15, -10, -6, -2, -2, -4, -7, -8, -8, -7, -4, 0, 1, 1, 2, 4, 6, 8, 10, 10 },
        { 9, 9, 9, 9, 9, 9, 8, 8, 7, 4, -1, -8, -15, -19, -20, -18, -14, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 3, 5, 7, 8, 9 },
        { 8, 8, 8, 9, 9, 9, 8, 8, 6, 2, -3, -9, -15, -18, -17, -14, -10, -6, -2, 0, 1, 2, 2, 0, -1, -1, -2, -1, 0, 0, 0, 0, 1, 3, 5, 7, 8 },
        { 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -5, -11, -15, -16, -15, -12, -8, -4, -1, 0, 2, 3, 2, 1, 0, 0, 0, 0, 0, -1, -2, -2, -1, 0, 3, 6, 8 },
        { 6, 9, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -7, -3, 0, 1, 2, 3, 3, 3, 2, 1, 0, 0, -1, -3, -4, -5, -5, -2, 0, 3, 6 },
        { 5, 8, 11, 13, 15, 15, 14, 11, 5, -1, -9, -14, -17, -16, -14, -11, -7, -3, 0, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -4, -7, -8, -8, -6, -2, 1, 5 },
        { 4, 8, 12, 15, 17, 18, 16, 12, 5, -3, -12, -18, -20, -19, -16, -13, -8, -4, -1, 1, 4, 6, 8, 9, 9, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, 0, 4 },
        { 3, 9, 14, 17, 20, 21, 19, 14, 4, -8, -19, -25, -26, -25, -21, -17, -12, -7, -2, 1, 5, 9, 13, 15, 16, 16, 13, 7, 0, -7, -12, -15, -14, -11, -6, -1, 3 },
    };

    private double get_lookup_table_val(int lat_index, int lon_index) {
        return declination_table[lat_index][lon_index];
    }

    /**
     * Get the mag declination at this point
     *
     * @param lat latitude in degrees
     * @param lon longitude in degrees
     * @return mag declination in degrees
     */
    public double getMagDeclination(double lat, double lon) {
        /*
         * If the values exceed valid ranges, return zero as default
         * as we have no way of knowing what the closest real value
         * would be.
         */
        if (lat < -90.0f || lat > 90.0f ||
                lon < -180.0f || lon > 180.0f) {
            return 0.0f;
        }

        /* round down to nearest sampling resolution */
        int min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES;
        int min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES;

        /* for the rare case of hitting the bounds exactly
         * the rounding logic wouldn't fit, so enforce it.
         */

        /* limit to table bounds - required for maxima even when table spans full globe range */
        if (lat <= SAMPLING_MIN_LAT) {
            min_lat = SAMPLING_MIN_LAT;
        }

        if (lat >= SAMPLING_MAX_LAT) {
            min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
        }

        if (lon <= SAMPLING_MIN_LON) {
            min_lon = SAMPLING_MIN_LON;
        }

        if (lon >= SAMPLING_MAX_LON) {
            min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
        }

        /* find index of nearest low sampling point */
        int min_lat_index = (-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES;
        int min_lon_index = (-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES;

        double declination_sw = get_lookup_table_val(min_lat_index, min_lon_index);
        double declination_se = get_lookup_table_val(min_lat_index, min_lon_index + 1);
        double declination_ne = get_lookup_table_val(min_lat_index + 1, min_lon_index + 1);
        double declination_nw = get_lookup_table_val(min_lat_index + 1, min_lon_index);

        /* perform bilinear interpolation on the four grid corners */

        double declination_min = ((lon - min_lon) / SAMPLING_RES) * (declination_se - declination_sw) +
                                 declination_sw;
        double declination_max = ((lon - min_lon) / SAMPLING_RES) * (declination_ne - declination_nw) +
                                 declination_nw;

        return ((lat - min_lat) / SAMPLING_RES) * (declination_max - declination_min) + declination_min;
    }

}
