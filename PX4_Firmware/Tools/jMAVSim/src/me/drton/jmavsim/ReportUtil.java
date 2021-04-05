package me.drton.jmavsim;

import javax.vecmath.Vector3d;

/**
 * A class containing helper methods for producing simulation reports.
 */
public final class ReportUtil {
    public static final String ff = "%+010.5f; ";
//    private static final DecimalFormat df = new DecimalFormat("0.#####", DecimalFormatSymbols.getInstance(Locale.ENGLISH));

    private ReportUtil() {
    }

    /**
     * Converts a vector to a shorter string.
     * When numbers are changing quickly on the screen, there may not be time to read the E notation.
     */
    public static String vector2str(Vector3d vec) {
        return String.format(ff + ff + ff, vec.x, vec.y, vec.z);
    }

    /**
     * Converts a double to a string.
     */
    public static String d2str(double val) {
        return String.format(ff, val);
    }

    /**
     * Converts a vector of radians to a vector of degrees
     */
    public static Vector3d vectRad2Deg(Vector3d v) {
        return new Vector3d(Math.toDegrees(v.x), Math.toDegrees(v.y), Math.toDegrees(v.z));
    }
}
