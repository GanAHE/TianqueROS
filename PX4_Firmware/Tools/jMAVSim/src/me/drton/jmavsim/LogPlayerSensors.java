package me.drton.jmavsim;

import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavlib.log.FormatErrorException;
import me.drton.jmavlib.log.LogReader;
import me.drton.jmavlib.log.px4.PX4LogReader;

import javax.vecmath.Vector3d;
import java.io.EOFException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Sensors object that uses PX4 log file replay as source.
 */
public class LogPlayerSensors implements Sensors {
    private LogReader logReader = null;
    private long logStart = 0;
    private long logT = 0;
    private Vector3d acc = new Vector3d();
    private Vector3d gyro = new Vector3d();
    private Vector3d mag = new Vector3d();
    private double baroAlt;
    private GNSSReport gnss = new GNSSReport();
    private LatLonAlt globalPosition = new LatLonAlt(0, 0, 0);
    private boolean gpsUpdated = false;
    private boolean reset = false;

    void openLog(String fileName, long startTime) throws IOException, FormatErrorException {
        logReader = new PX4LogReader(fileName);
        logStart = startTime - logReader.getStartMicroseconds() / 1000;
    }

    @Override
    public void setObject(DynamicObject object, long t) {
    }

    @Override
    public void setGPSStartTime(long time) {}

    @Override
    public long getGPSStartTime() {
        return 0;
    }

    @Override
    public Vector3d getAcc() {
        return acc;
    }

    @Override
    public Vector3d getGyro() {
        return gyro;
    }

    @Override
    public Vector3d getMag() {
        return mag;
    }

    @Override
    public double getPressureAlt() {
        return baroAlt;
    }

    @Override
    public double getPressure() {
        return SimpleEnvironment.alt2baro(getPressureAlt());
    }

    @Override
    public GNSSReport getGNSS() {
        return gnss;
    }

    @Override
    public LatLonAlt getGlobalPosition() {
        return globalPosition;
    }

    @Override
    public boolean isGPSUpdated() {
        boolean res = gpsUpdated;
        gpsUpdated = false;
        return res;
    }

    @Override
    public void update(long t, boolean paused) {
        if (logReader != null) {
            Map<String, Object> logData = new HashMap<String, Object>();
            while (logStart + logT < t) {
                try {
                    logT = logReader.readUpdate(logData) / 1000;
                } catch (EOFException e) {
                    break;
                } catch (IOException e) {
                    e.printStackTrace();
                    break;
                } catch (FormatErrorException e) {
                    e.printStackTrace();
                    break;
                }
            }
            if (logData.containsKey("IMU.AccX") &&
                    logData.containsKey("IMU.AccY") &&
                    logData.containsKey("IMU.AccZ")) {
                acc.set((Float) logData.get("IMU.AccX"), (Float) logData.get("IMU.AccY"),
                        (Float) logData.get("IMU.AccZ"));
            }
            if (logData.containsKey("IMU.GyroX") &&
                    logData.containsKey("IMU.GyroY") &&
                    logData.containsKey("IMU.GyroZ")) {
                gyro.set((Float) logData.get("IMU.GyroX"), (Float) logData.get("IMU.GyroY"),
                         (Float) logData.get("IMU.GyroZ"));
            }
            if (logData.containsKey("IMU.MagX") &&
                    logData.containsKey("IMU.MagY") &&
                    logData.containsKey("IMU.MagZ")) {
                mag.set((Float) logData.get("IMU.MagX"), (Float) logData.get("IMU.MagY"),
                        (Float) logData.get("IMU.MagZ"));
            }
            if (logData.containsKey("SENS.BaroAlt")) {
                baroAlt = (Float) logData.get("SENS.BaroAlt");
            }
            if (logData.containsKey("GPS.Lat") &&
                    logData.containsKey("GPS.Lon") &&
                    logData.containsKey("GPS.Alt")) {
                gpsUpdated = true;
                gnss.position = new LatLonAlt(((Number) logData.get("GPS.Lat")).doubleValue(),
                                              ((Number) logData.get("GPS.Lon")).doubleValue(),
                                              ((Number) logData.get("GPS.Alt")).doubleValue());
                gnss.eph = ((Number) logData.get("GPS.EPH")).floatValue();
                gnss.epv = ((Number) logData.get("GPS.EPV")).floatValue();
                gnss.velocity = new Vector3d(((Number) logData.get("GPS.VelN")).doubleValue(),
                                             ((Number) logData.get("GPS.VelE")).doubleValue(),
                                             ((Number) logData.get("GPS.VelD")).doubleValue());
                gnss.fix = (Integer) logData.get("GPS.Fix");
                gnss.time = (Long) logData.get("GPS.GPSTime");

                globalPosition = gnss.position;
            }
        }
    }

    @Override
    public boolean isReset() {
        return reset;
    }

    @Override
    public void setReset(boolean reset) {
        this.reset = reset;
    }

    @Override
    public void setParameter(String name, float value) {}

    @Override
    public float param(String name) {
        return 0.0f;
    }

}
