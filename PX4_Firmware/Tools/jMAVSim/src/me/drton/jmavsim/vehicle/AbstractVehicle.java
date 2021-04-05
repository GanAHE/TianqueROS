package me.drton.jmavsim.vehicle;

import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavsim.DynamicObject;
import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.ReportingObject;
import me.drton.jmavsim.Sensors;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Abstract vehicle class, should be used for creating vehicle of any type.
 * Child class should use member 'control' as control input for actuators.
 * 'update()' method of AbstractVehicle must be called from child class implementation if overridden.
 */
public abstract class AbstractVehicle extends DynamicObject implements ReportingObject {
    protected List<Double> control = Collections.emptyList();
    protected Sensors sensors = null;

    public AbstractVehicle(World world, String modelName, boolean showGui) {
        super(world, showGui);
        if (showGui) {
            modelFromFile(modelName);
        }
        resetObjectParameters();
    }

    public void report(StringBuilder builder) {
        Vector3d tv = new Vector3d();
        builder.append("VEHICLE");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);

        builder.append("Pos: ");
        builder.append(ReportUtil.vector2str(position));
//        builder.append(String.format("  X: %.5f Y: %.5f Z: %.5f", position.x, position.y, position.z));
        builder.append(newLine);

        builder.append("Vel: ");
        builder.append(ReportUtil.vector2str(velocity));
        builder.append(newLine);

        builder.append("Acc: ");
        builder.append(ReportUtil.vector2str(acceleration));
        builder.append(newLine);

        builder.append("Rot: ");
        builder.append(ReportUtil.vector2str(rotationRate));
        builder.append(newLine);

        builder.append("Att: ");
        builder.append(ReportUtil.vector2str(ReportUtil.vectRad2Deg(attitude)));
        builder.append(newLine);
        builder.append(newLine);

        if (sensors != null) {
            builder.append("SENSORS");
            builder.append(newLine);
            builder.append("--------");
            builder.append(newLine);

            tv = sensors.getAcc();
            builder.append("ACC: ");
            builder.append(ReportUtil.vector2str(tv));
            builder.append(newLine);
            builder.append(String.format("    Magnitude: %s;",
                                         ReportUtil.d2str(Math.sqrt(tv.x * tv.x + tv.y * tv.y + tv.z * tv.z))));
            builder.append(newLine);
            builder.append(String.format("    P: %s; R: %s", ReportUtil.d2str(Math.toDegrees(Math.atan2(tv.x,
                                                                              -tv.z))), ReportUtil.d2str(Math.toDegrees(Math.atan2(-tv.y, -tv.z)))));
            builder.append(newLine + newLine);

            tv = sensors.getGyro();
            builder.append("GYO: ");
            builder.append(ReportUtil.vector2str(tv));
            builder.append(newLine);
            builder.append(String.format("    Magnitude: %s;",
                                         ReportUtil.d2str(Math.sqrt(tv.x * tv.x + tv.y * tv.y + tv.z * tv.z))));
            builder.append(newLine + newLine);

            tv = sensors.getMag();
            builder.append("MAG: ");
            builder.append(ReportUtil.vector2str(tv));
            builder.append(newLine);
            builder.append(String.format("    Magnitude: %s;",
                                         ReportUtil.d2str(Math.sqrt(tv.x * tv.x + tv.y * tv.y + tv.z * tv.z))));
            builder.append(newLine + newLine);

            LatLonAlt pos;
            if (sensors.getGNSS() != null && sensors.getGNSS().position != null) {
                pos = sensors.getGNSS().position;
            } else {
                pos = sensors.getGlobalPosition();
            }
            builder.append(String.format("GPS Lat: %+013.8f;\n    Lon: %+013.8f\n    Alt: %07.3f", pos.lat,
                                         pos.lon, pos.alt));
            builder.append(newLine);
            builder.append(String.format("Baro Alt: %07.3f; Pa: %08.2f", sensors.getPressureAlt(),
                                         sensors.getPressure()));
            builder.append(newLine + newLine);
        }

    }

    public void setControl(List<Double> control) {
        this.control = new ArrayList<Double>(control);
    }

    public List<Double> getControl() {
        return control;
    }

    /**
     * Set sensors object for the vehicle.
     *
     * @param sensors
     */
    public void setSensors(Sensors sensors, long t) {
        this.sensors = sensors;
        sensors.setObject(this, t);
    }

    public Sensors getSensors() {
        return sensors;
    }

    @Override
    public void resetObjectParameters() {
        super.resetObjectParameters();
        position.set(0.0, 0.0, 0.0);
        if (sensors != null) {
            sensors.setReset(true);
        }
    }

    @Override
    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }
        super.update(t, paused);
        if (sensors != null) {
            sensors.update(t, paused);
        }
    }
}
