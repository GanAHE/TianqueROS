package me.drton.jmavsim;

import com.sun.j3d.utils.geometry.Sphere;
import me.drton.jmavlib.geo.GlobalPositionProjector;
import me.drton.jmavlib.geo.LatLonAlt;

import javax.vecmath.Vector3d;
import java.io.FileNotFoundException;

/**
 * User: ton Date: 01.02.14 Time: 22:12
 */
public abstract class Target extends KinematicObject {
    private GlobalPositionProjector gpsProjector = new GlobalPositionProjector();

    public Target(World world, double size, boolean showGui) throws FileNotFoundException {
        super(world, showGui);
        Sphere sphere = new Sphere((float) size);
        transformGroup.addChild(sphere);
        gpsProjector.init(world.getGlobalReference());
    }

    public GNSSReport getGlobalPosition() {
        Vector3d pos = getPosition();
        LatLonAlt latLonAlt = gpsProjector.reproject(new double[] {pos.x, pos.y, pos.z});
        GNSSReport gps = new GNSSReport();
        gps.position = latLonAlt;
        gps.eph = 1.0f;
        gps.epv = 1.0f;
        gps.velocity = getVelocity();
        return gps;
    }
}
