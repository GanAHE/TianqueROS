package me.drton.jmavsim;

import me.drton.jmavlib.conversion.RotationConversion;
import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.vecmath.*;
import java.util.Arrays;

/**
 * MAVLinkDisplayOnly is MAVLink bridge between AbstractVehicle and autopilot connected via MAVLink.
 * MAVLinkDisplayOnly should have the same sysID as the autopilot, but different componentId.
 * It reads HIL_STATE_QUATERNION from the MAVLink and displays the vehicle position and attitude.
 * @author Romain Chiappinelli
 */
public class MAVLinkDisplayOnly extends MAVLinkHILSystemBase {

    private boolean firstMsg=true;       // to detect the first MAVLink message
    private double lat0;                // initial latitude (radians)
    private double lon0;                // initial longitude (radians)
    private double alt0;                // initial altitude (meters)
    private double lat;                 // geodetic latitude (radians)
    private double lon;                 // geodetic longitude (radians)
    private double alt;                  // above sea level (meters)
    private double [] quat={0.0,0.0,0.0,0.0};   // unit quaternion for attitude representation
    private Double [] control = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    private static final double EARTH_RADIUS=6371000.0;    // earth radius in meters

    /**
     * Create MAVLinkDisplayOnly, MAVLink system that sends nothing to autopilot and passes states from
     * autopilot to simulator
     *
     * @param sysId       SysId of simulator should be the same as autopilot
     * @param componentId ComponentId of simulator should be different from autopilot
     * @param vehicle     vehicle to connect
     */
    public MAVLinkDisplayOnly(MAVLinkSchema schema, int sysId, int componentId, AbstractVehicle vehicle) {
        super(schema, sysId, componentId, vehicle);
    }

    @Override
    public void handleMessage(MAVLinkMessage msg) {
        if ("HIL_STATE_QUATERNION".equals(msg.getMsgName())) {

            if (firstMsg) {
                firstMsg=false;
                // we take the first received position as initial position
                lat0=Math.toRadians(msg.getDouble("lat")*1e-7);
                lon0=Math.toRadians(msg.getDouble("lon")*1e-7);
                alt0=msg.getDouble("alt")*1e-3;
            }
            for (int i = 0; i < 4; ++i) {
                quat[i] = ((Number)((Object[])msg.get("attitude_quaternion"))[i]).doubleValue();
            }        
            lat=Math.toRadians(msg.getDouble("lat")*1e-7);
            lon=Math.toRadians(msg.getDouble("lon")*1e-7);
            alt=msg.getDouble("alt")*1e-3;
        
            Vector3d pos = new Vector3d(EARTH_RADIUS*(lat-lat0),EARTH_RADIUS*(lon-lon0)*Math.cos(lat0),alt0-alt);
            double [] euler = RotationConversion.eulerAnglesByQuaternion(quat);
            Matrix3d dcm = new Matrix3d(RotationConversion.rotationMatrixByEulerAngles(euler[0],euler[1],euler[2]));   

            vehicle.setControl(Arrays.asList(control));     // set 0 throttles
            vehicle.setPosition(pos);   // we want ideally a "local" pos groundtruth
            vehicle.setRotation(dcm); 
        }
    }

    @Override
    public boolean gotHilActuatorControls()
    {
        return !firstMsg;
    }

    @Override
    public void initMavLink()
    {
        firstMsg=true;
    }

    @Override
    public void endSim()
    {

    }

}
