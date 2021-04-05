package me.drton.jmavsim;

import me.drton.jmavlib.conversion.RotationConversion;
import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.vecmath.*;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.TimeZone;

/**
 * MAVLinkHILSystem is MAVLink bridge between AbstractVehicle and autopilot connected via MAVLink.
 * MAVLinkHILSystem should have the same sysID as the autopilot, but different componentId.
 */
public class MAVLinkHILSystem extends MAVLinkHILSystemBase {
    // private Simulator simulator;
    // private AbstractVehicle vehicle;
    private boolean gotHeartBeat = false;
    private boolean inited = false;
    private boolean stopped = false;
    private boolean gotHilActuatorControls = false;
    private long hilStateUpdateInterval = -1; //don't publish by default
    private long nextHilStatePub = 0;
    private long timeThrottleCounter = 0;
    private long lastHeartbeatMs = 0;

    /**
     * Create MAVLinkHILSimulator, MAVLink system that sends simulated sensors to autopilot and passes controls from
     * autopilot to simulator
     *
     * @param sysId       SysId of simulator should be the same as autopilot
     * @param componentId ComponentId of simulator should be different from autopilot
     * @param vehicle     vehicle to connect
     */
    public MAVLinkHILSystem(MAVLinkSchema schema, int sysId, int componentId, AbstractVehicle vehicle) {
        super(schema, sysId, componentId, vehicle);
    }

    @Override
    public boolean gotHilActuatorControls() {
        return gotHilActuatorControls;
    }

    @Override
    public void handleMessage(MAVLinkMessage msg) {
        super.handleMessage(msg);
        long t = simulator.getSimMillis();
        if ("HIL_ACTUATOR_CONTROLS".equals(msg.getMsgName())) {
            gotHilActuatorControls = true;
            List<Double> control = new ArrayList<Double>();
            for (int i = 0; i < 8; ++i) {
                control.add(((Number)((Object[])msg.get("controls"))[i]).doubleValue());
            }

            // Get the system arming state if the mode
            // field is valid
            int mode = msg.getInt("mode");
            boolean armed = true;

            if (mode != 0) {
                if ((mode & 128) > 0 /* armed */) {
                    armed = true;
                } else {
                    armed = false;
                }
            }

            simulator.advanceTime();

            vehicle.setControl(control);

        } else if ("HIL_CONTROLS".equals(msg.getMsgName()) &&
                   !gotHilActuatorControls) { //this is deprecated, but we still support it for now
            List<Double> control = Arrays.asList(msg.getDouble("roll_ailerons"),
                                                 msg.getDouble("pitch_elevator"),
                                                 msg.getDouble("yaw_rudder"), msg.getDouble("throttle"), msg.getDouble("aux1"),
                                                 msg.getDouble("aux2"), msg.getDouble("aux3"), msg.getDouble("aux4"));

            // Get the system arming state if the mode
            // field is valid
            int mode = msg.getInt("mode");
            boolean armed = true;

            if (mode != 0) {
                if ((mode & 128) > 0 /* armed */) {
                    armed = true;
                } else {
                    armed = false;
                }
            }

            vehicle.setControl(control);

        } else if ("COMMAND_LONG".equals(msg.getMsgName())) {
            int command = msg.getInt("command");
            if (command == 511) { //MAV_CMD_SET_MESSAGE_INTERVAL
                int msg_id = (int)(msg.getFloat("param1") + 0.5);
                if (msg_id == 115) { //HIL_STATE_QUATERNION
                    hilStateUpdateInterval = (int)(msg.getFloat("param2") + 0.5);
                }
            }
        } else if ("HEARTBEAT".equals(msg.getMsgName())) {
            long realMs = simulator.getRealMillis();

            // We timeout after 3 seconds and do a reset.
            long diffMs = realMs - lastHeartbeatMs;
            if (diffMs > 3000) {
                if (gotHeartBeat) {
                    System.out.println("Reseting after silence of " + diffMs + " ms");
                }
                reset();
            }

            if (!gotHeartBeat && !stopped) {
                if (sysId < 0 || sysId == msg.systemID) {
                    gotHeartBeat = true;
                    if (sysId < 0) {
                        sysId = msg.systemID;
                    }

                    System.out.println("Init MAVLink");
                    initMavLink();

                } else if (sysId > -1 && sysId != msg.systemID) {
                    System.out.println("WARNING: Got heartbeat from system #" + Integer.toString(msg.systemID) +
                                       " but configured to only accept messages from system #" + Integer.toString(sysId) +
                                       ". Please change the system ID parameter to match in order to use HITL/SITL.");
                }
            }
            if ((msg.getInt("base_mode") & 128) == 0) {
                vehicle.setControl(Collections.<Double>emptyList());
            }

            lastHeartbeatMs = realMs;

        } else if ("STATUSTEXT".equals(msg.getMsgName())) {
            System.out.println("MSG: " + msg.getString("text"));
        }
    }

    @Override
    public void initMavLink() {
        if (vehicle.getSensors().getGPSStartTime() == -1) {
            vehicle.getSensors().setGPSStartTime(simulator.getSimMillis() + 1000);
        }
        stopped = false;
        inited = true;
    }

    @Override
    public void endSim() {
        if (!inited) {
            return;
        }
        inited = false;
        gotHeartBeat = false;
        stopped = true;
        vehicle.getSensors().setGPSStartTime(-1);
    }

    @Override
    public void update(long t, boolean paused) {
        super.update(t, paused);

        if (paused) {
            return;
        }

        long tu = t * 1000; // Time in us

        if (!this.inited) {
            return;
        }

        Sensors sensors = vehicle.getSensors();

        // Sensors
        MAVLinkMessage msg_sensor = new MAVLinkMessage(schema, "HIL_SENSOR", sysId, componentId,
                                                       protocolVersion);

        // sensor source bitmask
        int sensor_source = 0;

        msg_sensor.set("time_usec", tu);
        Vector3d tv = sensors.getAcc();
        msg_sensor.set("xacc", tv.x);
        msg_sensor.set("yacc", tv.y);
        msg_sensor.set("zacc", tv.z);
        tv = sensors.getGyro();
        sensor_source |= 0b111;
        msg_sensor.set("xgyro", tv.x);
        msg_sensor.set("ygyro", tv.y);
        msg_sensor.set("zgyro", tv.z);
        tv = sensors.getMag();
        sensor_source |= 0b111000;
        msg_sensor.set("xmag", tv.x);
        msg_sensor.set("ymag", tv.y);
        msg_sensor.set("zmag", tv.z);
        sensor_source |= 0b111000000;
        msg_sensor.set("pressure_alt", sensors.getPressureAlt());
        msg_sensor.set("abs_pressure", sensors.getPressure() * 0.01);  // Pa to millibar
        sensor_source |= 0b1101000000000;
        if (sensors.isReset()) {
            msg_sensor.set("fields_updated", (1 << 31));
            sensors.setReset(false);
        } else {
            msg_sensor.set("fields_updated", sensor_source);
        }
        sendMessage(msg_sensor);

        /* ground truth */
        if (hilStateUpdateInterval != -1 && nextHilStatePub <= tu) {
            MAVLinkMessage msg_hil_state = new MAVLinkMessage(schema, "HIL_STATE_QUATERNION", sysId,
                                                              componentId, protocolVersion);
            msg_hil_state.set("time_usec", tu);

            Float[] q = RotationConversion.quaternionByEulerAngles(vehicle.attitude);
            msg_hil_state.set("attitude_quaternion", q);

            Vector3d v3d = vehicle.getRotationRate();
            msg_hil_state.set("rollspeed", (float) v3d.x);
            msg_hil_state.set("pitchspeed", (float) v3d.y);
            msg_hil_state.set("yawspeed", (float) v3d.z);

            int alt = (int)(1000 * vehicle.position.z);
            msg_hil_state.set("alt", alt);
            msg_hil_state.set("lat", (int)(sensors.getGlobalPosition().lat * 1.e7));
            msg_hil_state.set("lon", (int)(sensors.getGlobalPosition().lon * 1.e7));

            v3d = vehicle.getVelocity();
            msg_hil_state.set("vx", (int)(v3d.x * 100));
            msg_hil_state.set("vy", (int)(v3d.y * 100));
            msg_hil_state.set("vz", (int)(v3d.z * 100));

            Vector3d airSpeed = new Vector3d(vehicle.getVelocity());
            airSpeed.scale(-1.0);
            airSpeed.add(vehicle.getWorld().getEnvironment().getCurrentWind(vehicle.position));
            float as_mag = (float) airSpeed.length();
            msg_hil_state.set("true_airspeed", (int)(as_mag * 100));

            v3d = vehicle.acceleration;
            msg_hil_state.set("xacc", (int)(v3d.x * 1000));
            msg_hil_state.set("yacc", (int)(v3d.y * 1000));
            msg_hil_state.set("zacc", (int)(v3d.z * 1000));

            sendMessage(msg_hil_state);
            nextHilStatePub = tu + hilStateUpdateInterval;
        }

        // GPS
        if (sensors.isGPSUpdated()) {
            GNSSReport gps = sensors.getGNSS();
            if (gps != null && gps.position != null && gps.velocity != null) {
                MAVLinkMessage msg_gps = new MAVLinkMessage(schema, "HIL_GPS", sysId, componentId, protocolVersion);
                msg_gps.set("time_usec", tu);
                msg_gps.set("lat", (long)(gps.position.lat * 1e7));
                msg_gps.set("lon", (long)(gps.position.lon * 1e7));
                msg_gps.set("alt", (long)(gps.position.alt * 1e3));
                msg_gps.set("vn", (int)(gps.velocity.x * 100));
                msg_gps.set("ve", (int)(gps.velocity.y * 100));
                msg_gps.set("vd", (int)(gps.velocity.z * 100));
                msg_gps.set("eph", (int)(gps.eph * 100f));
                msg_gps.set("epv", (int)(gps.epv * 100f));
                msg_gps.set("vel", (int)(gps.getSpeed() * 100));
                msg_gps.set("cog", (int) Math.toDegrees(gps.getCog()) * 100);
                msg_gps.set("fix_type", gps.fix);
                msg_gps.set("satellites_visible", 10);
                sendMessage(msg_gps);
            }
        }

        // SYSTEM TIME from host
        if (timeThrottleCounter++ % 1000 == 0) {
            MAVLinkMessage msg_system_time = new MAVLinkMessage(schema, "SYSTEM_TIME", sysId, componentId,
                                                                protocolVersion);
            Calendar cal = Calendar.getInstance(TimeZone.getTimeZone("GMT"));
            msg_system_time.set("time_unix_usec", cal.getTimeInMillis() * 1000);
            msg_system_time.set("time_boot_ms", tu / 1000);
            sendMessage(msg_system_time);
        }
    }

    private void reset() {
        gotHeartBeat = false;
        inited = false;
        stopped = false;
        gotHilActuatorControls = false;
        nextHilStatePub = 0;
    }
}
