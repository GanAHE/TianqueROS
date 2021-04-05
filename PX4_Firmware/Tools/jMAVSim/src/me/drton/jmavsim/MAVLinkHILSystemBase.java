package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.vehicle.AbstractVehicle;

/**
 * MAVLinkHILSystemBase is the superclass of MAVLinkHILSystem and MAVLinkDisplayOnly. It is a bridge between 
 * AbstractVehicle and autopilot connected via MAVLink.
 * MAVLinkHILSystemBase should have the same sysID as the autopilot, but different componentId.
 */
public abstract class MAVLinkHILSystemBase extends MAVLinkSystem {
    protected Simulator simulator;
    protected AbstractVehicle vehicle;

    /**
     * Create MAVLinkHILSimulator, MAVLink system that sends simulated sensors to autopilot and passes controls from
     * autopilot to simulator
     *
     * @param sysId       SysId of simulator should be the same as autopilot
     * @param componentId ComponentId of simulator should be different from autopilot
     * @param vehicle     vehicle to connect
     */
    public MAVLinkHILSystemBase(MAVLinkSchema schema, int sysId, int componentId, AbstractVehicle vehicle) {
        super(schema, sysId, componentId);
        this.vehicle = vehicle;
    }

    public void setSimulator(Simulator simulator) {
        this.simulator = simulator;
    }

    public abstract boolean gotHilActuatorControls();

    public abstract void initMavLink();

    public abstract void endSim();

}
