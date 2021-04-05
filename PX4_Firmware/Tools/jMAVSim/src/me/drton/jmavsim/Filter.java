package me.drton.jmavsim;

public final class Filter {
    private double tc = 0.0;
    private double z1 = 0.0;

    /**
     * First-order fixed-timestep filter.
     * larger tau, smoother filter
     *
     */
    public void filterInit(double dt, double tau, double setpoint) {
        this.tc = dt / tau;
        filterReset(setpoint);
    }

    public void filterReset(double setpoint) {
        this.z1 = setpoint;
    }

    double filter(double signal) {
        this.z1 += (signal - this.z1) * this.tc;
        return this.z1;
    }
}
