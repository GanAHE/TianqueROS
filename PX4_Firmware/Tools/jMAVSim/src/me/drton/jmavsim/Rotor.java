package me.drton.jmavsim;

/**
 * Simple rotor model. Thrust and torque are proportional to control signal filtered with simple LPF (RC filter), to
 * simulate spin up/slow down.
 */
public class Rotor {
    private double tau = 1.0;
    private double fullThrust = 1.0;
    private double fullTorque = 1.0;
    private double w = 0.0;
    private long lastTime = -1;
    private double control = 0.0;

    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }

        if (lastTime >= 0) {
            double dt = (t - lastTime) / 1000.0;
            w += (control - w) * (1.0 - Math.exp(-dt / tau));
        }
        lastTime = t;
    }

    /**
     * Set control signal
     * @param control control signal normalized to [0...1] for traditional or [-1...1] for reversable rotors
     */
    public void setControl(double control) {
        this.control = control;
    }

    /**
     * Set full thrust
     * @param fullThrust [N]
     */
    public void setFullThrust(double fullThrust) {
        this.fullThrust = fullThrust;
    }

    /**
     * Set torque at full thrust
     * @param fullTorque [N * m]
     */
    public void setFullTorque(double fullTorque) {
        this.fullTorque = fullTorque;
    }

    /**
     * Set time constant (spin-up time).
     * @param timeConstant [s]
     */
    public void setTimeConstant(double timeConstant) {
        this.tau = timeConstant;
    }

    /**
     * Get control signal
     */
    public double getControl() {
        return control;
    }

    /**
     * Get current rotor thrust, [N]
     */
    public double getThrust() {
        return w * fullThrust;
    }

    /**
     * Get current rotor torque [N * m]
     */
    public double getTorque() {
        return control * fullTorque;
    }

    /**
     * Get full thrust, [N]
     */
    public double getFullThrust() {
        return fullThrust;
    }

    /**
     * Get torque at full thrust, [N * m]
     */
    public double getFullTorque() {
        return fullTorque;
    }

    /**
     * Get time constant (spin-up time), [s].
     */
    public double getTimeConstant() {
        return tau;
    }
}
