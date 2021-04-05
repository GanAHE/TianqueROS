package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Vector3d;

/**
 * Abstract multicopter class. Does all necessary calculations for multirotor with any placement of rotors.
 * Only rotors on one plane supported now.
 */
public abstract class AbstractMulticopter extends AbstractVehicle {
    private double dragMove = 0.0;
    private double dragRotate = 0.0;
    protected Rotor[] rotors;

    public AbstractMulticopter(World world, String modelName, boolean showGui) {
        super(world, modelName, showGui);
        rotors = new Rotor[getRotorsNum()];
        for (int i = 0; i < getRotorsNum(); i++) {
            rotors[i] = new Rotor();
        }
    }

    public void report(StringBuilder builder) {
        super.report(builder);
        builder.append("MULTICOPTER");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);

        for (int i = 0; i < getRotorsNum(); i++) {
            reportRotor(builder, i);
            builder.append(newLine);
        }

    }

    private void reportRotor(StringBuilder builder, int rotorIndex) {
        Rotor rotor = rotors[rotorIndex];

        builder.append("ROTOR #");
        builder.append(rotorIndex);
        builder.append(newLine);

        builder.append("Control: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getControl())));
        builder.append(newLine);

        builder.append("Thrust: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getThrust())));
        builder.append(" / ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getFullThrust())));
        builder.append(" [N]");
        builder.append(newLine);

        builder.append("Torque: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getTorque())));
        builder.append(" / ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getFullTorque())));
        builder.append(" [Nm]");
        builder.append(newLine);

        builder.append("Spin up: ");
        builder.append(String.format("%s", ReportUtil.d2str(rotor.getTimeConstant())));
        builder.append(" [s]");
        builder.append(newLine);

        builder.append("Position: ");
        builder.append(ReportUtil.vector2str(getRotorPosition(rotorIndex)));
        builder.append(newLine);

    }

    /**
     * Get number of rotors.
     *
     * @return number of rotors
     */
    protected abstract int getRotorsNum();

    /**
     * Get rotor position relative to gravity center of vehicle.
     *
     * @param i rotor number
     * @return rotor radius-vector from GC
     */
    protected abstract Vector3d getRotorPosition(int i);

    public void setDragMove(double dragMove) {
        this.dragMove = dragMove;
    }

    public void setDragRotate(double dragRotate) {
        this.dragRotate = dragRotate;
    }

    @Override
    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }
        for (Rotor rotor : rotors) {
            rotor.update(t, paused);
        }
        super.update(t, paused);
        for (int i = 0; i < rotors.length; i++) {
            double c = control.size() > i ? control.get(i) : 0.0;
            rotors[i].setControl(c);
        }
    }

    @Override
    protected Vector3d getForce() {
        int n = getRotorsNum();
        Vector3d f = new Vector3d();
        for (int i = 0; i < n; i++) {
            f.z -= rotors[i].getThrust();
        }
        rotation.transform(f);
        Vector3d airSpeed = new Vector3d(getVelocity());
        airSpeed.scale(-1.0);
        if (!ignoreWind) {
            airSpeed.add(getWorld().getEnvironment().getCurrentWind(position));
        }
        f.add(getAirFlowForce(airSpeed));
        return f;
    }

    @Override
    protected Vector3d getTorque() {
        int n = getRotorsNum();
        Vector3d torque = new Vector3d();
        Vector3d m = new Vector3d();
        Vector3d t = new Vector3d();
        for (int i = 0; i < n; i++) {
            // Roll / pitch
            t.z = -rotors[i].getThrust();
            m.cross(getRotorPosition(i), t);
            // Yaw
            m.z -= rotors[i].getTorque();
            torque.add(m);
        }
        Vector3d airRotationRate = new Vector3d(rotationRate);
        airRotationRate.scale(-1.0);
        torque.add(getAirFlowTorque(airRotationRate));
        return torque;
    }

    protected Vector3d getAirFlowForce(Vector3d airSpeed) {
        Vector3d f = new Vector3d(airSpeed);
        f.scale(f.length() * dragMove);
        return f;
    }

    protected Vector3d getAirFlowTorque(Vector3d airRotationRate) {
        Vector3d f = new Vector3d(airRotationRate);
        f.scale(f.length() * dragRotate);
        return f;
    }
}
