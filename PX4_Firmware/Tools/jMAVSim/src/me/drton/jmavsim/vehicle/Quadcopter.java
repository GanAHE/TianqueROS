package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Generic quadcopter model.
 */
public class Quadcopter extends AbstractMulticopter {
    private static final int rotorsNum = 4;
    private Vector3d[] rotorPositions = new Vector3d[rotorsNum];
    private int[] rotorRotations = new int[rotorsNum];

    /**
     * Generic quadcopter constructor.
     *
     * @param world          world where to place the vehicle
     * @param modelName      filename of model to load, in .obj format
     * @param orientation    "x" or "+"
     * @param style          rotor position layout style. "default"/"px4" for px4, or "cw_fr" CW sequential layout starting at front motor
     * @param armLength      length of arm from center [m]
     * @param rotorThrust    full thrust of one rotor [N]
     * @param rotorTorque    torque at full thrust of one rotor in [Nm]
     * @param rotorTimeConst spin-up time of rotor [s]
     * @param rotorsOffset   rotors positions offset from gravity center
     * @param showGui        false if the GUI has been disabled
     */
    public Quadcopter(World world, String modelName, String orientation, String style,
                      double armLength, double rotorThrust, double rotorTorque,
                      double rotorTimeConst, Vector3d rotorsOffset, boolean showGui) {
        super(world, modelName, showGui);

        int i;

        if (style == "cw_fr") {
            rotorPositions[0] = new Vector3d(armLength, 0.0, 0.0);
            rotorPositions[1] = new Vector3d(0.0, armLength, 0.0);
            rotorPositions[2] = new Vector3d(-armLength, 0.0, 0.0);
            rotorPositions[3] = new Vector3d(0.0, -armLength, 0.0);
            for (i = 0; i < rotorsNum; ++i) {
                rotorRotations[i] = ((i % 2) == 0) ? 1 : -1;
            }
        } else {
            rotorPositions[0] = new Vector3d(0.0, armLength, 0.0);
            rotorPositions[1] = new Vector3d(0.0, -armLength, 0.0);
            rotorPositions[2] = new Vector3d(armLength, 0.0, 0.0);
            rotorPositions[3] = new Vector3d(-armLength, 0.0, 0.0);
            for (i = 0; i < rotorsNum; ++i) {
                rotorRotations[i] = (i < 2) ? -1 : 1;
            }
        }

        if (orientation.equals("x") || orientation.equals("X")) {
            Matrix3d r = new Matrix3d();
            r.rotZ(-Math.PI / 4);
            for (i = 0; i < rotorsNum; i++) {
                r.transform(rotorPositions[i]);
            }
        } else if (!orientation.equals("+")) {
            throw new RuntimeException("Unknown orientation: " + orientation);
        }
        for (i = 0; i < rotors.length; i++) {
            rotorPositions[i].add(rotorsOffset);
            Rotor rotor = rotors[i];
            rotor.setFullThrust(rotorThrust);
            rotor.setFullTorque(rotorTorque * rotorRotations[i]);
            rotor.setTimeConstant(rotorTimeConst);
        }
    }

    @Override
    protected int getRotorsNum() {
        return rotorsNum;
    }

    @Override
    protected Vector3d getRotorPosition(int i) {
        return rotorPositions[i];
    }
}
