package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.World;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Generic hexacopter model.
 */
public class Hexacopter extends AbstractMulticopter {
    private static final int rotorsNum = 6;
    private Vector3d[] rotorPositions = new Vector3d[rotorsNum];

    /**
     * Generic hexacopter constructor.
     *
     * @param world          world where to place the vehicle
     * @param modelName      filename of model to load, in .obj format
     * @param orientation    "x" or "+"
     * @param armLength      length of arm from center
     * @param rotorThrust    full thrust of one rotor
     * @param rotorTorque    torque at full thrust of one rotor
     * @param rotorTimeConst spin-up time of rotor
     * @param rotorsOffset   rotors positions offset from gravity center
     * @param showGui        false if the GUI has been disabled
     */
    public Hexacopter(World world, String modelName, String orientation, double armLength,
                      double rotorThrust, double rotorTorque, double rotorTimeConst,
                      Vector3d rotorsOffset, boolean showGui) {
        super(world, modelName, showGui);
        rotorPositions[0] = new Vector3d(armLength, 0.0, 0.0);
        rotorPositions[1] = new Vector3d(-armLength, 0.0, 0.0);
        rotorPositions[2] = new Vector3d(-armLength * Math.cos(Math.PI / 3),
                                         -armLength * Math.sin(Math.PI / 3), 0.0);
        rotorPositions[3] = new Vector3d(armLength * Math.cos(Math.PI / 3),
                                         armLength * Math.sin(Math.PI / 3), 0.0);
        rotorPositions[4] = new Vector3d(armLength * Math.cos(Math.PI / 3),
                                         -armLength * Math.sin(Math.PI / 3), 0.0);
        rotorPositions[5] = new Vector3d(-armLength * Math.cos(Math.PI / 3),
                                         armLength * Math.sin(Math.PI / 3), 0.0);
        if (orientation.equals("x") || orientation.equals("X")) {
            Matrix3d r = new Matrix3d();
            r.rotZ(Math.PI / 2);
            for (int i = 0; i < rotorsNum; i++) {
                r.transform(rotorPositions[i]);
            }
        } else if (orientation.equals("+")) {
        } else {
            throw new RuntimeException("Unknown orientation: " + orientation);
        }
        for (int i = 0; i < rotors.length; i++) {
            rotorPositions[i].add(rotorsOffset);
            Rotor rotor = rotors[i];
            rotor.setFullThrust(rotorThrust);
            rotor.setFullTorque((i == 1 || i == 3 || i == 4) ? -rotorTorque : rotorTorque);
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
