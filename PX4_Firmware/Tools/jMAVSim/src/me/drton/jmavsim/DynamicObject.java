package me.drton.jmavsim;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Abstract dynamic object class.
 * Calculates all kinematic parameters (attitude, attitude rates, position, velocity, acceleration) from force and torque acting on the vehicle.
 */
public abstract class DynamicObject extends KinematicObject {
    protected long lastTime = -1;
    protected double mass = 1.0;
    protected Matrix3d momentOfInertia = new Matrix3d();
    protected Matrix3d momentOfInertiaInv = new Matrix3d();

    // temp storage objects for calculations
    private Vector3d tmpVec = new Vector3d();
    private Vector3d angularAcc = new Vector3d();
    private Matrix3d rotMtx = new Matrix3d();
    private AxisAngle4d rotAng = new AxisAngle4d();

    public DynamicObject(World world, boolean showGui) {
        super(world, showGui);
        rotation.rotX(0);
        momentOfInertia.rotZ(0.0);
        momentOfInertiaInv.rotZ(0.0);
    }

    public double getMass() {
        return mass;
    }

    public void setMass(double mass) {
        this.mass = mass;
    }

    public void setMomentOfInertia(Matrix3d momentOfInertia) {
        this.momentOfInertia.set(momentOfInertia);
        this.momentOfInertiaInv.invert(momentOfInertia);
    }

    @Override
    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }
        if (lastTime >= 0) {
            double dt = Math.max((t - lastTime) / 1000.0, 0.001);  // constrain time step
            double grnd = getWorld().getEnvironment().getGroundLevelAt(position);

            // Position
            tmpVec.set(velocity);
            tmpVec.scale(dt);
            position.add(tmpVec);
            // Velocity
            acceleration = getForce();
            acceleration.scale(1.0 / mass);
            if (!ignoreGravity) {
                acceleration.add(getWorld().getEnvironment().getG());
            }
            if (position.z >= grnd && velocity.z + acceleration.z * dt >= 0.0) {
                // On ground
//                acceleration.x = -velocity.x / dt;
//                acceleration.y = -velocity.y / dt;
//                acceleration.z = -velocity.z / dt;
                position.z = grnd;
                acceleration.set(0.0, 0.0, 0.0);
                velocity.set(0.0, 0.0, 0.0);
                rotationRate.set(0.0, 0.0, 0.0);
            } else {
                tmpVec.set(acceleration);
                tmpVec.scale(dt);
                velocity.add(tmpVec);
                // Rotation
                if (rotationRate.length() > 0.0) {
                    tmpVec.set(rotationRate);
                    tmpVec.normalize();
                    rotAng.set(tmpVec, rotationRate.length() * dt);
                    rotMtx.set(rotAng);
                    rotation.mulNormalize(rotMtx);
                }
                // Rotation rate
                tmpVec.set(rotationRate);
                momentOfInertia.transform(tmpVec);
                angularAcc.cross(rotationRate, tmpVec);
                angularAcc.negate();
                angularAcc.add(getTorque());
                momentOfInertiaInv.transform(angularAcc);
                angularAcc.scale(dt);
                rotationRate.add(angularAcc);
            }
            attitude.set(utilMatrixToEulers(rotation));
        }
        lastTime = t;
    }

    protected abstract Vector3d getForce();
    protected abstract Vector3d getTorque();
    protected abstract Vector3d getAirFlowForce(Vector3d airSpeed);
}
