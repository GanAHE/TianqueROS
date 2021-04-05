package me.drton.jmavsim;

import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.media.j3d.Appearance;
import javax.media.j3d.Material;
import javax.media.j3d.Texture;
import javax.media.j3d.TextureAttributes;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import com.sun.j3d.utils.geometry.Sphere;
import com.sun.j3d.utils.image.TextureLoader;

import java.util.List;

/**
 * User: ton Date: 21.03.14 Time: 23:22
 */
public class CameraGimbal2D extends KinematicObject implements ReportingObject {
    private DynamicObject baseObject;
    private int pitchChannel = -1;
    private double pitchScale = 1.0;
    private int rollChannel = -1;
    private double rollScale = 1.0;
    private Vector3d positionOffset = new Vector3d(0, 0, -0.045);  // from base object
    private double rotOffset = Math.PI / 2.0;
    private float gimbalDia = 0.03f;
    private double[] controls = {0, 0};
    private Sphere model;
    private TransformGroup gimbalTG;
    private Matrix3d rotM3d = new Matrix3d(); // temp storage

    public CameraGimbal2D(World world, String modelName, boolean showGui) {
        super(world, showGui);
        if (!modelName.isEmpty()) {

            Texture texture = new TextureLoader(modelName, null).getTexture();
            texture.setBoundaryModeS(Texture.WRAP);
            texture.setBoundaryModeT(Texture.WRAP);

            Appearance app = new Appearance();
            Color3f black = new Color3f(0.0f, 0.0f, 0.0f);
            Color3f white = new Color3f(1.0f, 1.0f, 1.0f);
            app.setMaterial(new Material(white, black, white, black, 10.0f));

            TextureAttributes texAttr = new TextureAttributes();
            texAttr.setTextureMode(TextureAttributes.REPLACE);
            app.setTexture(texture);
            app.setTextureAttributes(texAttr);

            // the actual model
            model = new Sphere(gimbalDia, Sphere.GENERATE_NORMALS | Sphere.GENERATE_TEXTURE_COORDS, 32, app);

            gimbalTG = new TransformGroup();
            gimbalTG.setCapability(TransformGroup.ALLOW_TRANSFORM_READ);
            gimbalTG.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);

            Transform3D rotT3d = new Transform3D();
            rotT3d.rotZ(rotOffset);
            gimbalTG.setTransform(rotT3d);

            gimbalTG.addChild(model);
            transformGroup.addChild(gimbalTG);
        }
    }

    public void report(StringBuilder builder) {
        builder.append("GIMBAL");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);

        builder.append("Att: ");
        builder.append(ReportUtil.vector2str(ReportUtil.vectRad2Deg(attitude)));
        builder.append(newLine);

        builder.append(String.format("Roll ctrl: %s", ReportUtil.d2str(controls[1])));
        builder.append(newLine);
        builder.append(String.format("Pitch ctrl: %s", ReportUtil.d2str(controls[0])));
        builder.append(newLine);
    }

    public void setBaseObject(DynamicObject object) {
        this.baseObject = object;
    }

    public Vector3d getPositionOffset() {
        return positionOffset;
    }

    public void setPositionOffset(Vector3d positionOffset) {
        this.positionOffset = positionOffset;
    }

    public void setPitchChannel(int pitchChannel) {
        this.pitchChannel = pitchChannel;
    }

    public void setPitchScale(double pitchScale) {
        this.pitchScale = pitchScale / 2;
    }

    public void setRollChannel(int channel) {
        this.rollChannel = channel;
    }

    public void setRollScale(double scale) {
        this.rollScale = scale / 2;
    }

    @Override
    public void update(long t, boolean paused) {
        this.position = (Vector3d) baseObject.position.clone();
        this.attitude = (Vector3d) baseObject.attitude.clone();
        this.rotation.rotZ(this.attitude.z);
        if ((pitchChannel >= 0 || rollChannel >= 0) && baseObject instanceof AbstractVehicle &&
                ((AbstractVehicle) baseObject).getControl().size() > 0) {
            // Control camera pitch/roll
            List<Double> control = ((AbstractVehicle) baseObject).getControl();

            if (rollChannel >= 0) {
                if (control.size() > rollChannel) {
                    this.controls[0] = control.get(rollChannel);
                }
                this.attitude.x = (this.controls[0] * rollScale);
            }
            if (pitchChannel >= 0) {
                if (control.size() > pitchChannel) {
                    this.controls[1] = control.get(pitchChannel);
                }
                this.attitude.y = (this.controls[1] * pitchScale);
            }
        }

        rotM3d.rotX(this.attitude.y);
        rotation.mul(rotM3d);
        rotM3d.rotY(this.attitude.x);
        rotation.mul(rotM3d);
        rotation.normalize();

        Vector3d newOffset = (Vector3d) positionOffset.clone();
        rotation.transform(newOffset);
        this.position.add(newOffset);

    }
}
