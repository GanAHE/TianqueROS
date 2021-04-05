package me.drton.jmavsim;

import javax.vecmath.Vector3d;
import java.io.FileNotFoundException;

/**
 * User: ton Date: 04.05.14 Time: 23:25
 */
public class SimpleTarget extends Target {
    private Vector3d positionStart = new Vector3d();
    private Vector3d positionFinish = new Vector3d();
    private long timeStart = 0;
    private long timeFinish = 10000;

    public SimpleTarget(World world, double size, boolean showGui)
        throws FileNotFoundException {
        super(world, size, showGui);
    }

    public void setTrajectory(Vector3d positionStart, Vector3d positionFinish, long timeStart,
                              long timeFinish) {
        this.positionStart = positionStart;
        this.positionFinish = positionFinish;
        this.timeStart = timeStart;
        this.timeFinish = timeFinish;
    }

    @Override
    public void update(long t, boolean paused) {
        double progress = Math.min(1.0, Math.max(0.0,
                                                 (double)(t - timeStart) / (double)(timeFinish - timeStart)));
        Vector3d vec = new Vector3d();
        vec.sub(positionFinish, positionStart);
        position.scaleAdd(progress, vec, positionStart);
        if (progress > 0.0 && progress < 1.0) {
            velocity.scale(1000.0 / (timeFinish - timeStart), vec);
        } else {
            velocity.set(0.0, 0.0, 0.0);
        }
    }
}
