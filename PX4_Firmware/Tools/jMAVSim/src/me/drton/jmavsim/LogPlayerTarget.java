package me.drton.jmavsim;

import me.drton.jmavlib.geo.GlobalPositionProjector;
import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavlib.log.FormatErrorException;
import me.drton.jmavlib.log.LogReader;

import javax.vecmath.Vector3d;
import java.io.EOFException;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * User: ton Date: 04.05.14 Time: 23:41
 */
public class LogPlayerTarget extends Target {
    private LogReader logReader = null;
    private long logStart = 0;
    private long timeStart = 0;
    private long logT = 0;
    private Vector3d positionOffset = new Vector3d();
    private String[] posKeys = new String[] {"LPOS.X", "LPOS.Y", "LPOS.Z"};
    private String[] velKeys = new String[] {"LPOS.VX", "LPOS.VY", "LPOS.VZ"};
    private boolean globalFrame = false;
    private GlobalPositionProjector projector = null;
    private Vector3d postitionPrev = new Vector3d();
    private long timePrev = 0;

    public LogPlayerTarget(World world, double size, boolean showGui)
        throws FileNotFoundException {
        super(world, size, showGui);
    }

    public void openLog(LogReader logReader) {
        this.logReader = logReader;
        logStart = timeStart - logReader.getStartMicroseconds() / 1000;
    }

    public void setLogKeys(String[] posKeys, String[] velKeys) {
        this.posKeys = posKeys;
        this.velKeys = velKeys;
    }

    public void setGlobalFrame(boolean globalFrame) {
        this.globalFrame = globalFrame;
        this.projector = new GlobalPositionProjector();
    }

    public void setGlobalReference(LatLonAlt reference) {
        this.projector.init(reference);
    }

    public void setTimeStart(long timeStart) {
        this.timeStart = timeStart;
    }

    public void setPositionOffset(Vector3d positionOffset) {
        this.positionOffset = positionOffset;
    }

    @Override
    public void update(long t, boolean paused) {
        if (logReader != null) {
            Map<String, Object> logData = new HashMap<String, Object>();
            while (logStart + logT < t) {
                try {
                    logT = logReader.readUpdate(logData) / 1000;
                } catch (EOFException e) {
                    break;
                } catch (IOException e) {
                    e.printStackTrace();
                    break;
                } catch (FormatErrorException e) {
                    e.printStackTrace();
                    break;
                }
            }
            if (logData.containsKey(posKeys[0]) &&
                    logData.containsKey(posKeys[1]) &&
                    logData.containsKey(posKeys[2])) {
                double[] v = new double[] {
                    ((Number) logData.get(posKeys[0])).doubleValue(),
                    ((Number) logData.get(posKeys[1])).doubleValue(),
                    ((Number) logData.get(posKeys[2])).doubleValue()
                };
                if (globalFrame) {
                    LatLonAlt latLonAlt = new LatLonAlt(v[0], v[1], v[2]);
                    if (!projector.isInited()) {
                        projector.init(latLonAlt);
                    }
                    position.add(new Vector3d(projector.project(latLonAlt)), positionOffset);
                } else {
                    position.add(new Vector3d(v), positionOffset);
                }
                if (velKeys == null) {
                    // Calculate velocity from position changes
                    velocity.sub(position, postitionPrev);
                    velocity.scale(1000.0 / (logT - timePrev));
                    postitionPrev.set(position);
                    timePrev = logT;
                }
            }
            if (velKeys != null) {
                // Use velocity from log
                if (logData.containsKey(velKeys[0]) &&
                        logData.containsKey(velKeys[1]) &&
                        logData.containsKey(velKeys[2])) {
                    velocity.set((Float) logData.get(velKeys[0]), (Float) logData.get(velKeys[1]),
                                 (Float) logData.get(velKeys[2]));
                }
            }
        }
    }
}
