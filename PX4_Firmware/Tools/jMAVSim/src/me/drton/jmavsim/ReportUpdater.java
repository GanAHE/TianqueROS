package me.drton.jmavsim;

/**
 * Updater for the visualizer's simulation state's report.
 */
public class ReportUpdater extends WorldObject {
    private static final long UPDATE_FREQ_MS = 250;

    private static final StringBuilder builder = new StringBuilder();
    private static long updateFreq;
    private static long nextUpdateT;
    private final Visualizer3D visualizer;


    public ReportUpdater(World world, Visualizer3D visualizer) {
        super(world);
        this.visualizer = visualizer;
        setUpdateFreq(UPDATE_FREQ_MS);
    }

    public static long getUpdateFreq() {
        return ReportUpdater.updateFreq;
    }

    public static void setUpdateFreq(long updateFreq) {
        ReportUpdater.updateFreq = updateFreq;
        ReportUpdater.nextUpdateT = System.currentTimeMillis() + updateFreq;
    }

    public static void resetUpdateFreq() {
        setUpdateFreq(UPDATE_FREQ_MS);
    }

    @Override
    public void update(long t, boolean paused) {
        if (t < nextUpdateT) {
            return;
        }

        nextUpdateT = t + updateFreq;

        if (!visualizer.showReportText()) {
            return;
        }

        builder.setLength(0);

        for (WorldObject object : getWorld().getObjects()) {
            if (object instanceof ReportingObject) {
                ((ReportingObject) object).report(builder);
            }
        }

        visualizer.setReportText(builder.toString());
    }
}
