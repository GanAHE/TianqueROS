package me.drton.jmavsim;

/**
 * User: ton Date: 22.05.14 Time: 17:31
 */
public class MAVLinkMissionItem {
    public final int command;
    public final int frame;
    public final float param1;
    public final float param2;
    public final float param3;
    public final float param4;
    public final float x;
    public final float y;
    public final float z;
    public final int autocontinue;

    public MAVLinkMissionItem(int frame, int command, float param1, float param2, float param3,
                              float param4, float x,
                              float y, float z, int autocontinue) {
        this.frame = frame;
        this.command = command;
        this.param1 = param1;
        this.param2 = param2;
        this.param3 = param3;
        this.param4 = param4;
        this.x = x;
        this.y = y;
        this.z = z;
        this.autocontinue = autocontinue;
    }

    @Override
    public String toString() {
        return String.format(
                   "<MAVLinkMissionItem frame=%s command=%s param1=%s param2=%s param3=%s param4=%s x=%s y=%s z=%s autocontinue=%s />",
                   frame, command, param1, param2, param3, param4, x, y, z, autocontinue);
    }
}
