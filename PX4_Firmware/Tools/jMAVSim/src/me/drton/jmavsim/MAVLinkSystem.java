package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;

/**
 * MAVLinkSystem represents generic MAVLink system with SysID and ComponentID that can handle and send messages.
 * <p/>
 * User: ton Date: 13.02.14 Time: 20:32
 */
public class MAVLinkSystem extends MAVLinkNode {
    public int sysId;
    public int componentId;
    private long heartbeatInterval = 1000;  // [ms]
    private long heartbeatNext = 0;
    protected int protocolVersion = 1;

    public MAVLinkSystem(MAVLinkSchema schema, int sysId, int componentId) {
        super(schema);
        this.sysId = sysId;
        this.componentId = componentId;
    }

    public void setHeartbeatInterval(long interval) {
        this.heartbeatInterval = interval;
    }

    @Override
    public void handleMessage(MAVLinkMessage msg) {
        // Update our mavlink version according to incoming messages.
        protocolVersion = msg.protocolVersion;
    }

    @Override
    public void update(long t, boolean paused) {
        if (heartbeatNext <= t && heartbeatInterval > 0) {
            MAVLinkMessage msg = new MAVLinkMessage(schema, "HEARTBEAT", sysId, componentId, protocolVersion);
            msg.set("mavlink_version", 3);
            sendMessage(msg);
            heartbeatNext = t + heartbeatInterval;
        }
    }
}
