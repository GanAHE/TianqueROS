package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;

import java.util.ArrayList;
import java.util.List;

/**
 * MAVLinkNode is generic object that can handle and send MAVLink messages, but may have no own ID, i.e. it can be e.g.
 * bridge between physical port and virtual MAVLinkConnection.
 * <p/>
 * User: ton Date: 13.02.14 Time: 21:51
 */
public abstract class MAVLinkNode {
    protected MAVLinkSchema schema;
    private List<MAVLinkConnection> connections = new ArrayList<MAVLinkConnection>();

    protected MAVLinkNode(MAVLinkSchema schema) {
        this.schema = schema;
    }

    public void addConnection(MAVLinkConnection connection) {
        connections.add(connection);
    }

    protected void sendMessage(MAVLinkMessage msg) {
        for (MAVLinkConnection connection : connections) {
            connection.sendMessage(this, msg);
        }
    }

    public abstract void handleMessage(MAVLinkMessage msg);

    public abstract void update(long t, boolean paused);
}
