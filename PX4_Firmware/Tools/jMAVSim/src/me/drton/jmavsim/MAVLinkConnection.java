package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkMessage;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * User: ton Date: 13.02.14 Time: 21:50
 */
public class MAVLinkConnection extends WorldObject {
    private List<MAVLinkNode> nodes = new ArrayList<MAVLinkNode>();
    private Set<Integer> skipMessages = new HashSet<Integer>();

    public MAVLinkConnection(World world) {
        super(world);
    }

    public void addNode(MAVLinkNode node) {
        nodes.add(node);
        node.addConnection(this);
    }

    public void addSkipMessage(int msgType) {
        skipMessages.add(msgType);
    }

    public void sendMessage(MAVLinkNode sender, MAVLinkMessage msg) {
        if (skipMessages.contains(msg.getMsgType())) {
            return;
        }
        for (MAVLinkNode node : nodes) {
            if (node != sender) {
                node.handleMessage(msg);
            }
        }
    }

    @Override
    public void update(long t, boolean paused) {
        for (MAVLinkNode node : nodes) {
            node.update(t, paused);
        }
    }
}
