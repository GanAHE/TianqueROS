package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkSchema;

import java.io.IOException;

/**
 * User: ton Date: 02.12.13 Time: 20:56
 */
public abstract class MAVLinkPort extends MAVLinkNode {
    protected MAVLinkPort(MAVLinkSchema schema) {
        super(schema);
    }

    public abstract void open() throws IOException;

    public abstract void close() throws IOException;

    public abstract boolean isOpened();

    public abstract void setDebug(boolean debug);
}
