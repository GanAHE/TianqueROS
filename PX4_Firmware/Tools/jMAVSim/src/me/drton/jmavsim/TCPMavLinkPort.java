package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavlib.mavlink.MAVLinkStream;
import me.drton.jmavlib.mavlink.MAVLinkMessage;

import java.io.IOException;
import java.net.*;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.nio.ByteBuffer;
import java.nio.channels.ByteChannel;
import java.util.*;

/**
 * User: ton Date: 02.12.13 Time: 20:56
 */
public class TCPMavLinkPort extends MAVLinkPort {
    private MAVLinkSchema schema;
    private ByteBuffer rxBuffer = ByteBuffer.allocate(8192);
    private InetSocketAddress inetSocketAddress = null;
    private ServerSocketChannel serverSocketChannel = null;
    private SocketChannel socketChannel = null;
    private MAVLinkStream stream;
    private boolean debug = false;

    private boolean monitorMessage = false;
    private HashSet<Integer> monitorMessageIDs;
    private HashMap<Integer, Integer> messageCounts = new HashMap<Integer, Integer>();

    static int MONITOR_MESSAGE_RATE = 100; // rate at which to print message info
    static int TIME_PASSING = 10;         // change the print so it's visible to the user.
    static int time = 0;


    public TCPMavLinkPort(MAVLinkSchema schema) {
        super(schema);
        this.schema = schema;
        rxBuffer.flip();
    }

    public void setMonitorMessageID(HashSet<Integer> ids) {
        this.monitorMessageIDs  = ids;
        for (int id : ids) {
            messageCounts.put(id, 0);
        }
        this.monitorMessage = true;
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void setup(String address, int port) throws UnknownHostException, IOException {
        inetSocketAddress = new InetSocketAddress(port);
    }

    public void open() throws IOException {
        serverSocketChannel = ServerSocketChannel.open();
        serverSocketChannel.socket().bind(inetSocketAddress);
        accept();
        stream = new MAVLinkStream(schema, socketChannel);
        stream.setDebug(true);
    }

    private void accept() {
        if (debug) {
            System.out.println("Waiting to accept TCP connection");
        }

        try {
            socketChannel = serverSocketChannel.accept();
            socketChannel.configureBlocking(false);
            socketChannel.setOption(StandardSocketOptions.TCP_NODELAY, true);
        } catch (IOException ignored) {
        }

        if (debug) {
            System.out.println("TCP connection accepted");
        }
    }

    private void reset() {
        if (debug) {
            System.out.println("Reseting TCP connection.");
        }

        try {
            close();
            open();
        } catch (IOException e) {
            System.err.println("Reset failed: " + e);
        }
    }

    @Override
    public void close() throws IOException {
        serverSocketChannel.close();
    }

    @Override
    public boolean isOpened() {
        return serverSocketChannel != null && serverSocketChannel.isOpen();
    }

    @Override
    public void handleMessage(MAVLinkMessage msg) {
        if (debug) {
            System.out.println("[handleMessage] msg.name: " + msg.getMsgName() + ", type: " + msg.getMsgType());
        }

        if (isOpened()) {
            try {
                stream.write(msg);
                IndicateReceivedMessage(msg.getMsgType());
            } catch (IOException ignored) {
                // This can happen when px4 shuts down and the connection is dropped.
                if (debug) {
                    System.out.println("got exception: " + ignored);
                }
                reset();
            }
        }
    }

    private void IndicateReceivedMessage(int type) {
        if (monitorMessage) {
            boolean shouldPrint = false;
            int count = 0;
            // if the list of messages to monitor is empty, but the flag is on, monitor all messages.
            if (monitorMessageIDs.isEmpty()) {
                if (messageCounts.containsKey(type)) { count = messageCounts.get(type); }
                shouldPrint = count >= MONITOR_MESSAGE_RATE;
            } else {
                // otherwise, only print messages in the list of message IDs we're monitoring.
                if (messageCounts.containsKey(type)) { count = messageCounts.get(type); }
                shouldPrint = count >= MONITOR_MESSAGE_RATE && monitorMessageIDs.contains(type);
            }
            printMessage(shouldPrint, count, type);
        }
    }

    private void printMessage(boolean should, int count, int type) {
        if (should) {
            System.out.println(type);
            messageCounts.put(type, 0);
            if (time >= TIME_PASSING) {
                System.out.println("---");
                time = 0;
            } else {
                time++;
            }
        } else {
            messageCounts.put(type, count + 1);
        }
    }

    @Override
    public void update(long t, boolean paused) {
        while (isOpened()) {
            try {
                MAVLinkMessage msg = stream.read();
                if (msg == null) {
                    break;
                }
                if (debug) {
                    System.out.println("[update] msg.name: " + msg.getMsgName() + ", type: " + msg.getMsgType());
                }
                IndicateReceivedMessage(msg.getMsgType());
                sendMessage(msg);
            } catch (IOException ignored) {
                // This can happen when px4 shuts down and the connection is dropped.
                if (debug) {
                    System.out.println("Received IOException");
                }
                reset();
                return;
            }
        }
    }
}
