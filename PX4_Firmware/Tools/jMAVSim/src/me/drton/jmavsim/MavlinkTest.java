package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import org.xml.sax.SAXException;

import javax.xml.parsers.ParserConfigurationException;
import java.io.IOException;

/**
 * User: ton Date: 21.03.14 Time: 13:44
 */
public class MavlinkTest {
    public static void main(String[] args)
    throws InterruptedException, IOException, ParserConfigurationException, SAXException {
        World world = new World();
        MAVLinkSchema schema = new MAVLinkSchema("mavlink/message_definitions/common.xml");
        MAVLinkConnection connection = new MAVLinkConnection(world);
        SerialMAVLinkPort port = new SerialMAVLinkPort(schema);
        connection.addNode(port);
        MAVLinkNode node = new MAVLinkNode(schema) {
            @Override
            public void handleMessage(MAVLinkMessage msg) {
                System.out.println(msg);
            }

            @Override
            public void update(long t, boolean paused) {
            }
        };
        connection.addNode(node);
        port.setup("/dev/tty.usbmodem1", 57600, 8, 1, 0);
        port.open();
        port.sendRaw("\nsh /etc/init.d/rc.usb\n".getBytes());
        while (true) {
            port.update(System.currentTimeMillis(), false);
            Thread.sleep(10);
        }
    }
}
