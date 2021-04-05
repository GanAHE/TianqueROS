package me.drton.jmavsim;

import jssc.SerialPort;
import jssc.SerialPortException;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.ByteChannel;

public class SerialPortChannel implements ByteChannel {
    private final SerialPort serialPort;

    public SerialPortChannel(SerialPort serialPort) {
        this.serialPort = serialPort;
    }

    @Override
    public int read(ByteBuffer buffer) throws IOException {
        try {
            int available = serialPort.getInputBufferBytesCount();
            if (available <= 0) {
                return 0;
            }
            byte[] b = serialPort.readBytes(Math.min(available, buffer.remaining()));
            if (b != null) {
                buffer.put(b);
                return b.length;
            } else {
                return 0;
            }
        } catch (SerialPortException e) {
            throw new IOException(e);
        }
    }

    @Override
    public int write(ByteBuffer buffer) throws IOException {
        try {
            byte[] b = new byte[buffer.remaining()];
            buffer.get(b);
            return serialPort.writeBytes(b) ? b.length : 0;
        } catch (SerialPortException e) {
            throw new IOException(e);
        }
    }

    @Override
    public boolean isOpen() {
        return serialPort.isOpened();
    }

    @Override
    public void close() throws IOException {
        try {
            serialPort.closePort();
        } catch (SerialPortException e) {
            throw new IOException(e);
        }
    }
}
