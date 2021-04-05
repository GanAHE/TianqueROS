package me.drton.jmavsim;

//import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

public class SystemOutHandler {
    private PrintStream osPrintStream;
    private SystemOutHandlerOutputStream os;
    private boolean active;
    private boolean logToStdOut;

    public SystemOutHandler(boolean logToStdOut) {
        this.active = false;
        this.logToStdOut = logToStdOut;
        this.osPrintStream = System.out;
        this.os = new SystemOutHandlerOutputStream(new ArrayList<OutputStream>(1));
    }

    public void setLogToStdOut(boolean logToStdOut) {
        this.logToStdOut = logToStdOut;
    }

    public void addOutputStream(OutputStream os) {
        this.os.addStream(os);
    }

    public void removeOutputStream(OutputStream os) {
        this.os.removeStream(os);
    }

    public void start(boolean clearStreams) {
        if (this.active) {
            return;
        }

        this.active = true;
        if (clearStreams) {
            os.clearStreams();
        }
        if (this.logToStdOut) {
            os.addStream(this.osPrintStream);
        }

        System.setOut(new PrintStream(this.os));
    }

    public void stop() {
        this.active = false;
        System.setOut(osPrintStream);
    }

    private static class SystemOutHandlerOutputStream extends OutputStream {
        private List<OutputStream> outputStreams;

        public SystemOutHandlerOutputStream(List<OutputStream> outputStreams) {
            this.outputStreams = outputStreams;
        }

        public void addStream(OutputStream os) {
            if (!this.outputStreams.contains(os)) {
                this.outputStreams.add(os);
            }
        }

        public void removeStream(OutputStream os) {
            if (this.outputStreams.contains(os)) {
                this.outputStreams.remove(os);
            }
        }

        public void clearStreams() {
            this.outputStreams.clear();
        }

        public void write(int b) throws IOException {
            for (OutputStream os : this.outputStreams) {
                if (os != null) {
                    os.write(b);
                }
            }
        }

        public void flush() throws IOException {
            for (OutputStream os : this.outputStreams) {
                if (os != null) {
                    os.flush();
                }
            }
        }

        public void close() throws IOException {
            for (OutputStream os : outputStreams) {
                if (os != null) {
                    os.close();
                }
            }
        }
    }
}