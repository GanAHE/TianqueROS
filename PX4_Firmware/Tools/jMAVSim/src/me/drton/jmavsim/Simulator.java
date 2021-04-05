package me.drton.jmavsim;

import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.Visualizer3D.ViewTypes;
import me.drton.jmavsim.Visualizer3D.ZoomModes;
import me.drton.jmavsim.vehicle.AbstractMulticopter;
import me.drton.jmavsim.vehicle.Quadcopter;

import org.xml.sax.SAXException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.ParserConfigurationException;

import java.io.IOException;
import java.io.InputStream;
import java.lang.Math;
import java.net.URL;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Scanner;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;;

/**
 * User: ton Date: 26.11.13 Time: 12:33
 */
public class Simulator implements Runnable {

    private static enum Port {
        SERIAL,
        UDP,
        TCP
    }
    private static Port PORT = Port.UDP;

    public static boolean   COMMUNICATE_WITH_QGC  = false;   // open UDP port to QGC
    public static boolean   COMMUNICATE_WITH_SDK  = false;   // open UDP port to SDK
    public static boolean   DO_MAG_FIELD_LOOKUP   =
        false;  // perform online mag incl/decl lookup for current position
    public static boolean   USE_GIMBAL            =
        true;   // enable gimbal modeling (optionally also define remote pitch/roll controls below)
    public static boolean   SHOW_GUI              = true;   // Default is to have the GUI
    public static boolean   GUI_SHOW_REPORT_PANEL = false;  // start with report panel showing
    public static boolean   GUI_START_MAXIMIZED   = false;  // start with gui in maximized window
    public static boolean   GUI_ENABLE_AA         = true;   // anti-alias on 3D scene
    public static ViewTypes GUI_START_VIEW        = ViewTypes.VIEW_STATIC;
    public static ZoomModes GUI_START_ZOOM        = ZoomModes.ZOOM_DYNAMIC;
    public static boolean LOCKSTEP_ENABLED = false;
    public static boolean   LOG_TO_STDOUT         =
        true;   // send System.out messages to stdout (console) as well as any custom handlers (see SystemOutHandler)
    public static boolean DEBUG_MODE = false;
    public static boolean DISPLAY_ONLY = false; // display HIL_STATE_QUATERNION from the autopilot, simulation engine disabled

    public static final int    DEFAULT_SIM_RATE = 250; // Hz
    public static final double    DEFAULT_SPEED_FACTOR = 1.0;
    public static final int    DEFAULT_AUTOPILOT_SYSID =
        -1; // System ID of autopilot to communicate with. -1 to auto set ID on first received heartbeat.
    public static final String DEFAULT_AUTOPILOT_TYPE = "generic";  // eg. "px4" or "aq"
    public static final int    DEFAULT_AUTOPILOT_PORT = 14560;
    public static final int    DEFAULT_QGC_PEER_PORT = 14550;
    public static final int    DEFAULT_SDK_PEER_PORT = 14540;
    public static final String DEFAULT_SERIAL_PATH = "/dev/tty.usbmodem1";
    public static final int    DEFAULT_SERIAL_BAUD_RATE = 230400;
    public static final String LOCAL_HOST = "127.0.0.1";
    public static final String DEFAULT_VEHICLE_MODEL = "models/3dr_arducopter_quad_x.obj";
    public static final String DEFAULT_GIMBAL_MODEL =
        "models/gimbal.png";  // blank for invisible gimbal

    // Set global reference point
    // Zurich Irchel Park: 47.397742, 8.545594, 488m
    // Seattle downtown: 47.592182, -122.316031, 86m
    // Moscow downtown: 55.753395, 37.625427, 155m
    // Trumansburg: 42.5339037, -76.6452384, 287m
    public static LatLonAlt DEFAULT_ORIGIN_POS = new LatLonAlt(47.397742, 8.545594, 488);

    // Mag inclination and declination in degrees. If both are left as zero, then DEFAULT_MAG_FIELD is used.
    // If DO_MAG_FIELD_LOOKUP = true or -automag switch is used then both this value and DEFAULT_MAG_FIELD are ignored.
    // Zurich:  63.39, 2.75
    // Seattle: 69.00, 15.61
    // Moscow: 71.53, 11.45
    // T-burg: 68.17, -11.75
    // public static double  DEFAULT_MAG_INCL = 63.23;
    // public static double  DEFAULT_MAG_DECL = 2.44;
    public static double  DEFAULT_MAG_INCL = 0.f;
    public static double  DEFAULT_MAG_DECL = 0.f;
    // Alternate way to set mag field vectors directly if MAG_INCL and MAG_DECL are zero.
    //   If Y value is left as zero, the X value specifies the horizontal field and an
    //   approximate declination will be added later based on the origin GPS position.
    // Zurich:  (0.21506f, 0.01021f, 0.42974f)
    // Seattle: (0.18403f, 0.05142f, 0.49779f)
    // Moscow:  (0.16348f, 0.03311f, 0.49949f)
    // T-burg:  (0.19202f, -0.03993f, 0.48963f)
    public static Vector3d  DEFAULT_MAG_FIELD = new Vector3d(0.21506f, 0.01021f, 0.42974f);

    public static int    DEFAULT_CAM_PITCH_CHAN =
        4;     // Control gimbal pitch from autopilot, -1 to disable
    public static int    DEFAULT_CAM_ROLL_CHAN  =
        -1;     // Control gimbal roll from autopilot, -1 to disable
    public static Double DEFAULT_CAM_PITCH_SCAL =
        1.57;  // channel value to physical movement (+/-90 deg)
    public static Double DEFAULT_CAM_ROLL_SCAL  =
        1.57;  // channel value to physical movement (+/-90 deg)

    private static int sleepInterval = (int)1e6 / DEFAULT_SIM_RATE;  // Main loop interval, in us
    private static double speedFactor = DEFAULT_SPEED_FACTOR;
    private static int autopilotSysId = DEFAULT_AUTOPILOT_SYSID;
    private static String autopilotType = DEFAULT_AUTOPILOT_TYPE;
    private static String autopilotIpAddress = LOCAL_HOST;
    private static int autopilotPort = DEFAULT_AUTOPILOT_PORT;
    private static String qgcIpAddress = LOCAL_HOST;
    private static String sdkIpAddress = LOCAL_HOST;
    private static int qgcPeerPort = DEFAULT_QGC_PEER_PORT;
    private static int sdkPeerPort = DEFAULT_SDK_PEER_PORT;
    private static String serialPath = DEFAULT_SERIAL_PATH;
    private static int serialBaudRate = DEFAULT_SERIAL_BAUD_RATE;

    private static HashSet<Integer> monitorMessageIds = new HashSet<Integer>();
    private static boolean monitorMessage = false;


    private Visualizer3D visualizer;
    private AbstractMulticopter vehicle;
    private CameraGimbal2D gimbal;
    private MAVLinkHILSystemBase hilSystem;
    private MAVLinkPort autopilotMavLinkPort;
    private UDPMavLinkPort udpGCMavLinkPort;
    private UDPMavLinkPort udpSDKMavLinkPort;
    private ScheduledFuture<?> thisHandle;
    private World world;
    private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
    private SystemOutHandler outputHandler;
//  private int simDelayMax = 500;  // Max delay between simulated and real time to skip samples in simulator, in ms

    private long simTimeUs = 0;
    private volatile boolean paused = false;
    private long lastTimeRan = 0;
    private int checkFactor = 2;
    private int slowDownCounter = 0;
    public volatile boolean shutdown = false;

    public Simulator() throws IOException, InterruptedException {

        // set up custom output handler for all System.out messages
        outputHandler = new SystemOutHandler(LOG_TO_STDOUT);
        outputHandler.start(true);

        // Create world
        world = new World();

        // Initialize the GPS origin - it can be overridden via env variables
        double latRef = DEFAULT_ORIGIN_POS.lat;
        double lonRef = DEFAULT_ORIGIN_POS.lon;
        double altRef = DEFAULT_ORIGIN_POS.alt;
        String latOverride = System.getenv("PX4_HOME_LAT");
        String lonOverride = System.getenv("PX4_HOME_LON");
        if (latOverride != null && lonOverride != null) {
            latRef = Double.parseDouble(latOverride);
            lonRef = Double.parseDouble(lonOverride);
        }
        String altOverride = System.getenv("PX4_HOME_ALT");
        if (altOverride != null) {
            altRef = Double.parseDouble(altOverride);
        }
        LatLonAlt referencePos = new LatLonAlt(latRef, lonRef, altRef);
        world.setGlobalReference(referencePos);

        // Get SITL speed from environment as well.
        String speedFactorStr = System.getenv("PX4_SIM_SPEED_FACTOR");
        if (speedFactorStr != null) {
            speedFactor = Double.parseDouble(speedFactorStr);
        }

        // Create environment
        SimpleEnvironment simpleEnvironment = new SimpleEnvironment(world);
        //simpleEnvironment.setWind(new Vector3d(0.8, 2.0, 0.0));
        simpleEnvironment.setWindDeviation(new Vector3d(6.0, 8.0, 0.00));
        //simpleEnvironment.setGroundLevel(0.0f);
        world.addObject(simpleEnvironment);

        if (SHOW_GUI) {
            // Create GUI
            System.out.println("Starting GUI...");  // this is the longest part of startup so let user know
            visualizer = new Visualizer3D(world);
            visualizer.setSimulator(this);
            visualizer.setAAEnabled(GUI_ENABLE_AA);
            if (GUI_START_MAXIMIZED) {
                visualizer.setExtendedState(JFrame.MAXIMIZED_BOTH);
            }

            // add GUI output stream handler for displaying messages
            outputHandler.addOutputStream(visualizer.getOutputStream());
        } else {
            // GUI is disabled
            System.out.println("GUI not enabled");
            visualizer = null;
        }

        MAVLinkSchema schema = null;
        try {
            schema = new MAVLinkSchema("mavlink/message_definitions/common.xml");
        } catch (ParserConfigurationException | IOException | SAXException e) {
            System.out.println("ERROR: Could not load Mavlink Schema: " + e.getLocalizedMessage());
            shutdown = true;
        }

        // Create MAVLink connections
        MAVLinkConnection connHIL = new MAVLinkConnection(world);
        world.addObject(connHIL);

        // Create ports
        if (PORT == Port.SERIAL) {
            SerialMAVLinkPort port = new SerialMAVLinkPort(schema);
            port.setup(serialPath, serialBaudRate, 8, 1, 0);
            port.setDebug(DEBUG_MODE);
            autopilotMavLinkPort = port;

        } else if (PORT == Port.TCP) {
            TCPMavLinkPort port = new TCPMavLinkPort(schema);
            port.setDebug(DEBUG_MODE);
            port.setup(autopilotIpAddress, autopilotPort);
            if (monitorMessage) {
                port.setMonitorMessageID(monitorMessageIds);
            }
            autopilotMavLinkPort = port;
        } else {
            UDPMavLinkPort port = new UDPMavLinkPort(schema);
            port.setDebug(DEBUG_MODE);
            port.setup(autopilotIpAddress, autopilotPort);
            if (monitorMessage) {
                port.setMonitorMessageID(monitorMessageIds);
            }
            autopilotMavLinkPort = port;
        }

        // allow HIL and GCS to talk to this port
        connHIL.addNode(autopilotMavLinkPort);

        // We don't want to spam QGC or SDK with HIL messages.
        String[] skipMessages = {
            "HIL_CONTROLS",
            "HIL_ACTUATOR_CONTROLS",
            "HIL_SENSOR",
            "HIL_GPS",
            "HIL_STATE_QUATERNION"
        };

        if (COMMUNICATE_WITH_QGC) {
            MAVLinkConnection connQGC = new MAVLinkConnection(world);
            if (schema != null) {
                for (String  skipMessage : skipMessages) {
                    connQGC.addSkipMessage(schema.getMessageDefinition(skipMessage).id);
                }
            }
            world.addObject(connQGC);

            udpGCMavLinkPort = new UDPMavLinkPort(schema);
            udpGCMavLinkPort.setDebug(DEBUG_MODE);
            udpGCMavLinkPort.setup(qgcIpAddress, qgcPeerPort);
            if (monitorMessage && PORT == Port.SERIAL) {
                udpGCMavLinkPort.setMonitorMessageID(monitorMessageIds);
            }
            connQGC.addNode(udpGCMavLinkPort);
            connQGC.addNode(autopilotMavLinkPort);
        }

        if (COMMUNICATE_WITH_SDK) {

            MAVLinkConnection connSDK = new MAVLinkConnection(world);
            if (schema != null) {
                for (String  skipMessage : skipMessages) {
                    connSDK.addSkipMessage(schema.getMessageDefinition(skipMessage).id);
                }
            }
            world.addObject(connSDK);

            udpSDKMavLinkPort = new UDPMavLinkPort(schema);
            udpSDKMavLinkPort.setDebug(DEBUG_MODE);
            udpSDKMavLinkPort.setup(sdkIpAddress, sdkPeerPort);
            if (monitorMessage && PORT == Port.SERIAL) {
                udpSDKMavLinkPort.setMonitorMessageID(monitorMessageIds);
            }
            connSDK.addNode(udpSDKMavLinkPort);
            connSDK.addNode(autopilotMavLinkPort);
        }

        // Set up magnetic field deviations
        // (do this after environment already has a reference point in case we need to look up declination manually)
        if (DO_MAG_FIELD_LOOKUP) {
            simpleEnvironment.setMagField(magFieldLookup(referencePos));
        } else if (DEFAULT_MAG_INCL != 0.0 || DEFAULT_MAG_DECL != 0.0) {
            simpleEnvironment.setMagFieldByInclDecl(DEFAULT_MAG_INCL, DEFAULT_MAG_DECL);
        } else if (DEFAULT_MAG_FIELD.y == 0.0 && (DEFAULT_MAG_FIELD.x != 0.0 ||
                                                   DEFAULT_MAG_FIELD.z != 0.0)) {
            Vector3d magField = DEFAULT_MAG_FIELD;
            // Set declination based on the initialization position of the Simulator
            // getMagDeclination() returns degrees and variable decl is in radians.
            double decl = Math.toRadians(simpleEnvironment.getMagDeclination(referencePos.lat,
                                                                             referencePos.lon));
            //System.out.println("Declination: " + (Math.toDegrees(decl)));
            Matrix3d magDecl = new Matrix3d();
            magDecl.rotZ(decl);
            magDecl.transform(magField);
            simpleEnvironment.setMagField(magField);
        } else if (DEFAULT_MAG_FIELD.y != 0.0
                   && DEFAULT_MAG_FIELD.x != 0.0
                   && DEFAULT_MAG_FIELD.z != 0.0) {

            simpleEnvironment.setMagField(DEFAULT_MAG_FIELD);
        }

        // Create vehicle with sensors
        if (autopilotType == "aq") {
            vehicle = buildAQ_leora();
        } else {
            vehicle = buildMulticopter();
        }

        // Create MAVLink HIL system
        // SysId should be the same as autopilot, ComponentId should be different!
        if (DISPLAY_ONLY){
            vehicle.setIgnoreGravity(true);
            vehicle.setIgnoreWind(true);
            hilSystem = new MAVLinkDisplayOnly(schema, autopilotSysId, 51, vehicle);
        } else {
            hilSystem = new MAVLinkHILSystem(schema, autopilotSysId, 51, vehicle);
            if (SHOW_GUI) {
                visualizer.setHilSystem((MAVLinkHILSystem)hilSystem);
            }
        }
        hilSystem.setSimulator(this);
        //hilSystem.setHeartbeatInterval(0);
        connHIL.addNode(hilSystem);
        world.addObject(vehicle);

        if (SHOW_GUI) {
            // Put camera on vehicle with gimbal
            if (USE_GIMBAL) {
                gimbal = buildGimbal();
                world.addObject(gimbal);
                visualizer.setGimbalViewObject(gimbal);
            }

            // Create simulation report updater
            world.addObject(new ReportUpdater(world, visualizer));

            visualizer.addWorldModels();
            visualizer.setVehicleViewObject(vehicle);

            // set default view and zoom mode
            visualizer.setViewType(GUI_START_VIEW);
            visualizer.setZoomMode(GUI_START_ZOOM);
            visualizer.toggleReportPanel(GUI_SHOW_REPORT_PANEL);
        }

        // Open ports
        try {
            autopilotMavLinkPort.open();
        } catch (IOException e) {
            System.out.println("ERROR: Failed to open MAV port: " + e.getLocalizedMessage());
            shutdown = true;
        }

        if (COMMUNICATE_WITH_QGC) {
            try {
                udpGCMavLinkPort.open();
            } catch (IOException e) {
                System.out.println("ERROR: Failed to open UDP link to QGC: " + e.getLocalizedMessage());
            }
        }

        if (COMMUNICATE_WITH_SDK) {
            try {
                udpSDKMavLinkPort.open();
            } catch (IOException e) {
                System.out.println("ERROR: Failed to open UDP link to SDK: " + e.getLocalizedMessage());
            }
        }

        if (LOCKSTEP_ENABLED) {
            thisHandle = executor.scheduleAtFixedRate(this, 0, (int)(sleepInterval / speedFactor / checkFactor),
                                                      TimeUnit.MICROSECONDS);
        } else {
            thisHandle = executor.scheduleAtFixedRate(this, 0, (int)(sleepInterval), TimeUnit.MICROSECONDS);
        }

        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                try {
                    Thread.sleep(200);

                    System.out.println("Shutting down...");
                    if (hilSystem != null) {
                        hilSystem.endSim();
                    }

                    // Close ports
                    if (autopilotMavLinkPort != null && autopilotMavLinkPort.isOpened()) {
                        autopilotMavLinkPort.close();
                    }
                    if (udpGCMavLinkPort != null && udpGCMavLinkPort.isOpened()) {
                        udpGCMavLinkPort.close();
                    }

                    if (thisHandle != null) {
                        thisHandle.cancel(true);
                    }
                    executor.shutdown();

                } catch (InterruptedException | IOException e) {
                    e.printStackTrace();
                }
            }
        });

        while (true) {
            Thread.sleep(100);

            if (shutdown) {
                break;
            }
        }

        System.exit(0);

    }

    public void pauseToggle() {
        paused = !paused;
    }

    private AbstractMulticopter buildMulticopter() {
        Vector3d gc = new Vector3d(0.0, 0.0, 0.0);  // gravity center
        AbstractMulticopter vehicle = new Quadcopter(world, DEFAULT_VEHICLE_MODEL, "x", "default",
                                                     0.33 / 2, 4.0, 0.05, 0.005, gc, SHOW_GUI);
        Matrix3d I = new Matrix3d();
        // Moments of inertia
        I.m00 = 0.005;  // X
        I.m11 = 0.005;  // Y
        I.m22 = 0.009;  // Z
        vehicle.setMomentOfInertia(I);
        vehicle.setMass(0.8);
        vehicle.setDragMove(0.01);
        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSInterval(50);
        sensors.setGPSDelay(200);
        sensors.setNoise_Acc(0.05f);
        sensors.setNoise_Gyo(0.01f);
        sensors.setNoise_Mag(0.005f);
        sensors.setNoise_Prs(0.1f);
        vehicle.setSensors(sensors, getSimMillis());
        //v.setDragRotate(0.1);

        return vehicle;
    }

    // 200mm, 250g small quad X "Leora" with AutoQuad style layout (clockwise from front)
    private AbstractMulticopter buildAQ_leora() {
        Vector3d gc = new Vector3d(0.0, 0.0, 0.0);  // gravity center
        AbstractMulticopter vehicle = new Quadcopter(world, DEFAULT_VEHICLE_MODEL, "x", "cw_fr", 0.1, 1.35,
                                                     0.02, 0.0005, gc, SHOW_GUI);

        Matrix3d I = new Matrix3d();
        // Moments of inertia
        I.m00 = 0.0017;  // X
        I.m11 = 0.0017;  // Y
        I.m22 = 0.002;   // Z

        vehicle.setMomentOfInertia(I);
        vehicle.setMass(0.25);
        vehicle.setDragMove(0.01);
        //v.setDragRotate(0.1);

        SimpleSensors sensors = new SimpleSensors();
        sensors.setGPSInterval(50);
        sensors.setGPSDelay(0);  // [ms]
        //sensors.setPressureAltOffset(world.getGlobalReference().alt);
        sensors.setNoise_Acc(0.02f);
        sensors.setNoise_Gyo(0.001f);
        sensors.setNoise_Mag(0.005f);
        sensors.setNoise_Prs(0.01f);

        vehicle.setSensors(sensors, getSimMillis());

        return vehicle;
    }

    private CameraGimbal2D buildGimbal() {
        CameraGimbal2D g = new CameraGimbal2D(world, DEFAULT_GIMBAL_MODEL, SHOW_GUI);
        g.setBaseObject(vehicle);
        g.setPitchChannel(DEFAULT_CAM_PITCH_CHAN);
        g.setPitchScale(DEFAULT_CAM_PITCH_SCAL);
        g.setRollChannel(DEFAULT_CAM_ROLL_CHAN);
        g.setRollScale(DEFAULT_CAM_ROLL_SCAL);
        return g;
    }

    public void run() {
        if (paused) {
            return;
        }

        boolean needsToPause = false;
        long now;

        if (LOCKSTEP_ENABLED && !DISPLAY_ONLY) {
            // In lockstep we run every update with a checkFactor of (e.g. 2).
            // This way every second update is just an IO (input/output) run where
            // time is not increased.
            boolean ioRunOnly = (slowDownCounter % checkFactor != 0);

            if (!hilSystem.gotHilActuatorControls() && !ioRunOnly) {
                advanceTime();
            }

            now = getSimMillis();

            needsToPause = ((lastTimeRan == now) || ioRunOnly);
        } else {
            now = getSimMillis();
        }

        try {
            world.update(now, needsToPause);
        } catch (Exception e) {
            System.err.println("Exception in Simulator.world.update() : ");
            e.printStackTrace();
            executor.shutdown();
        }

        if (!needsToPause) {
            lastTimeRan = now;
        }
        slowDownCounter++;
    }

    /**
     * Look up the magnetic inclination and declination for a given Lat/Lon/Alt using a NOAA Web service.
     * If successful, returns a valid Vector3d() suitable for setting the magnetic field in the simulated environment.
     * Also displays the resulting vector so it can be copied to the DEFAULT_MAG_FIELD setting to avoid future lookups.
     *
     * @param pos {@link me.drton.jmavlib.geo.LatLonAlt} object of reference point.
     * @return Vector3d The magnetic field variance vector, or Vector3d(0,0,0) if lookup failed.
     */
    public static Vector3d magFieldLookup(LatLonAlt pos) {

        Double decl;
        Double incl;
        Vector3d magField = new Vector3d(0.0f, 0.0f, 0.0f);
        String resp, vals[];

        String reqUrl = "http://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?";
        reqUrl += "resultFormat=csv&coordinateSystem=M&lat1=" + pos.lat + "&lon1=" + pos.lon + "&elevation="
                  + pos.alt / 1e3;
        System.out.println("Attempting magnetic field data lookup from NOAA...");
        try {
            InputStream instr = new URL(reqUrl).openStream();
            Scanner scan = new Scanner(instr, "UTF-8");
            resp = scan.useDelimiter("\\A").hasNext() ? scan.next() : "";
            String lines[] = resp.split("\n");
            vals = lines[lines.length - 1].split(",");
            scan.close();
            instr.close();
        } catch (IOException e) {
            System.err.println("Error requesting URL: " + reqUrl + "\n");
            return magField;
        }

        if (vals.length > 3) {
            try {
                decl = Double.valueOf(vals[1]);
                incl = Double.valueOf(vals[2]);
            } catch (NumberFormatException e) {
                System.err.println("Error parsing response: " + resp + "\n");
                return magField;
            }
            System.out.println("Lookup Declination: " + decl + "; Inclination: " + incl);
            decl = Math.toRadians(decl);
            incl = Math.toRadians(incl);
            magField = new Vector3d(Math.cos(incl), 0.0f, Math.sin(incl));
            Matrix3d declMtx = new Matrix3d();
            declMtx.rotZ(decl);
            declMtx.transform(magField);
            System.out.printf("Result Vectors: Vector3d(%.5f, %.5f, %.5f) \n", magField.x, magField.y,
                              magField.z);
            System.out.printf("       Declination: %.5f; Inclination: %.5f \n",
                              Math.toDegrees(Math.atan2(magField.y, magField.x)), Math.toDegrees(Math.atan2(magField.z,
                                      magField.x)));
        } else {
            System.err.println("Error parsing response: " + resp + "\n");
        }

        return magField;
    }

    public long getSimMillis() {
        if (LOCKSTEP_ENABLED) {
            if (simTimeUs == 0) {
                simTimeUs = System.currentTimeMillis() * 1000;
            }
            return simTimeUs / 1000;
        } else {
            return System.currentTimeMillis();
        }
    }

    public long getRealMillis() {
        return System.currentTimeMillis();
    }

    public void advanceTime() {
        if (LOCKSTEP_ENABLED) {
            simTimeUs += sleepInterval;
        }
        // not needed without lockstep.
    }

    public final static String PRINT_INDICATION_STRING = "-m [<MsgID[, MsgID]...>]";
    public final static String UDP_STRING = "-udp <mav ip>:<mav port>";
    public final static String TCP_STRING = "-tcp <mav ip>:<mav port>";
    public final static String QGC_STRING = "-qgc <qgc ip address>:<qgc peer port>";
    public final static String SDK_STRING = "-sdk <sdk ip address>:<sdk peer port>";
    public final static String SERIAL_STRING = "-serial [<path> <baudRate>]";
    public final static String MAG_STRING = "-automag";
    public final static String REP_STRING = "-rep";
    public final static String SHOW_GUI_STRING = "[-no]-gui";
    public final static String GUI_AA_STRING = "[-no]-aa";
    public final static String GIMBAL_STRING = "[-no]-gimbal";
    public final static String GUI_MAX_STRING = "-max";
    public final static String GUI_VIEW_STRING = "-view (fpv|grnd|gmbl)";
    public final static String AP_STRING = "-ap <autopilot_type>";
    public final static String RATE_STRING = "-r <Hz>";
    public final static String SPEED_FACTOR_STRING = "-f";
    public final static String LOCKSTEP_STRING = "-lockstep";
    public final static String DISPLAY_ONLY_STRING = "-disponly";
    public final static String CMD_STRING =
        "java [-Xmx512m] -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator";
    public final static String CMD_STRING_JAR = "java [-Xmx512m] -jar jmavsim_run.jar";
    public final static String USAGE_STRING = CMD_STRING_JAR + " [-h] [" +
                                              UDP_STRING + " | " +
                                              SERIAL_STRING + "] [" +
                                              RATE_STRING + "] [" +
                                              AP_STRING + "] [" +
                                              MAG_STRING + "] " + "[" +
                                              QGC_STRING + "] [" +
                                              SDK_STRING + "] [" +
                                              GIMBAL_STRING + "] [" +
                                              SHOW_GUI_STRING + "] [" +
                                              GUI_AA_STRING + "] [" +
                                              GUI_MAX_STRING + "] [" +
                                              GUI_VIEW_STRING + "] [" +
                                              REP_STRING + "] [" +
                                              PRINT_INDICATION_STRING + "] [" +
                                              DISPLAY_ONLY_STRING + "]";

    public static void main(String[] args)
    throws InterruptedException, IOException {

        int i = 0;
        while (i < args.length) {
            String arg = args[i++];
            if (arg.equalsIgnoreCase("-h") || arg.equalsIgnoreCase("--help")) {
                handleHelpFlag();
                return;
            }
            if (arg.equalsIgnoreCase("-m")) {
                monitorMessage = true;
                if (i < args.length) {
                    String nextArg = args[i++];
                    try {
                        if (nextArg.startsWith("-")) {
                            // if user ONLY passes in -m, monitor all messages.
                            i--;
                            continue;
                        }
                        if (nextArg.contains(",")) {
                            String split[] = nextArg.split(",");
                            for (String s : split) {
                                monitorMessageIds.add(Integer.parseInt(s));
                            }
                        } else {
                            monitorMessageIds.add(Integer.parseInt(nextArg));
                        }
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + PRINT_INDICATION_STRING + ", got: " + Arrays.toString(args));
                        return;
                    }
                } else {
                    // if user ONLY passes in -m, monitor all messages.
                    continue;
                }
            } else if (arg.equalsIgnoreCase("-udp")) {
                PORT = Port.UDP;
                if (i == args.length) {
                    // only arg is -udp, so use default values.
                    break;
                }
                if (i < args.length) {
                    String nextArg = args[i++];
                    if (nextArg.startsWith("-")) {
                        // only turning on udp, but want to use default ports
                        i--;
                        continue;
                    }
                    try {
                        // try to parse passed-in ports.
                        String[] list = nextArg.split(":");
                        if (list.length != 2) {
                            System.err.println("Expected: " + UDP_STRING + ", got: " + Arrays.toString(list));
                            return;
                        }
                        autopilotIpAddress = list[0];
                        autopilotPort = Integer.parseInt(list[1]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + USAGE_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-udp needs an argument: " + UDP_STRING);
                    return;
                }
            } else if (arg.equalsIgnoreCase("-tcp")) {
                PORT = Port.TCP;
                if (i == args.length) {
                    // only arg is -tcp, so use default values.
                    break;
                }
                if (i < args.length) {
                    String nextArg = args[i++];
                    if (nextArg.startsWith("-")) {
                        // only turning on udp, but want to use default ports
                        i--;
                        continue;
                    }
                    try {
                        // try to parse passed-in ports.
                        String[] list = nextArg.split(":");
                        if (list.length != 2) {
                            System.err.println("Expected: " + TCP_STRING + ", got: " + Arrays.toString(list));
                            return;
                        }
                        autopilotIpAddress = list[0];
                        autopilotPort = Integer.parseInt(list[1]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + USAGE_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-tcp needs an argument: " + TCP_STRING);
                    return;
                }
            } else if (arg.equals("-serial")) {
                PORT = Port.SERIAL;
                if (i >= args.length) {
                    // only arg is -serial, so use default values
                    break;
                }
                String nextArg = args[i++];
                if (nextArg.startsWith("-")) {
                    i--;
                    continue;
                }
                if ((i + 1) <= args.length) {
                    try {
                        serialPath = nextArg;
                        serialBaudRate = Integer.parseInt(args[i++]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + USAGE_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-serial needs two arguments. Expected: " + SERIAL_STRING + ", got: " +
                                       Arrays.toString(args));
                    return;
                }
            } else if (arg.equals("-qgc")) {
                COMMUNICATE_WITH_QGC = true;
                if (i == args.length) {
                    // only arg is -qgc, so use default values.
                    break;
                }
                if (i < args.length) {
                    String nextArg = args[i++];
                    if (nextArg.startsWith("-")) {
                        // only turning on udp, but want to use default ports
                        i--;
                        continue;
                    }
                    try {
                        // try to parse passed-in ports.
                        String[] list = nextArg.split(":");
                        if (list.length != 2) {
                            System.err.println("Expected: " + QGC_STRING + ", got: " + Arrays.toString(list));
                            return;
                        }
                        qgcIpAddress = list[0];
                        qgcPeerPort = Integer.parseInt(list[1]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + QGC_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-qgc needs an argument: " + QGC_STRING);
                    return;
                }
            } else if (arg.equals("-sdk")) {
                COMMUNICATE_WITH_SDK = true;
                if (i == args.length) {
                    // only arg is -sdk, so use default values.
                    break;
                }
                if (i < args.length) {
                    String nextArg = args[i++];
                    if (nextArg.startsWith("-")) {
                        // only turning on udp, but want to use default ports
                        i--;
                        continue;
                    }
                    try {
                        // try to parse passed-in ports.
                        String[] list = nextArg.split(":");
                        if (list.length != 2) {
                            System.err.println("Expected: " + SDK_STRING + ", got: " + Arrays.toString(list));
                            return;
                        }
                        sdkIpAddress = list[0];
                        sdkPeerPort = Integer.parseInt(list[1]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected: " + SDK_STRING + ", got: " + e.toString());
                        return;
                    }
                } else {
                    System.err.println("-sdk needs an argument: " + SDK_STRING);
                    return;
                }
            } else if (arg.equals("-ap")) {
                if (i < args.length) {
                    autopilotType = args[i++];
                } else {
                    System.err.println("-ap requires the autopilot name as an argument.");
                    return;
                }
            } else if (arg.equals("-r")) {
                if (i < args.length) {
                    int t;
                    try {
                        t = Integer.parseInt(args[i++]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected numeric argument after -r: " + RATE_STRING);
                        return;
                    }
                    sleepInterval = (int)1e6 / t;
                } else {
                    System.err.println("-r requires Hz as an argument.");
                    return;
                }
            } else if (arg.equals("-f")) {
                if (i < args.length) {
                    double f;
                    try {
                        f = Double.parseDouble(args[i++]);
                    } catch (NumberFormatException e) {
                        System.err.println("Expected numeric argument after -f: " + SPEED_FACTOR_STRING);
                        return;
                    }
                    speedFactor = f;
                    System.out.println("Warning: Setting speed factor using -f is deprecated.");
                    System.out.println("Please use the environment variable PX4_SIM_SPEED_FACTOR instead.");
                } else {
                    System.err.println("-r requires Hz as an argument.");
                    return;
                }
            } else if (arg.equals("-view")) {
                String t;
                if (i < args.length) {
                    t = args[i++];
                    if (t.equals("fpv")) {
                        GUI_START_VIEW = ViewTypes.VIEW_FPV;
                    } else if (t.equals("grnd")) {
                        GUI_START_VIEW = ViewTypes.VIEW_STATIC;
                    } else if (t.equals("gmbl")) {
                        GUI_START_VIEW = ViewTypes.VIEW_GIMBAL;
                    } else {
                        System.out.println("Warning: Unrecognized value for -view option, ignoring.");
                    }
                } else {
                    System.err.println("-view requires an argument: " + GUI_VIEW_STRING);
                    return;
                }
            } else if (arg.equals(DISPLAY_ONLY_STRING)) {
                DISPLAY_ONLY = true;    // // display HIL_STATE_QUATERNION from the autopilot, simulation engine disabled
            } else if (arg.equals("-automag")) {
                DO_MAG_FIELD_LOOKUP = true;
            } else if (arg.equals("-rep")) {
                GUI_SHOW_REPORT_PANEL = true;
            } else if (arg.equals("-max")) {
                GUI_START_MAXIMIZED = true;
            } else if (arg.equals("-aa")) {
                GUI_ENABLE_AA = true;
            } else if (arg.equals("-no-aa")) {
                GUI_ENABLE_AA = false;
            } else if (arg.equals("-gimbal")) {
                USE_GIMBAL = true;
            } else if (arg.equals("-no-gimbal")) {
                USE_GIMBAL = false;
            } else if (arg.equals("-gui")) {
                SHOW_GUI = true;
            } else if (arg.equals("-no-gui")) {
                SHOW_GUI = false;
            } else if (arg.equals("-lockstep")) {
                LOCKSTEP_ENABLED = true;
            } else if (arg.equals("-debug")) {
                DEBUG_MODE = true;
            } else {
                System.err.println("Unknown flag: " + arg + ", usage: " + USAGE_STRING);
                return;
            }
        }

        if (i != args.length) {
            System.err.println("Usage: " + USAGE_STRING);
            return;
        }

        if (speedFactor != DEFAULT_SPEED_FACTOR && !LOCKSTEP_ENABLED) {
            System.err.println(SPEED_FACTOR_STRING + " requires lockstep to be enabled using: '" +
                               LOCKSTEP_STRING + "'");
            return;
        }

        System.out.println("Options parsed, starting Sim.");

        SwingUtilities.invokeLater(new Simulator());
    }

    public static void handleHelpFlag() {
        String viewType = (GUI_START_VIEW == ViewTypes.VIEW_FPV ? "fpv" : GUI_START_VIEW ==
                           ViewTypes.VIEW_GIMBAL ? "gmbl" : "grnd");

        System.out.println("\nUsage: " + USAGE_STRING + "\n");
        System.out.println("Command-line options:\n");
        System.out.println(UDP_STRING);
        System.out.println("      Open a TCP/IP UDP connection to the MAV (default: " + autopilotIpAddress +
                           ":" + autopilotPort + ").");
        System.out.println(SERIAL_STRING);
        System.out.println("      Open a serial connection to the MAV instead of UDP.");
        System.out.println("      Default path/baud is: " + serialPath + " " + serialBaudRate + "");
        System.out.println(RATE_STRING);
        System.out.println("      Refresh rate at which jMAVSim runs. This dictates the frequency");
        System.out.println("      of the HIL_SENSOR messages. Default is " + DEFAULT_SIM_RATE + " Hz");
        System.out.println(SPEED_FACTOR_STRING);
        System.out.println("      Speed factor at which jMAVSim runs. A factor of 2.0 means the system");
        System.out.println("      runs double than real time speed. Default is " + DEFAULT_SPEED_FACTOR);
        System.out.println(LOCKSTEP_STRING);
        System.out.println("      Set to enable Lockstep simulation (used with PX4 SITL),");
        System.out.println("      required for speed factor '-f'.");
        System.out.println(AP_STRING);
        System.out.println("      Specify the MAV type. E.g. 'px4' or 'aq'. Default is: " + autopilotType +
                           "");
        System.out.println(MAG_STRING);
        System.out.println("      Attempt automatic magnetic field inclination/declination lookup");
        System.out.println("      for starting global position via NOAA Web service.");
        System.out.println(QGC_STRING);
        System.out.println("      Forward message packets to QGC via UDP at " + qgcIpAddress + ":" +
                           qgcPeerPort + "");
        System.out.println(SDK_STRING);
        System.out.println("      Forward message packets to SDK via UDP at " + sdkIpAddress + ":" +
                           sdkPeerPort + "");
        System.out.println(GIMBAL_STRING);
        System.out.println("      Enable/Disable the gimbal model. Default is '" + USE_GIMBAL + "'.");
        System.out.println(SHOW_GUI_STRING);
        System.out.println("      Enable/Disable the GUI. Default is '" + SHOW_GUI +
                           "'.");
        System.out.println(GUI_AA_STRING);
        System.out.println("      Enable/Disable anti-aliasing on 3D scene. Default is '" + GUI_ENABLE_AA +
                           "'.");
        System.out.println(GUI_VIEW_STRING);
        System.out.println("      Start with the specified view type. One of: 'fpv', 'grnd', or 'gmbl'.");
        System.out.println("      Default is '" + viewType + "'.");
        System.out.println(GUI_MAX_STRING);
        System.out.println("      Start with the visualizer GUI window maximized.");
        System.out.println(REP_STRING);
        System.out.println("      Start with data report visible.");
        System.out.println(PRINT_INDICATION_STRING);
        System.out.println("      Monitor (echo) all/selected MAVLink messages to the console.");
        System.out.println("      If no MsgIDs are specified, all messages are monitored.");
        System.out.println(DISPLAY_ONLY_STRING);
        System.out.println("      Disable the simulation engine.");
        System.out.println("      Display the autopilot states from HIL_STATE_QUATERNION.");
        System.out.println("      Compatible with simulation-in-hardware.");
        System.out.println("");
        System.out.println("Key commands (in the visualizer window):");
        System.out.println("");
        printKeyCommands();
        //System.out.println("\n Note: if <qgc <port> is set to -1, JMavSim won't generate Mavlink messages for GroundControl.");
    }

    public static void printKeyCommands() {
        System.out.println("Views:");
        System.out.println("    F    - First-person-view camera.");
        System.out.println("    S    - Stationary ground camera.");
        System.out.println("    G    - Gimbal camera.");
        System.out.println("    Z    - Toggle auto-zoom for Stationary camera.");
        System.out.println("   +/-   - Zoom in/out");
        System.out.println(" 0/ENTER - Reset zoom to default.");
        System.out.println("");
        System.out.println("Actions:");
        System.out.println("   Q   - Disable sim on MAV.");
        System.out.println("   I   - Enable sim on MAV.");
        System.out.println("   H   - Toggle HUD overlay.");
        System.out.println("   C   - Clear all messages on HUD.");
        System.out.println("   R   - Toggle data report sidebar.");
        System.out.println("   T   - Toggle data report updates.");
        System.out.println("   D   - Toggle sensor parameter control sidebar.");
        System.out.println("   F1  - Show this key commands reference.");
        System.out.println("   P   - Pause simulation.");
        System.out.println("  ESC  - Exit jMAVSim.");
        System.out.println(" SPACE - Reset vehicle & view to start position.");
        System.out.println("");
        System.out.println("Manipulate Vehicle:");
        System.out.println("  ARROW KEYS      - Rotate around pitch/roll.");
        System.out.println("  END/PG-DN       - Rotate CCW/CW around yaw.");
        System.out.println("  SHIFT + ARROWS  - Move N/S/E/W.");
        System.out.println("  SHIFT + INS/DEL - Move Up/Down.");
        System.out.println("  NUMPAD 8/2/4/6  - Start/increase rotation rate around pitch/roll axis.");
        System.out.println("  NUMPAD 1/3      - Start/increase rotation rate around yaw axis.");
        System.out.println("  NUMPAD 5        - Stop all rotation.");
        System.out.println("  CTRL + NUMPAD 5 - Reset vehicle attitude, velocity, & accelleration.");
        System.out.println("");
        System.out.println("Manipulate Environment:");
        System.out.println(" ALT +");
        System.out.println("  ARROW KEYS      - Increase wind deviation in N/S/E/W direction.");
        System.out.println("  INS/DEL         - Increase wind deviation in Up/Down direction.");
        System.out.println("  NUMPAD 8/2/4/6  - Increase wind speed in N/S/E/W direction.");
        System.out.println("  NUMPAD 7/1      - Increase wind speed in Up/Down direction.");
        System.out.println("  NUMPAD 5        - Stop all wind and deviations.");
        System.out.println("");
        System.out.println(" CTRL+ Manipulate - Rotate/move/increase at a higher/faster rate.");
        System.out.println("");
    }

}
