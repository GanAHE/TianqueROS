package me.drton.jmavsim;

//import com.sun.j3d.utils.geometry.Box;
//import com.sun.j3d.utils.geometry.Cylinder;
import com.sun.j3d.utils.geometry.Sphere;
import com.sun.j3d.utils.image.ImageException;
import com.sun.j3d.utils.image.TextureLoader;
import com.sun.j3d.utils.universe.SimpleUniverse;
import me.drton.jmavsim.vehicle.AbstractVehicle;

import javax.imageio.ImageIO;
import javax.media.j3d.*;
import javax.swing.*;
import javax.vecmath.*;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.RoundRectangle2D;
import java.awt.image.BufferedImage;
//import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.URL;
import java.util.List;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Enumeration;
import java.util.Map;
import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;

/**
 * 3D Visualizer, works in own thread, synchronized with "world" thread.
 */
public class Visualizer3D extends JFrame {
    public static enum ViewTypes { VIEW_STATIC, VIEW_FPV, VIEW_GIMBAL }
    public static enum ZoomModes { ZOOM_NONE, ZOOM_DYNAMIC, ZOOM_FIXED }
    public static final double PI_2 = Math.PI / 2d;

    public static final String    TEX_DIR = "environment/";  // folder for all environment textures

    public static final String    SKY_TEXTURE = "HDR_040_Field_Bg.jpg";
    //public static final String    SKY_TEXTURE = "HDR_111_Parking_Lot_2_Bg.jpg";
    // the following has a lower resolution and reduces memory usage
    public static final String    SKY_TEXTURE_LOW_RES = "earth3.jpg";

    public static final String    GND_TEXTURE = "grass3.jpg";
    //public static final String    GND_TEXTURE = "ground.jpg";
    public static final String    COMPASS_IMG = "compass_rose.png";  // for overlay HUD
    public static final Dimension WINDOW_SIZE = new Dimension(1024,
                                                              768);  // default application window size
    public static final float     WORLD_SIZE = 5000.0f;  // [m] size of world sphere
    public static final boolean   AA_ENABLED = true;  // default antialising for 3D scene
    public static final ViewTypes VIEW_TYPE  = ViewTypes.VIEW_STATIC;  // default view type
    public static final ZoomModes ZOOM_MODE  = ZoomModes.ZOOM_DYNAMIC;  // default zoom type
    public static final int       FPS_TARGET = 60;  // target frames per second

    private Dimension reportPanelSize = new Dimension(Math.min(WINDOW_SIZE.width / 2, 350), 200);
    private Dimension sensorParamPanelSize = new Dimension(Math.min(WINDOW_SIZE.width / 2, 250), 200);
    private boolean reportPaused = false;

    private int overlaySize = 260;  // width & height of compass overlay window
    private boolean showOverlay = true;

    private double defaultFOV = Math.PI / 3;  // field of view
    private float defaultDZDistance =
        25.0f;  // [m] distance to object at which dynamic zoom is activated
    private float manZoomStep = 0.1f;  // manual zoom steps as fraction of current zoom level
    private Vector3d viewerGroundOffset = new Vector3d(-5.0, 0.0,
                                                       -1.7);  // origin of ground-based fixed view


    private final World world;
    private double currentFOV = defaultFOV;
    private float dynZoomDistance = defaultDZDistance;
    private ViewTypes viewType;
    private ZoomModes zoomMode;
    private Vector3d viewerPosition = new Vector3d();
    private Vector3d viewerPositionOffset = new Vector3d();
    private Transform3D viewerTransform = new Transform3D();
    private SimpleUniverse universe;
    private View view;
    private Canvas3D canvas;
    private BoundingSphere sceneBounds;
    private TransformGroup viewerTransformGroup;
    private KinematicObject viewerTargetObject;
    private KinematicObject viewerPositionObject;
    private AbstractVehicle vehicleViewObject;
    private KinematicObject gimbalViewObject;
    private MAVLinkHILSystem hilSystem;
    private Simulator simulator;
    private JSplitPane splitPane;
    private ReportPanel reportPanel;
    private JSplitPane propertySplitPane;
    private SensorParamPanel sensorParamPanel;
    private KeyboardHandler keyHandler;
    private OutputStream outputStream;  // for receiving system output messages
    private MessageOutputStream msgOutputStream;  // for logging messages
    private Matrix3d tmp_m3d1 = new Matrix3d();  // for calculations
    private Matrix3d tmp_m3d2 = new Matrix3d();
    private Vector3d tmp_v3d = new Vector3d();
    private BranchGroup tmp_bGrp;
    private static final long serialVersionUID = 1L;

    public Visualizer3D(World world) {
        this.world = world;

        keyHandler = new KeyboardHandler();
        msgOutputStream = new MessageOutputStream();
        outputStream = msgOutputStream;
//        outputStream = new BufferedOutputStream(msgOutputStream);

        // load window position & size
        Dimension size = WINDOW_SIZE;
        Rectangle sizeBounds = getWindowBoundsAllScreens();
        Preferences root = Preferences.userRoot();
        final Preferences node = root.node("/me/drton/jmavsim");
        int left = node.getInt("left", 0);
        int top = node.getInt("top", 0);
        size.width = node.getInt("width", size.width);
        size.height = node.getInt("height", size.height);
        addWindowListener(new WindowAdapter() {
            private void saveState() {
                // save window location
                node.putInt("left", getX());
                node.putInt("top", getY());
                node.putInt("width", getWidth());
                node.putInt("height", getHeight());
                try {
                    node.flush();
                } catch (BackingStoreException e) {
                    e.printStackTrace();
                }
            }
            public void windowClosing(WindowEvent we) {
                saveState();
            }
            public void windowLostFocus(WindowEvent e) {
                // jmavsim might get killed via ctrl-c, so save state now
                saveState();
            }
            public void windowDeactivated(WindowEvent e) {
                // jmavsim might get killed via ctrl-c, so save state now
                saveState();
            }
        });
        // check bounds
        size.width = Math.min(size.width, sizeBounds.width);
        left = Math.max(left, sizeBounds.x);
        if (left + size.width > sizeBounds.x + sizeBounds.width) {
            left = sizeBounds.x + sizeBounds.width - size.width;
        }
        size.height = Math.min(size.height, sizeBounds.height);
        top = Math.max(top, sizeBounds.y);
        if (top + size.height > sizeBounds.y + sizeBounds.height) {
            top = sizeBounds.y + sizeBounds.height - size.height;
        }
        setBounds(left, top, size.width, size.height);

        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setTitle("jMAVSim");

        splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
        splitPane.setOneTouchExpandable(false);
        splitPane.setContinuousLayout(true);
        splitPane.setFocusable(false);

        propertySplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
        propertySplitPane.setOneTouchExpandable(false);
        propertySplitPane.setContinuousLayout(true);
        propertySplitPane.setFocusable(false);
        propertySplitPane.setRightComponent(splitPane);
        propertySplitPane.setDividerSize(0);

        getContentPane().add(propertySplitPane);

        reportPanel = new ReportPanel();
        reportPanel.setFocusable(false);
        reportPanel.setMinimumSize(new Dimension(50, 0));
        reportPanel.setPreferredSize(reportPanelSize);
        splitPane.setLeftComponent(reportPanel);

        // Sensor Parameter Control Panel
        sensorParamPanel = new SensorParamPanel();
        sensorParamPanel.setFocusable(false);
        sensorParamPanel.setMinimumSize(new Dimension(50, 0));
        sensorParamPanel.setPreferredSize(sensorParamPanelSize);
        propertySplitPane.setLeftComponent(sensorParamPanel);
        sensorParamPanel.setVisible(false);

        // 3D graphics canvas
        GraphicsConfiguration gc = SimpleUniverse.getPreferredConfiguration();
        if (showOverlay) {
            canvas = new CustomCanvas3D(gc, size, overlaySize);
        } else {
            canvas = new Canvas3D(gc);
        }
        canvas.setFocusable(false);
        canvas.addKeyListener(keyHandler);
        canvas.setMinimumSize(new Dimension(250, 250));
        canvas.setPreferredSize(new Dimension(250, 250));
        splitPane.setRightComponent(canvas);

        universe = new SimpleUniverse(canvas);
        view = universe.getViewer().getView();
        view.setMinimumFrameCycleTime(1000 / FPS_TARGET);
        view.setBackClipDistance(WORLD_SIZE / 4);
        view.setSceneAntialiasingEnable(AA_ENABLED);
        view.setTransparencySortingPolicy(View.TRANSPARENCY_SORT_GEOMETRY);
        view.setFieldOfView(defaultFOV);
        viewerTransformGroup = universe.getViewingPlatform().getViewPlatformTransform();

        createEnvironment();

        setViewType(VIEW_TYPE);
        setZoomMode(ZOOM_MODE);
        setVisible(true);
        splitPane.resetToPreferredSizes();
        toggleReportPanel(false);
        resetView();
        canvas.requestFocus();
    }

    public Rectangle getWindowBoundsAllScreens() {
        GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
        Rectangle virtualBounds =
            GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
        GraphicsDevice[] gs = ge.getScreenDevices();
        for (int j = 0; j < gs.length; j++) {
            GraphicsDevice gd = gs[j];
            GraphicsConfiguration[] gc = gd.getConfigurations();
            for (int i = 0; i < gc.length; i++) {
                virtualBounds = virtualBounds.union(gc[i].getBounds());
            }
        }
        return virtualBounds;
    }

    public void addWorldModels() {
        // add any models in World
        for (WorldObject object : world.getObjects()) {
            if (object instanceof KinematicObject) {
                BranchGroup bg = ((KinematicObject) object).getBranchGroup();
                if (bg != null) {
                    bg.compile();
                    universe.addBranchGraph(bg);
                }
            }
        }
    }

    private void createEnvironment() {
        BranchGroup group = new BranchGroup();
        sceneBounds = new BoundingSphere(new Point3d(0.0, 0.0, 0.0), WORLD_SIZE);
        float grndLevel = (float)world.getEnvironment().getGroundLevel();
        double ground_offset = grndLevel + 0.005;

        Texture2D tex;
        Transform3D trans;
        TransformGroup tg;
        Matrix3d rot = new Matrix3d();
        rot.rotX(PI_2);

        // Sky
        Sphere skySphere = new Sphere(1.0f, Sphere.GENERATE_NORMALS_INWARD | Sphere.GENERATE_TEXTURE_COORDS,
                                      36);
        Map propMap = canvas.queryProperties();
        String driverVendor = (String) propMap.get("native.vendor");
        // The VMware graphics driver has a bug, where the large sky texture just shows up white,
        // without any other errors. The reported maximum texture size is not useful either, it's 16384.
        if (driverVendor.equals("VMware, Inc.")) {
            tex = loadTexture(TEX_DIR + SKY_TEXTURE_LOW_RES);
        } else {
            tex = loadTexture(TEX_DIR + SKY_TEXTURE);
        }
        skySphere.getAppearance().setTexture(tex);
        trans = new Transform3D();
        Matrix3d rotSky = new Matrix3d();
        rotSky.rotZ(-120d * Math.PI / 180d);
        rotSky.mul(rot);
        trans.setRotation(rotSky);
        tg = new TransformGroup(trans);
        tg.addChild(skySphere);

        // Background (sky)
        Background bg = new Background();
        bg.setApplicationBounds(sceneBounds);
        bg.setColor(0, 0, 0.639f); // dark blue
        BranchGroup backGeoBranch = new BranchGroup();
        backGeoBranch.addChild(tg);
        bg.setGeometry(backGeoBranch);
        group.addChild(bg);

        // Ground
        group.addChild(createFlatFloor(ground_offset));
//      group.addChild(createMultiFloor(ground_offset));

//        // cylinder-as-floor attempt, but isn't blending right with transparent overlay
//        trans = new Transform3D();
//        trans.setRotation(rot);
//        trans.transform(new Vector3d(0.0, 0.0, ground_offset));
//        tg = new TransformGroup(trans);
//        tg.addChild(createFlatFloor(ground_offset));
//        group.addChild(tg);


        // Compass rose on ground
//      CompassRose rose = new CompassRose(world, 25.0f);
//      rose.setPositionOffset(new Vector3d(viewerGroundOffset.x, viewerGroundOffset.y, ground_offset - 0.05));
//      group.addChild(rose.getBranchGroup());

        // Light
        DirectionalLight light1 = new DirectionalLight(new Color3f(1.0f, 1.0f, 1.0f), new Vector3f(4.0f,
                                                       7.0f, 12.0f));
        light1.setInfluencingBounds(sceneBounds);
        group.addChild(light1);
        AmbientLight light2 = new AmbientLight(new Color3f(0.9f, 0.9f, 0.9f));
        light2.setInfluencingBounds(sceneBounds);
        group.addChild(light2);

        // Update behavior
        Behavior b = new UpdateBehavior();
        b.setSchedulingBounds(sceneBounds);
        group.addChild(b);
        group.compile();
        universe.addBranchGraph(group);
    }

    private Shape3D createFlatFloor(double height) {
        Appearance ap = new Appearance();
        double side = WORLD_SIZE * 2.0;
        double dZ = height;
        Texture2D tex;
        float tiles = 1.0f;

        tex = loadTexture(TEX_DIR + GND_TEXTURE);
        if (tex != null) {
            tiles = (int)(WORLD_SIZE / tex.getWidth()) * 800;
        }

        QuadArray plane = new QuadArray(4,  GeometryArray.COORDINATES | GeometryArray.TEXTURE_COORDINATE_2);
        plane.setCoordinate(0, new Point3d(-side, side, dZ));
        plane.setCoordinate(1, new Point3d(side, side, dZ));
        plane.setCoordinate(2, new Point3d(side, -side, dZ));
        plane.setCoordinate(3, new Point3d(-side, -side, dZ));
        plane.setTextureCoordinate(0, 0, new TexCoord2f(0.0f, 0.0f));
        plane.setTextureCoordinate(0, 1, new TexCoord2f(tiles, 0.0f));
        plane.setTextureCoordinate(0, 2, new TexCoord2f(tiles, tiles));
        plane.setTextureCoordinate(0, 3, new TexCoord2f(0.0f, tiles));

        // for cylinder
//      tex.setBoundaryModeT(Texture.WRAP);
//      tex.setBoundaryModeS(Texture.WRAP);
//      Transform3D trans = new Transform3D();
//      trans.setScale(worldExtent / tex.getWidth());
//      TextureAttributes texat = new TextureAttributes();
//      texat.setTextureTransform(trans);
//      texat.setTextureMode(TextureAttributes.REPLACE);
//      texat.setPerspectiveCorrectionMode(TextureAttributes.NICEST);
//      ap.setTextureAttributes(texat);

        ap.setTexture(tex);

        //return new Cylinder(worldExtent, 0.001f, Cylinder.GENERATE_TEXTURE_COORDS | Cylinder.GENERATE_NORMALS, 8, 4, ap);
        return new Shape3D(plane, ap);
    }

    /*
    private float[][] heights;  // height map for the floor
    // the floor is a multi-textured mesh, with splashes of extra textures
    private OrderedGroup createMultiFloor() {
        MultiFloor floor = new MultiFloor(TEX_DIR + "grass.gif", 4, TEX_DIR + "stoneBits.gif", 2);
        // the ground detail textures are grass and bits of stone
        //   the frequencies (4, 2) should divide into the floor length
        //   (FLOOR_LEN (20) in MultiFloor) with no remainder

        heights = floor.getHeightMap();

        // Start building an ordered group of floor meshes.
         Ordering avoids rendering conflicts between the meshes.
        OrderedGroup floorOG = new OrderedGroup();
        floorOG.addChild(floor);

        // load the textures for the splashes
        Texture2D flowersTex = loadTexture(TEX_DIR + "flowers.jpg");
        Texture2D waterTex = loadTexture(TEX_DIR + "water.jpg");

        // add splashes
        for(int i=0; i < 8; i++)
            floorOG.addChild( new SplashShape(flowersTex, heights) );

        for (int i=0; i < 3; i++)
            floorOG.addChild( new SplashShape(waterTex, heights) );

        // return all the meshes
        return floorOG;
    }
    */

    // load image from file as a texture
    private Texture2D loadTexture(String fn) {
        System.gc(); // cleanup memory before loading the texture
        TextureLoader texLoader = null;
        Texture2D texture = new Texture2D();
        texture.setEnable(false);
        try {
            texLoader = new TextureLoader(fn, null);
            // enable Mipmapping (increases memory usage considerably)
            //texLoader = new TextureLoader(fn, TextureLoader.GENERATE_MIPMAP, null);
        }  catch (ImageException e) {
            System.out.println("Error, could not load texture: " + fn);
            System.out.println("Error message:" + e.getLocalizedMessage());
        }
        if (texLoader != null) {
            texture = (Texture2D) texLoader.getTexture();
            if (texture == null) {
                System.out.println("Cannot load texture from " + fn);
            } else {
                //System.out.println( "\t\tNumber Of MIPMAPS->" + texture.numMipMapLevels() );
                texture.setMinFilter(Texture.NICEST);
                texture.setMagFilter(Texture.NICEST);
                texture.setAnisotropicFilterMode(texture.ANISOTROPIC_SINGLE_VALUE);
                texture.setAnisotropicFilterDegree(4.f);
                //System.out.println("Loaded texture from " + fn);
                texture.setEnable(true);
            }
        }
        return texture;
    }


    /**
     * Target object to point camera, has effect only if viewerPositionObject is not set.
     *
     * @param object
     */
    public void setViewerTargetObject(KinematicObject object) {
        this.viewerTargetObject = object;
    }

    /**
     * Object to place camera on, if nullptr then camera will be placed in fixed point set by setViewerPosition().
     *
     * @param object
     */
    public void setViewerPositionObject(KinematicObject object) {
        this.viewerPositionObject = object;
        if (object != null && zoomMode == ZoomModes.ZOOM_DYNAMIC) {
            nextZoomMode();
        }
    }

    /**
     * Fixed camera position, has effect only if viewerPositionObject not set.
     *
     * @param position
     */
    public void setViewerPosition(Vector3d position) {
        this.viewerPositionObject = null;
        this.viewerPosition = position;
        viewerTransform.setTranslation(viewerPosition);
    }

    /**
     * Camera position offset from object position when viewer placed on some object
     *
     * @param offset position offset
     */
    public void setViewerPositionOffset(Vector3d offset) {
        this.viewerPositionOffset = offset;
    }

    /**
     * Set the "vehicle" object to use for switching views.
     *
     * @param object
     */
    public void setVehicleViewObject(AbstractVehicle object) {
        this.vehicleViewObject = object;
    }

    /**
     * Set the "gimbal" object to use for switching views.
     *
     * @param object
     */
    public void setGimbalViewObject(KinematicObject object) {
        this.gimbalViewObject = object;
    }

    /**
     * Set the system being controlled.
     *
     * @param system
     */
    public void setHilSystem(MAVLinkHILSystem system) {
        this.hilSystem = system;
    }

    /**
     * Set the simulator being ran.
     *
     * @param simulator
     */
    public void setSimulator(Simulator simulator) {
        this.simulator = simulator;
    }

    /**
     * Sets the text of the simulation report.
     *
     * @param text
     */
    public void setReportText(String text) {
        if (showReportText()) {
            reportPanel.setText(text);
        }
    }

    /**
     * Check whether to show the report text
     */
    public boolean showReportText() {
        return reportPanel.isShowing() && !reportPaused;
    }

    /**
     * Show/hide the simulation report.
     *
     * @param text
     */
    public void toggleReportPanel(boolean on) {
        if (reportPanel == null || (on && reportPanel.isShowing()) || (!on && !reportPanel.isShowing())) {
            return;
        }

        setReportPaused(!on);
        if (reportPanel.isShowing()) {
            reportPanelSize = reportPanel.getSize();
            splitPane.setLeftComponent(null);
            splitPane.setDividerSize(0);
        } else {
            reportPanel.setPreferredSize(reportPanelSize);
            splitPane.setLeftComponent(reportPanel);
            splitPane.setDividerSize((int)UIManager.get("SplitPane.dividerSize"));
        }

        splitPane.resetToPreferredSizes();
        revalidate();
    }

    public void toggleSensorControlDialog() {
        if (sensorParamPanel == null || vehicleViewObject == null) {
            return;
        } else if (this.sensorParamPanel.isShowing()) {
            sensorParamPanel.setSensor(vehicleViewObject.getSensors());
            sensorParamPanel.setVisible(false);
            propertySplitPane.setLeftComponent(null);
            propertySplitPane.setDividerSize(0);
        } else {
            sensorParamPanel.setSensor(vehicleViewObject.getSensors());
            sensorParamPanel.setVisible(true);
            propertySplitPane.setLeftComponent(sensorParamPanel);
        }

        propertySplitPane.resetToPreferredSizes();
        revalidate();
    }

    public void toggleReportPanel() {
        this.toggleReportPanel(!reportPanel.isShowing());
    }

    /**
     * Toggles updates of the report panel text.
     *
     * @param pause
     */
    public void setReportPaused(boolean pause) {
        reportPaused = pause;
        reportPanel.setIsFocusable(pause);
        if (pause) {
            ReportUpdater.setUpdateFreq(0L);
        } else {
            ReportUpdater.resetUpdateFreq();
        }
    }

    public void setShowOverlay(boolean showOverlay) {
        this.showOverlay = showOverlay;
    }


    /**
     * Toggles scene renderer antialiasing on/off
     */
    public void setAAEnabled(boolean enable) {
        view.setSceneAntialiasingEnable(enable);
        if (showOverlay) {
            ((CustomCanvas3D)canvas).setAA(enable);
        }
    }


    public OutputStream getOutputStream() {
        return outputStream;
    }

    public void setZoomMode(ZoomModes zoomMode) {
        if (zoomMode == ZoomModes.ZOOM_DYNAMIC && viewType != ViewTypes.VIEW_STATIC) {
            nextZoomMode();
        } else {
            this.zoomMode = zoomMode;
        }
    }

    public void setDynZoomDistance(float dynZoomDistance) {
        if (dynZoomDistance < 0.5f) {
            dynZoomDistance = 0.5f;
        } else {
            double dist = getVectorToTargetObject(viewerPosition, viewerTargetObject).length();
            if (dynZoomDistance > dist + defaultDZDistance) {
                dynZoomDistance = (float)dist + defaultDZDistance;
            }
        }

        this.dynZoomDistance = dynZoomDistance;
    }

    public void setFieldOfView(double fov) {
        fov = Math.max(Math.min(fov, 2.7), 0.001);
        view.setFieldOfView(fov);
        currentFOV = fov;
    }

    public void setViewType(ViewTypes v) {
        switch (v) {
            case VIEW_STATIC :
                // Put camera on static point and point to vehicle
                if (this.viewType != ViewTypes.VIEW_STATIC && vehicleViewObject != null) {
                    this.viewType = ViewTypes.VIEW_STATIC;
                    Vector3d pos = new Vector3d(viewerGroundOffset);
                    pos.z = (pos.z + world.getEnvironment().getGroundLevel());
                    this.setViewerPosition(pos);
                    this.setViewerTargetObject(vehicleViewObject);
                }
                break;

            case VIEW_FPV :
                // Put camera on vehicle (FPV)
                if (this.viewType != ViewTypes.VIEW_FPV && vehicleViewObject != null) {
                    this.viewType = ViewTypes.VIEW_FPV;
                    this.setViewerPositionObject(vehicleViewObject);
                    this.setViewerPositionOffset(new Vector3d(-0.0f, 0.0f, -0.3f));   // Offset from vehicle center
                }
                break;

            case VIEW_GIMBAL :
                if (this.viewType != ViewTypes.VIEW_GIMBAL && gimbalViewObject != null) {
                    this.viewType = ViewTypes.VIEW_GIMBAL;
                    this.setViewerPositionObject(gimbalViewObject);
                    this.setViewerPositionOffset(new Vector3d(0.0f, 0.0f, 0.0f));
                } else {
                    System.out.println("Unable to set view, gimbal not mounted.");
                }
                break;
        }
    }

    private void nextZoomMode() {
        if (zoomMode == ZoomModes.ZOOM_NONE && viewType == ViewTypes.VIEW_STATIC) {
            zoomMode = ZoomModes.ZOOM_DYNAMIC;
        } else if (zoomMode == ZoomModes.ZOOM_FIXED) {
            zoomMode = ZoomModes.ZOOM_NONE;
            view.setFieldOfView(defaultFOV);
        } else  {
            zoomMode = ZoomModes.ZOOM_FIXED;
            view.setFieldOfView(currentFOV);
        }
    }

    public void resetView() {
        tmp_m3d1.rotZ(Math.PI);
        tmp_m3d2.rotY(PI_2);
        tmp_m3d1.mul(tmp_m3d2);
        tmp_m3d2.rotZ(-PI_2);
        tmp_m3d1.mul(tmp_m3d2);
        viewerTransform.setRotation(tmp_m3d1);
    }

    public Vector3d getVectorToTargetObject(Vector3d from, KinematicObject objTo) {
        Vector3d ret = new Vector3d();
        ret.sub(objTo.getPosition(), from);
        return ret;
    }

    private void updateVisualizer() {
        double dist;
        synchronized (world) { // Synchronize with "world" thread
            try {
                // Update branch groups of all kinematic objects
                for (WorldObject object : world.getObjects()) {
                    if (object instanceof KinematicObject) {
                        tmp_bGrp = ((KinematicObject) object).getBranchGroup();
                        if (tmp_bGrp != null) {
                            ((KinematicObject) object).updateBranchGroup();
                        }
                    }
                }
                // Update view platform
                if (viewerPositionObject != null) {
                    // Camera on object
                    viewerPosition.set(viewerPositionOffset);
                    viewerPositionObject.getRotation().transform(viewerPosition);
                    viewerPosition.add(viewerPositionObject.getPosition());
                    viewerTransform.setTranslation(viewerPosition);

                    tmp_m3d1.set(viewerPositionObject.getRotation());
                    tmp_m3d2.rotZ(PI_2);
                    tmp_m3d1.mul(tmp_m3d2);
                    tmp_m3d2.rotX(-PI_2);
                    tmp_m3d1.mul(tmp_m3d2);
                    viewerTransform.setRotation(tmp_m3d1);
                } else if (viewerTargetObject != null) {
                    // Fixed-position camera, point camera to target
                    tmp_v3d = viewerTargetObject.getPosition();
                    dist = getVectorToTargetObject(viewerPosition, viewerTargetObject).length();

                    tmp_m3d1.rotZ(Math.PI);
                    tmp_m3d2.rotY(PI_2);
                    tmp_m3d1.mul(tmp_m3d2);
                    tmp_m3d2.rotZ(-PI_2);
                    tmp_m3d1.mul(tmp_m3d2);
                    tmp_m3d2.rotY(-Math.atan2(tmp_v3d.y - viewerPosition.y, tmp_v3d.x - viewerPosition.x));
                    tmp_m3d1.mul(tmp_m3d2);
                    tmp_m3d2.rotX(-Math.asin((tmp_v3d.z - viewerPosition.z) / dist));
                    tmp_m3d1.mul(tmp_m3d2);
                    viewerTransform.setRotation(tmp_m3d1);

                    if (zoomMode == ZoomModes.ZOOM_DYNAMIC) {
                        if (dist > dynZoomDistance) {
                            view.setFieldOfView(dynZoomDistance / dist * currentFOV);
                        } else {
                            view.setFieldOfView(currentFOV);
                        }
                    }
                }
                viewerTransformGroup.setTransform(viewerTransform);
            } catch (BadTransformException e) {
                e.printStackTrace();
            }
        }
    }

    /*
     * Reset Rotation, Acceleration, Velocity
     */
    private void resetObjectRAV(KinematicObject obj, boolean resetPos) {
        if (obj == null) {
            return;
        }
        Vector3f oldpos = new Vector3f(obj.getPosition());
        obj.resetObjectParameters();
        if (!resetPos) {
            moveObject(obj, oldpos, true);
        } else {
            obj.setIgnoreGravity(false);
        }
    }

    /*
     * Rotate object in steps
     */
    private void rotateObject(KinematicObject obj, Vector3f vec, float deg) {
        if (obj == null) {
            return;
        }
        Matrix3d rot = obj.getRotation();
        Matrix3d r = new Matrix3d();
        if (vec == null) {
            r.rotZ(0.0);
        } else {
            AxisAngle4f aa = new AxisAngle4f(vec, (float)Math.toRadians(deg));
            r.set(aa);
        }
        rot.mulNormalize(r);
    }

    /*
     * Set a continuous rotation rate of an object
     */
    private void spinRateObject(KinematicObject obj, Vector3f vec) {
        if (obj == null) {
            return;
        }
        if (vec == null) {
            obj.setRotationRate(new Vector3d());
        } else {
            // if still on ground, move it up so it can rotate
            if (obj.getPosition().z >= 0) {
                moveObject(obj, new Vector3f(0f, 0f, -2.0f), false);
            }
            obj.getRotationRate().add(new Vector3d(vec));
        }
    }

    /*
     * Change position of an object
     */
    private void moveObject(KinematicObject obj, Vector3f vec, boolean absolute) {
        if (obj == null) {
            return;
        }
        Vector3d pos = obj.getPosition();
        if (absolute) {
            pos.set(vec);
        } else {
            pos.add(new Vector3d(vec));
        }
        obj.setIgnoreGravity(pos.z < 0.0);
//        if (pos.z >= 0.0)
//            //world.getEnvironment().setG(null);
//        else
//            world.getEnvironment().setG(new Vector3d());
    }

    /*
     * Manipulate wind in environment
     */
    private void windDirection(Vector3f vec, boolean setBase, boolean setCurrent,
                               boolean setDeviation) {
        if (vec == null) {
            if (setBase) {
                world.getEnvironment().setWind(new Vector3d());
            }
            if (setCurrent) {
                world.getEnvironment().setCurrentWind(new Vector3d());
                System.out.println("Wind reset to zero.");
            }
            if (setDeviation) {
                world.getEnvironment().setWindDeviation(new Vector3d());
                System.out.println("Wind deviation reset to zero.");
            }
        } else {
            Vector3d adj = new Vector3d(vec);
            if (setBase) {
                world.getEnvironment().getWind().add(adj);
            }
            if (setCurrent) {
                world.getEnvironment().getCurrentWind(null).add(adj);
                System.out.println("Wind vector is now " + ReportUtil.vector2str(
                                       world.getEnvironment().getCurrentWind(viewerPosition)));
            }
            if (setDeviation) {
                world.getEnvironment().world.getEnvironment().getWindDeviation().add(adj);
                System.out.println("Wind deviation is now " + ReportUtil.vector2str(
                                       world.getEnvironment().getWindDeviation()));
            }
        }
    }


    //
    //// private Classes
    //

    /*
     * Custom Canvas class for drawing optional overlay HUD
     */
    private class CustomCanvas3D extends Canvas3D {
        private static final long serialVersionUID = 7144426579917281131L;

        private int[] overlayMargins = {10, 10};  // x, y from left/right bottom corner
        private Font font = new Font("SansSerif", Font.BOLD, 14);
        private Color txtColor = Color.white;
        private Color hdgColor = Color.magenta;
        private Color crsColor = Color.green;
        private Color windColor = Color.blue;
        // system messages overlay
        private Font msgFont = new Font("SansSerif", Font.PLAIN, 14);
        private Color msgColor = new Color(255, 255, 255, 240);
        private Color msgBgColor = new Color(202, 162, 0, 60);

        private BufferedImage compassOverlay;
        private J3DGraphics2D g2d;
        private Matrix3d m1 = new Matrix3d();
        private AffineTransform affTrans = new AffineTransform();
        private BufferedImage drawImg;
        private Graphics2D drawg2d;
        private Line2D.Float hdgLine;
        private Line2D.Float crsLine;
        private Line2D.Float windLine;
        private BasicStroke crsStroke = new BasicStroke(2.5f);  // drawn last, on top
        private BasicStroke hdgStroke = new BasicStroke(4.0f);
        private BasicStroke wndStroke = new BasicStroke(5.5f);  // drawn first, on bottom
        private RoundRectangle2D.Float msgBg = new RoundRectangle2D.Float();
        private int[] overlaySize = new int[2];  // x, y
        private int[] messagesSize = {450, 600};  // x, y
        private int msgLineHeight = 15;
        private int halfW;
        private int fps = 1;
        private int framesCount = 0;
        private long frameTime = 0L;

        public CustomCanvas3D(GraphicsConfiguration gc, Dimension windowSize, int overlayWidth) {
            super(gc);
            g2d = this.getGraphics2D();

            setAA(AA_ENABLED);

            // constrain overlay sizes
            if (overlayWidth > windowSize.getWidth() / 2) {
                overlayWidth = (int)(windowSize.getWidth() / 2);
            }
            if (overlayWidth + 45 > windowSize.getHeight() / 2) {
                overlayWidth = (int)(windowSize.getHeight() / 2);
            }
            if (messagesSize[0] > windowSize.getWidth() / 2) {
                messagesSize[0] = (int)(windowSize.getWidth() / 2);
            }
            if (messagesSize[1] > windowSize.getHeight() * 0.75) {
                messagesSize[1] = (int)(windowSize.getHeight() * 0.75);
            }

            overlaySize[0] = overlayWidth;
            overlaySize[1] = overlayWidth + 45;
            halfW = overlayWidth / 2;
            frameTime = System.nanoTime();

            // drawing surface for vector lines
            drawImg = new BufferedImage(overlayWidth, overlayWidth, BufferedImage.TYPE_4BYTE_ABGR);
            drawg2d = drawImg.createGraphics();

            // load and scale compass image for overlay
            URL file = null;
            compassOverlay = new BufferedImage(overlayWidth, overlayWidth, BufferedImage.TYPE_4BYTE_ABGR);

            try {
                file = new URL("file:./" + TEX_DIR + COMPASS_IMG);
                if (file != null) {
                    Image img = ImageIO.read(file);
                    img = img.getScaledInstance(overlayWidth, overlayWidth, Image.SCALE_SMOOTH);
                    compassOverlay.createGraphics().drawImage(img,  0,  0, null);
                }
            } catch (IOException e) {
                System.out.println("Error, could not load image: " + TEX_DIR + COMPASS_IMG);
                System.out.println("Error message:" + e.getLocalizedMessage());
            }

            // set up vector lines for HUD
            hdgLine = new Line2D.Float(0, 0, 0, halfW * -0.85f);
            crsLine = new Line2D.Float(0, 0, 0, halfW * -0.425f);
            windLine = new Line2D.Float(0, 0, 0, halfW * -0.425f);

        }

        public void setAA(boolean on) {
            if (on) {
                g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
                g2d.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION,
                                     RenderingHints.VALUE_ALPHA_INTERPOLATION_QUALITY);
                g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BICUBIC);
            } else {
                g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_OFF);
                g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_SPEED);
                g2d.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION,
                                     RenderingHints.VALUE_ALPHA_INTERPOLATION_SPEED);
                g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
            }
        }

        // we draw the HUD/overlay here
        public void postRender() {
            if (!showOverlay) {
                return;
            }

            int x = overlayMargins[0];
            int y = this.getHeight() - overlaySize[1] - overlayMargins[1];
            double z, dZ, norm;
            Vector3d vect;

            clearDrawing();

            // compass rotation in relation to viewer
            viewerTransform.get(m1);
            dZ = -Math.atan2(m1.getElement(1, 0), m1.getElement(0, 0)) + Math.toRadians(90.0);
            affTrans.setToRotation(dZ, halfW, halfW);
            drawg2d.setTransform(affTrans);
            drawg2d.drawImage(compassOverlay, 0, 0, this);

            // wind line in relation to viewer
            vect = world.getEnvironment().getCurrentWind(viewerPosition);
            norm = Math.sqrt(vect.x * vect.x + vect.y * vect.y);
            affTrans.setToTranslation(halfW, halfW);
            affTrans.rotate(vect.x, vect.y);
            affTrans.rotate(dZ);
            // scale length and width based on wind speed
            affTrans.scale(Math.max(Math.min(Math.abs(vect.z) * 0.5, 10.0), 1.0), Math.min(norm * 0.2,
                                                                                           halfW * 0.85));
            drawg2d.setTransform(affTrans);
            drawg2d.setColor(windColor);
            drawg2d.setStroke(wndStroke);
            drawg2d.draw(windLine);

            if (vehicleViewObject != null) {
                // heading line
                m1 = (Matrix3d) vehicleViewObject.getRotation().clone();
                z = Math.atan2(m1.getElement(1, 0), m1.getElement(0, 0));
                affTrans.setToTranslation(halfW, halfW);
                affTrans.rotate(z + dZ);
                drawg2d.setTransform(affTrans);
                drawg2d.setColor(hdgColor);
                drawg2d.setStroke(hdgStroke);
                drawg2d.draw(hdgLine);

                // course over ground line
                vect = vehicleViewObject.getVelocity();
                z = Math.atan2(vect.y, vect.x);
                norm = Math.sqrt(vect.x * vect.x + vect.y * vect.y);
                affTrans.setToTranslation(halfW, halfW);
                affTrans.rotate(z + dZ);
                // scale length and width based on vehicle speed
                affTrans.scale(Math.max(Math.min(Math.abs(vect.z) * 0.5, 10.0), 1.0), Math.min(norm * 0.2,
                                                                                               halfW * 0.85));
                drawg2d.setTransform(affTrans);
                drawg2d.setColor(crsColor);
                drawg2d.setStroke(crsStroke);
                drawg2d.draw(crsLine);
            }

            // now draw the composed compass + vectors image on the main J3DGraphics2D
            g2d.drawImage(drawImg, x, y, this);

            // draw all HUD text items

            g2d.setFont(font);
            g2d.setColor(txtColor);
            y += drawImg.getHeight() + 25;
            String zmode = zoomMode == ZoomModes.ZOOM_NONE ? "Fixed" : zoomMode == ZoomModes.ZOOM_DYNAMIC ?
                           "Dynamic" : "Manual";
            if (zoomMode == ZoomModes.ZOOM_DYNAMIC) {
                zmode += String.format(" @ %.2fm", dynZoomDistance);
            }
            zmode += String.format("    FOV: %.2f\u00b0", Math.toDegrees(view.getFieldOfView()));
            g2d.drawString("Zoom mode: " + zmode, x, y);
            y += 20;
            g2d.drawString(String.format("FPS: %3d", fps), x, y);
            x += 70;
            g2d.setColor(hdgColor);
            g2d.drawString("HDG", x, y);
            x += 40;
            g2d.setColor(crsColor);
            g2d.drawString("CRS", x, y);
            x += 40;
            g2d.setColor(windColor);
            g2d.drawString("WND", x, y);

            // messages on the bottom right
            if (msgOutputStream.getListLen() > 0) {
                x = this.getWidth() - messagesSize[0] - overlayMargins[0];
                int h = Math.min(messagesSize[1], msgOutputStream.getListLen() * msgLineHeight + 5);
                y = this.getHeight() - h - overlayMargins[1];
                msgBg.setRoundRect(x, y, messagesSize[0], h, 15, 15);

                g2d.setColor(msgBgColor);
                g2d.draw(msgBg);
                g2d.fill(msgBg);

                x += 10;
                y += msgLineHeight;
                g2d.setFont(msgFont);
                g2d.setColor(msgColor);
                for (String msg : msgOutputStream.getStrings()) {
                    g2d.drawString(msg, x, y);
                    y += msgLineHeight;
                    if (y > this.getHeight()) {
                        break;
                    }
                }
            }

            g2d.flush(false);

            ++framesCount;
            if (System.nanoTime() - frameTime >= (long)1e9) {
                fps = framesCount;
                framesCount = 0;
                frameTime = System.nanoTime();
            }
        }

        private void clearDrawing() {
            // clear drawing image
            affTrans.setToIdentity();
            drawg2d.setTransform(affTrans);
            drawg2d.setComposite(AlphaComposite.getInstance(AlphaComposite.CLEAR, 0.0f));
            drawg2d.fillRect(0, 0, overlaySize[0], overlaySize[0]);
            drawg2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 1.0f));
            drawg2d.setColor(Color.BLACK);

        }
    }


    /*
     * KeyboardHandler
     */
    public class KeyboardHandler extends KeyAdapter {
        public BitSet keyBits = new BitSet(256);

        @Override
        public void keyReleased(KeyEvent e) {
            keyBits.clear(e.getKeyCode());

            checkCumulativeKeys();

            switch (e.getKeyCode()) {

                // View swtich keys
                case KeyEvent.VK_F :
                    setViewType(ViewTypes.VIEW_FPV);
                    break;

                case KeyEvent.VK_S :
                    setViewType(ViewTypes.VIEW_STATIC);
                    break;

                case KeyEvent.VK_G :
                    setViewType(ViewTypes.VIEW_GIMBAL);
                    break;

                // reporting panel
                case KeyEvent.VK_R :
                    toggleReportPanel();
                    break;

                case KeyEvent.VK_D :
                    toggleSensorControlDialog();
                    break;

                // pause/start report updates
                case KeyEvent.VK_T :
                    setReportPaused(!reportPaused);
                    break;

                // toggle zoom mode fixed/dynamic/manual
                case KeyEvent.VK_Z :
                    nextZoomMode();
                    break;

                // zoom reset
                case KeyEvent.VK_0 :
                case KeyEvent.VK_ENTER :
                    zoomMode = ZoomModes.ZOOM_NONE;
                    setFieldOfView(defaultFOV);
                    setDynZoomDistance(defaultDZDistance);
                    break;

                // init sim mode
                case KeyEvent.VK_I :
                    if (hilSystem != null) {
                        hilSystem.initMavLink();
                    }
                    break;

                // quit sim mode
                case KeyEvent.VK_Q :
                    if (hilSystem != null) {
                        hilSystem.endSim();
                    }
                    break;

                // toggle HUD overlay
                case KeyEvent.VK_H :
                    setShowOverlay(!showOverlay);
                    break;

                // clear messages from HUD
                case KeyEvent.VK_C :
                    msgOutputStream.clearMessages();
                    break;

                // show help text
                case KeyEvent.VK_F1 :
                    msgOutputStream.clearMessages();
                    msgOutputStream.setNumOfMessages(50);
                    Simulator.printKeyCommands();
                    msgOutputStream.resetNumOfMessages();
                    break;

                // Pause simulation
                case KeyEvent.VK_P :
                    simulator.pauseToggle();
                    break;

                // exit app
                case KeyEvent.VK_ESCAPE :
                    dispatchEvent(new WindowEvent(getWindows()[0], WindowEvent.WINDOW_CLOSING));
                    break;

                // full view and object reset
                case KeyEvent.VK_SPACE :
                    resetObjectRAV(vehicleViewObject, true);

                    resetView();
                    break;

                // vehicle object resets
                case KeyEvent.VK_NUMPAD5 :
                    // reset wind
                    if (keyBits.get(KeyEvent.VK_ALT)) {
                        windDirection(null, true, true, true);
                    }
                    // reset vehicle rotation, etc
                    else if (keyBits.get(KeyEvent.VK_CONTROL)) {
                        resetObjectRAV(vehicleViewObject, false);
                    }
                    // reset only rotation rate
                    else {
                        spinRateObject(vehicleViewObject, null);
                    }

                    break;

            }
        }

        @Override
        public void keyPressed(KeyEvent e) {
            keyBits.set(e.getKeyCode());

            checkCumulativeKeys();

            switch (e.getKeyCode()) {

                // zoom in
                case KeyEvent.VK_PLUS :
                case KeyEvent.VK_ADD :
                case KeyEvent.VK_EQUALS :
                    if (zoomMode == ZoomModes.ZOOM_DYNAMIC) {
                        setDynZoomDistance(dynZoomDistance * (1.0f - manZoomStep));
                    } else {
                        zoomMode = ZoomModes.ZOOM_FIXED;
                        setFieldOfView(currentFOV * (1.0f - manZoomStep));
                    }
                    break;

                // zoom out
                case KeyEvent.VK_MINUS :
                case KeyEvent.VK_SUBTRACT :
                    if (zoomMode == ZoomModes.ZOOM_DYNAMIC) {
                        setDynZoomDistance(dynZoomDistance * (1.0f + manZoomStep));
                    } else {
                        zoomMode = ZoomModes.ZOOM_FIXED;
                        setFieldOfView(currentFOV * (1.0f + manZoomStep));
                    }
                    break;
            }


        }

        public void checkCumulativeKeys() {
            // how to move
            Vector3f dir = new Vector3f();
            // how much to move
            float deg = keyBits.get(KeyEvent.VK_CONTROL) ? 5.0f : 1.0f;
            // magnitude of move (rotation magnitude is always 1)
            float m = keyBits.get(KeyEvent.VK_SHIFT) ? deg / 5.0f : 1.0f;

            if (keyBits.get(KeyEvent.VK_LEFT) || keyBits.get(KeyEvent.VK_KP_LEFT)) {
                dir.x = (-m);
            }

            if (keyBits.get(KeyEvent.VK_RIGHT) || keyBits.get(KeyEvent.VK_KP_RIGHT)) {
                dir.x = (m);
            }

            if (keyBits.get(KeyEvent.VK_UP) || keyBits.get(KeyEvent.VK_KP_UP)) {
                dir.y = (-m);
            }

            if (keyBits.get(KeyEvent.VK_DOWN) || keyBits.get(KeyEvent.VK_KP_DOWN)) {
                dir.y = (m);
            }

            if (keyBits.get(KeyEvent.VK_END) || keyBits.get(KeyEvent.VK_INSERT)) {
                dir.z = (-m);
            }

            if (keyBits.get(KeyEvent.VK_PAGE_DOWN) || keyBits.get(KeyEvent.VK_DELETE)) {
                dir.z = (m);
            }

            if (dir.length() != 0.0) {

                if (keyHandler.keyBits.get(KeyEvent.VK_ALT)) {
                    // wind deviation
                    dir.set(-dir.y, dir.x, dir.z);
                    windDirection(dir, false, false, true);
                } else if (keyBits.get(KeyEvent.VK_SHIFT)) {
                    // move vehicle
                    dir.set(-dir.y, dir.x, dir.z);
                    moveObject(vehicleViewObject, dir, false);
                } else
                    // rotate vehicle
                {
                    rotateObject(vehicleViewObject, dir, deg);
                }
            }


            // check for keypad events (rotation rate or wind force)

            if (keyBits.get(KeyEvent.VK_NUMPAD5)) {
                return;
            }

            dir = new Vector3f();
            m = keyHandler.keyBits.get(KeyEvent.VK_CONTROL) ? 1.0f : 0.5f;

            if (keyBits.get(KeyEvent.VK_NUMPAD4)) {
                dir.x = (-m);
            }
            if (keyBits.get(KeyEvent.VK_NUMPAD6)) {
                dir.x = (m);
            }
            if (keyBits.get(KeyEvent.VK_NUMPAD8)) {
                dir.y = (-m);
            }
            if (keyBits.get(KeyEvent.VK_NUMPAD2)) {
                dir.y = (m);
            }
            if (keyBits.get(KeyEvent.VK_NUMPAD1)) {
                dir.z = (-m);
            }
            if (keyBits.get(KeyEvent.VK_NUMPAD3) || keyBits.get(KeyEvent.VK_NUMPAD7)) {
                dir.z = (m);
            }

            if (dir.length() != 0.0) {
                if (keyHandler.keyBits.get(KeyEvent.VK_ALT)) {
                    // wind strength but not deviation
                    dir.set(-dir.y, dir.x, -dir.z);
                    windDirection(dir, true, true, false);
                } else
                    // adjust vehicle spin rate
                {
                    spinRateObject(vehicleViewObject, dir);
                }
            }

        }
    }
    // end KeyboardHandler


    /*
     * Thread updater
     */
    class UpdateBehavior extends Behavior {
        private WakeupCondition condition = new WakeupOnElapsedFrames(0, false);

        @Override
        public void initialize() {
            wakeupOn(condition);
        }

        @Override
        @SuppressWarnings("rawtypes")
        public void processStimulus(Enumeration wakeup) {
            Object w;
            while (wakeup.hasMoreElements()) {
                w = wakeup.nextElement();
                if (w instanceof WakeupOnElapsedFrames) {
                    updateVisualizer();
                }
                wakeupOn(condition);
            }
        }
    }


    /*
     * System message logger
     */
    class MessageOutputStream extends OutputStream {
        private final int strcap = 16;  // number of messages to store
        private final int bufcap = 80; // line length limit
        private int numOfMessages = strcap;
        private final StringBuffer buf = new StringBuffer(bufcap);
        private int buflen = 0;
        private final List<String> strings = new ArrayList<String>(strcap);
        private boolean mtx = false;

        @Override
        public void write(int b) throws IOException {
            char c = (char)b;
            buf.append(c);
            if (c == '\n' || ++buflen >= bufcap) {
                this.flush();
            }
        }

        @Override
        public void flush() {
            if (mtx) { // do not block
                return;
            }
            mtx = true;
            while (strings.size() > numOfMessages) {
                strings.remove(0);
            }

            String line = buf.toString().replaceAll("(.+)[\\r\\n]", "$1");
            if (!line.isEmpty()) {
                strings.add(line);
            }

            buflen = 0;
            buf.setLength(buflen);
            mtx = false;
        }

        public List<String> getStrings() {
            if (mtx) { // do not block
                return new ArrayList<String>();
            }

            return new ArrayList<String>(strings);
        }

        public int getListLen() {
            return strings.size();
        }

        public void clearMessages() {
            if (!mtx) {
                strings.clear();
            }
        }

        public int getNumOfMessages() {
            return numOfMessages;
        }

        public void setNumOfMessages(int numOfMessages) {
            this.numOfMessages = numOfMessages;
        }

        public void resetNumOfMessages() {
            this.numOfMessages = strcap;
        }

    }

}
