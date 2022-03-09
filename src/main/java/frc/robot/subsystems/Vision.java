package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public abstract class Vision
{
    private static class k
    {
        private static final int FRONT_ID = 0; // temp
        private static final int BACK_ID = 1;   // temp
        private static final String FRONT_NAME = "Intake Camera";
        private static final String BACK_NAME = "Conveyor Camera";
    }

    public static enum Display
    {
        INTAKE,
        CONVEYOR;
    }

    // ===== MEMBERS ===== //

    private static UsbCamera frontCam = new UsbCamera(k.FRONT_NAME, k.FRONT_ID);
    private static UsbCamera backCam = new UsbCamera(k.BACK_NAME, k.BACK_ID);
    private static Vision.Display _Display;
    private static NetworkTableEntry camSelect;
    

    // ===== METHODS ===== //

    public static void init()
    {
        frontCam = CameraServer.startAutomaticCapture(k.FRONT_ID);
        backCam = CameraServer.startAutomaticCapture(k.BACK_ID);

        frontCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        backCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        camSelect = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

        frontCam.setResolution(100, 100);
        backCam.setResolution(100, 100);
    }

    /**
     * @return title for the display
     */
    public static String toggle()
    {
        if (_Display != Display.INTAKE)
        {
            camSelect.setString(frontCam.getName());
            _Display = Display.INTAKE;
            return k.FRONT_NAME;
        }
        else
        {
            camSelect.setString(backCam.getName());
            _Display = Display.CONVEYOR;
            return k.BACK_NAME;
        }
    }
}