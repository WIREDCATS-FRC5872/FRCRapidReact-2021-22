package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public abstract class Cameras
{
    private static class k
    {
        private static final int INTAKE = 0; // temp
        private static final int CONVEYOR = 1;   // temp
        private static final String INTAKE_NAME = "Intake Camera";
        private static final String CONVEYOR_NAME = "Conveyor Camera";
    }

    public static enum Display
    {
        INTAKE,
        CONVEYOR;
    }

    // ===== MEMBERS ===== //

    private static UsbCamera frontCam = new UsbCamera(k.INTAKE_NAME, k.INTAKE);
    private static UsbCamera backCam = new UsbCamera(k.CONVEYOR_NAME, k.CONVEYOR);
    private static Cameras.Display _Display;
    private static NetworkTableEntry camSelect;
    

    // ===== METHODS ===== //

    public static void init()
    {
        frontCam = CameraServer.startAutomaticCapture(k.INTAKE);
        backCam = CameraServer.startAutomaticCapture(k.CONVEYOR);

        frontCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        backCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        camSelect = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
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
            return k.INTAKE_NAME;
        }
        else
        {
            camSelect.setString(backCam.getName());
            _Display = Display.CONVEYOR;
            return k.CONVEYOR_NAME;
        }
    }
}