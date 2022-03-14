// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// By Rachel Leela Dzwonkowski & Allen Dominic B. Sarmiento, Spring 2022

package frc.robot;


//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainEx;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hanger;

public class Robot extends TimedRobot 
{
    private static class k
    {
        private static final int LX_ID = 0, LY_ID = 1, RX_ID = 4, RY_ID = 5;
        private static final int A = 1, B = 2, X = 3, Y = 4, LB = 5, RB = 6,
            BACK = 7, START = 8, L_STICK = 9, R_STICK = 10;
        private static final int UP = 0, RIGHT = 90, DOWN = 180, LEFT = 270;
        private static final int CONTROLLER1_ID = 0, CONTROLLER2_ID = 1;
        private static final int PCM_ID = 0; // default node ID
    }

    /*
    private static class controls
    {
        // Drive
        private static final int slowMode = k.RB;
        private static final int shiftGear = k.L_STICK;

        // Intake
        private static final int intakeRaise = k.X;
        private static final int intakeFwd = k.A;
        private static final int intakeRev = k.B;
        
        // Conveyor
        private static final int conveyorUp = k.X;
        private static final int conveyorDown = k.DOWN;
        
        // Hanger
        private static final int hangerUp = k.UP;
        private static final int hangerDown = k.DOWN;
        private static final int hangerFwd = k.LEFT;
        private static final int hangerRest = k.RIGHT;

        // Vision
        private static final int shiftCam = k.LB;
    }
    */

    private final Joystick controller1 = new Joystick(k.CONTROLLER1_ID);
    private final Joystick controller2 = new Joystick(k.CONTROLLER1_ID);    // Should be 2
    private final Compressor pcmCompressor = new Compressor(k.PCM_ID, PneumaticsModuleType.CTREPCM);
    private final Timer auto_timer = new Timer();

    long runIntakeTime, raiseIntakeTime;
    private final long UNQUEUED = -1;    // Sentinel for prev line's vars

    // === Subsystems === //
    Drivetrain dt;
    Conveyor conveyor;
    Intake intake;
    //Hanger hanger;
    //Vision vision;
    //private static UsbCamera cam;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // SmartDashboard.updateValues();
        // pcmCompressor.enableDigital();
        dt = new DrivetrainEx();
        intake = new Intake();
        conveyor = new Conveyor();
        //cam = CameraServer.startAutomaticCapture();
        //cam.setResolution(100, 100);

        // TODO: SUBSYSTEMS ARE STARTING IN THEIR LAST-USED STATE! WHY IS THE INTAKE ON!?
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit()
    {
        auto_timer.reset();
        auto_timer.start();
        //dt.zeroHeading();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        //dt.updateOdometry();
        dt.printData();
    }

    /** This function is called once each time the robot enters teleoperated mode. */
    @Override
    public void teleopInit() {
        runIntakeTime = UNQUEUED;
        raiseIntakeTime = UNQUEUED;
    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        // ==== Drive control ==== //
        boolean rb1 = controller1.getRawButton(k.RB);
        double ly1 = controller1.getRawAxis(k.LY_ID);
        Double rx1 = controller1.getRawAxis(k.RX_ID);
        
        if (rb1)  // Slow mode
            dt.arcadeDrive(ly1/2, rx1/2);
        else
            dt.arcadeDrive(ly1, rx1);

        // DT TELEMENTRY
        dt.printData();
        /*
        SmartDashboard.putNumber("Raw Heading", dt.getRawHeading());
        SmartDashboard.putNumber("Abs Heading", dt.getHeading());
        SmartDashboard.putNumber("Turn rate", dt.getTurnRate());
        */

        /*
        // Shift gear
        if (controller1.getRawButtonPressed(controls.shiftGear) && DrivetrainEx._Gear != DrivetrainEx.Gear.HIGH)
            DrivetrainEx.setHighGear();
        else if (controller1.getRawButtonPressed(controls.shiftGear) && DrivetrainEx._Gear != DrivetrainEx.Gear.LOW)
            DrivetrainEx.setLowGear();
        */
        
        // ==== Intake ==== //
        /*
        // Raise/Lower
        // These actions queue further actions to ensure the intake does not destroy the wires in our beloved robot
        if (controller1.getRawButtonPressed(k.LB) && intake._Position == Intake.Position.UP)
        {
            intake.lower();
            runIntakeTime = System.currentTimeMillis() + Intake.DELAY;
        }
        else if (controller1.getRawButtonPressed(k.LB))
        {
            raiseIntakeTime = System.currentTimeMillis() + Intake.DELAY;
        }

        // Act on the queues
        // Run intake as queued
        if (System.currentTimeMillis() >= runIntakeTime)
        {
            intake.on();
            runIntakeTime = UNQUEUED; // Return to sentinel
        }
        // Raise intake as queued
        if (System.currentTimeMillis() >= raiseIntakeTime)
        {
            intake.raise();
            raiseIntakeTime = UNQUEUED;   // Return to sentinel
        }
        */

        // Spin
        boolean a1 = controller1.getRawButtonPressed(k.A);
        if (a1 && intake._RunState != Intake.RunState.ON)
            intake.on();
        else if (a1)
            intake.stop();      

        // ==== Conveyor ==== //
        boolean x2 = controller2.getRawButtonPressed(k.X);
        if (x2 && conveyor._RunState != Conveyor.RunState.UP)
            conveyor.up();
        else if (x2)
            conveyor.stop();

        /*
        // === Hanger === //

        // Vertical
        if (controller2.getRawButton(controls.hangerUp))
            Hanger.raise();
        else if (controller2.getRawButton(controls.hangerDown))
            Hanger.lower();
        else
            Hanger.stop();

        // Angle
        if (controller2.getRawButtonPressed(controls.hangerFwd) && Hanger._Angle != Hanger.Angle.FORWARD)
            Hanger.forward();
        else if (controller2.getRawButtonPressed(controls.hangerRest) && Hanger._Angle != Hanger.Angle.REST)
            Hanger.rest();
            
        // === Cameras === //
        if (controller1.getRawButtonPressed(controls.shiftCam))
            Vision.toggle();
        */
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    public void delay(int ms){

        try{
            Thread.sleep(ms);
        }
        catch(Exception e1){
            e1.printStackTrace();
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*
    public static Command getAutonomousCommand() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    kEx.sVolts,
                    kEx.vVoltSecondsPerMeter,
                    kEx.aVoltSecondsSquaredPerMeter),
                kEx.DriveKinematics,
                10);
    
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    kEx.MaxSpeedMetersPerSecond,
                    kEx.MaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kEx.DriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
    
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
    
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
                drive::getPose,
                new RamseteController(kEx.RamseteB, kEx.RamseteZeta),
                new SimpleMotorFeedforward(
                    kEx.sVolts,
                    kEx.vVoltSecondsPerMeter,
                    kEx.aVoltSecondsSquaredPerMeter),
                kEx.DriveKinematics,
                drive::getWheelSpeeds,
                new PIDController(kEx.PDriveVel, 0, 0),
                new PIDController(kEx.PDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drive::tankDriveVolts,
                m_robotDrive);
    
        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
      }
    }*/
}