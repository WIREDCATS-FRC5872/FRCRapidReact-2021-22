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
    private final Joystick controller2 = new Joystick(k.CONTROLLER1_ID);    // SHould be 2
    // private final PigeonIMU pigeon = new PigeonIMU(k.PIGEON_ID);
    private static final Compressor pcmCompressor = new Compressor(k.PCM_ID, PneumaticsModuleType.CTREPCM);
    private final Timer auto_timer = new Timer();
    // private double rawHeading = 0, absHeading = 0;

    // === Subsystems === //
    Drivetrain dt;
    Conveyor conveyor;
    Intake intake;
    //Hanger hanger;
    // Vision vision;
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
        SmartDashboard.putString("INITIALIZATION", "SUCCESS!");
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
    public void teleopInit() {}

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        pcmCompressor.disable();

        // ==== Drive control ==== //
        if (controller1.getRawButton(k.RB))  // Slow mode
            dt.arcadeDrive(controller1.getRawAxis(k.LY_ID)/2, controller1.getRawAxis(k.RX_ID)/2);
        else
            dt.arcadeDrive(controller1.getRawAxis(k.LY_ID), controller1.getRawAxis(k.RX_ID));

        // DT TELEMENTRY
        dt.printData();
        /*
        DrivetrainEx.printEncoders();
        SmartDashboard.putNumber("Raw Heading", DrivetrainEx.getRawHeading());
        SmartDashboard.putNumber("Abs Heading", DrivetrainEx.getHeading());
        SmartDashboard.putNumber("Turn rate", DrivetrainEx.getTurnRate());
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
        if (controller1.getRawButtonPressed(controls.intakeRaise) && Intake._Position == Intake.Position.UP)
            Intake.lower();
        else if (controller1.getRawButtonPressed(controls.intakeRaise))
            Intake.raise();
        */

        // Spin
<<<<<<< Updated upstream
        if (controller1.getRawButtonPressed(controls.intakeFwd) && Intake._RunState != Intake.RunState.FORWARD)
            Intake.forward();
        else if (controller1.getRawButtonPressed(controls.intakeRev) && Intake._RunState != Intake.RunState.REVERSE)
            Intake.reverse();
        else if (controller1.getRawButtonPressed(controls.intakeFwd) || controller1.getRawButtonPressed(controls.intakeRev))
            Intake.stop();
        */

        // Raise/Lower
        if (controller2.getRawButton(controls.intakeFwd))
        {
            Intake.lower();
            long startTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - startTime > Intake.DELAY)
                Intake.forward();
        }

        else if (controller2.getRawButton(controls.intakeRev))
        {
            Intake.lower();
            long startTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - startTime > Intake.DELAY)
                Intake.reverse();
        }

        else if (Intake._Position == Intake.Position.DOWN && Intake._RunState != Intake.RunState.STOP)
        {
            Intake.stop();
            long startTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - startTime > Intake.DELAY)
                Intake.raise();
        }
        
        /*
=======
        if (controller1.getRawButtonPressed(k.A) && intake._RunState != Intake.RunState.FORWARD)
        {
            intake.forward();
        }
        else if (controller1.getRawButtonPressed(k.B) || controller1.getRawButtonPressed(k.A))
            intake.stop();      

>>>>>>> Stashed changes
        // ==== Conveyor ==== //
        
        if (controller2.getRawButtonPressed(k.X) && conveyor._RunState != Conveyor.RunState.UP)
        {
            conveyor.up();
        }
        else if (controller2.getRawButtonPressed(k.Y) || controller2.getRawButtonPressed(k.X))
        {
            conveyor.stop();
        }
        SmartDashboard.putString("Conveyor State", conveyor._RunState.name());

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