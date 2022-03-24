// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// By Rachel Leela Dzwonkowski & Allen Dominic B. Sarmiento, Spring 2022

package frc.robot;


import java.util.List;

import org.ejml.dense.row.CovarianceOps_DDRM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainEx;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hanger;

public class Robot extends TimedRobot 
{
    // AUTO "CHOOSER" QUICK & DIRTY METHOD
    private enum Auto
    {
        SIMPLERIGHT,
        RIGHTMOST,
        RIGHTMOST_2ATATIME,

        SIMPLELEFT,
        LEFTMOST;
    }
    // CHANGE THIS TO SELECT AUTO
    private final Auto auto = Auto.RIGHTMOST;

    private static class k
    {
        private static final int LX_ID = 0, LY_ID = 1, RX_ID = 4, RY_ID = 5;
        private static final int LT = 2, RT = 3;
        private static final int A = 1, B = 2, X = 3, Y = 4, LB = 5, RB = 6,
            BACK = 7, START = 8, L_STICK = 9, R_STICK = 10;
        private static final int UP = 0, RIGHT = 90, DOWN = 180, LEFT = 270;
        private static final int CONTROLLER1_ID = 0, CONTROLLER2_ID = 1;
        private static final int PCM_ID = 0; // default node ID
    }

    private final Joystick controller1 = new Joystick(k.CONTROLLER1_ID);
    private final Joystick controller2 = new Joystick(k.CONTROLLER1_ID);    // Should be 2
    private final Compressor pcmCompressor = new Compressor(k.PCM_ID, PneumaticsModuleType.CTREPCM);
    private final Timer auto_timer = new Timer();

    // For the intake
    long runIntakeTime, raiseIntakeTime;
    private final long UNQUEUED = Long.MAX_VALUE;    // Sentinel for prev line's vars

    // === Subsystems === //
    DrivetrainEx dt;
    Conveyor conveyor;
    Intake intake;
    Hanger hanger;
    //Vision vision;
    //private static UsbCamera cam;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        SmartDashboard.updateValues();
        pcmCompressor.enableDigital();
        dt = new DrivetrainEx();
        intake = new Intake();
        conveyor = new Conveyor();
        hanger = new Hanger();
        //cam = CameraServer.startAutomaticCapture();
        //cam.setResolution(100, 100);
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit()
    {
        conveyor.init();
        hanger.init();
        intake.init();

        auto_timer.reset();
        auto_timer.start();
        dt.autoInit();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        SmartDashboard.putString("AUTO RUNNING", auto.toString());

        // === RIGHT SIMPLE === //
        if (auto == Auto.SIMPLERIGHT)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.lock();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            wait(3000);

            // Move & score pre-loaded
            dt.forward(2);
            dt.rotateRight(90);
            dt.backward(3);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Exit tarmac
            dt.forward(7*2.0/1.4);

            // Fin.
        }
        // === BLUE/RED RIGHTMOST SIDE === //
        else if (auto == Auto.RIGHTMOST)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.lock();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            wait(3000);

            // Move & score pre-loaded
            dt.forward(2);
            dt.rotateRight(90);
            dt.backward(3);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();
            
            // Move & pick up next ball
            dt.forward(7*2.0/1.4);
            dt.rotateLeft(90);
            intake.on();
            conveyor.up();
            dt.forward(7*0.7/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            dt.backward(7*0.7/1.4);
            dt.rotateRight(90);
            dt.backward(7*2.0/1.4);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Get 3rd & final ball
            dt.forward(7*1.6/1.4);
            dt.rotateLeft(90);
            dt.forward(7*1.0/1.4);
            intake.on();
            conveyor.up();
            dt.forward(7*0.7/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            dt.backward(7*1.7/1.4);
            dt.rotateRight(90);
            dt.backward(7*1.6/1.4);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Leave again
            dt.forward(7*2.0/1.4);
        }
        // === TODO === //
        else if (auto == Auto.RIGHTMOST_2ATATIME)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.lock();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            wait(3000);

            // Move & score pre-loaded
            dt.forward(2);
            dt.rotateRight(90);
            dt.backward(3);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();
            
            // Move & pick up next ball
            dt.forward(7*2.0/1.4);
            dt.rotateLeft(90);
            intake.on();
            conveyor.up();
            dt.forward(7*0.7/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            dt.backward(7*0.7/1.4);
            dt.rotateRight(90);
            dt.backward(7*2.0/1.4);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Get 3rd & final ball
            dt.forward(7*1.6/1.4);
            dt.rotateLeft(90);
            dt.forward(7*1.0/1.4);
            intake.on();
            conveyor.up();
            dt.forward(7*0.7/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            dt.backward(7*1.7/1.4);
            dt.rotateRight(90);
            dt.backward(7*1.6/1.4);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Leave again
            dt.forward(7*2.0/1.4);
        }
        // === LEFT SIMPLE === //
        else if (auto == Auto.SIMPLELEFT)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.lock();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            wait(3000);

            // Move & score pre-loaded
            dt.forward(2);
            dt.rotateRight(90);
            dt.backward(3);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();
            
            // Exit tarmac
            dt.forward(7*2.0/1.4);
        }
        // === BLUE/RED LEFTMOST SIDE === //
        else if (auto == Auto.LEFTMOST)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.lock();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            wait(3000);

            // Move & score pre-loaded
            dt.forward(2);
            dt.rotateRight(90);
            dt.backward(3);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();
            
            // Move & pick up next ball
            dt.forward(7);
            intake.on();
            conveyor.up();
            dt.forward(7*0.6/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            dt.backward(7*2.0/1.4));
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Get 3rd & final ball
            dt.forward(7*2.0/1.4);
            dt.rotateRight(45);
            dt.forward(7);
            dt.rotateRight(90);
            intake.on();
            conveyor.up();
            dt.forward(7*2.0/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            dt.backward(7*2.0/1.4);
            dt.rotateLeft(90);
            dt.backward(7);
            dt.rotateLeft(45);
            dt.backward(7*2.0/1.4);
            conveyor.release();
            conveyor.up();
            wait(2000);
            conveyor.stop();
            conveyor.lock();

            // Leave again
            dt.forward(7*2.0/1.4);
        }

        while (true)
            SmartDashboard.putString("AUTO RUNNING", "COMPLETED");
    }

    /** This function is called once each time the robot enters teleoperated mode. */
    @Override
    public void teleopInit() {
        
        conveyor.init();
        hanger.init();
        intake.init();

        runIntakeTime = UNQUEUED;
        raiseIntakeTime = UNQUEUED;
    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        // ==== Drive control ==== //
        if (controller1.getRawButton(k.RB))  // Slow mode
            dt.arcadeDrive(controller1.getRawAxis(k.LY_ID)/2, controller1.getRawAxis(k.RX_ID)/2);
        else
            dt.arcadeDrive(controller1.getRawAxis(k.LY_ID), controller1.getRawAxis(k.RX_ID));

        // DT TELEMENTRY
        dt.printData();


        // Shift gear
        /*
        if (controller1.getRawAxis(k.RT) > 0.2)
        {
            if (dt._Gear != DrivetrainEx.Gear.HIGH)
                dt.setHighGear();
            else // Low gear
                dt.setLowGear();
        }
        */
        
        // Shift gear
        if (controller1.getRawAxis(k.RT) > 0.2)
        {
            if (dt._Gear != DrivetrainEx.Gear.HIGH)
                dt.setLowGear();
        }
        else if (dt._Gear != DrivetrainEx.Gear.HIGH)
            dt.setHighGear();
        
        
        // ==== Intake ==== //
        long currTime = System.currentTimeMillis();

        // Raise/Lower
        // These actions queue further actions to ensure the intake does not destroy the wires in our beloved robot
        if (controller1.getRawAxis(k.LT) > 0.2)
        {
            if (intake._Position == Intake.Position.UP)
            {
                intake.lower();
                runIntakeTime = currTime + Intake.DELAY;
            }
            else
            {
                intake.stop();
                raiseIntakeTime = currTime + Intake.DELAY;
            }
        }

        // Act on the queues
        // Run intake as queued
        if (currTime >= runIntakeTime && intake._Position == Intake.Position.DOWN)
        {
            intake.on();
            runIntakeTime = UNQUEUED; // Return to sentinel
        }
        // Raise intake as queued
        if (currTime >= raiseIntakeTime && intake._RunState == Intake.RunState.STOP)
        {
            intake.raise();
            raiseIntakeTime = UNQUEUED;   // Return to sentinel
        }  

        // ==== Conveyor ==== //
        if (controller1.getRawButtonPressed(k.LB))
        {
            if (conveyor._RunState != Conveyor.RunState.UP)
                conveyor.up();
            else
                conveyor.stop();
        }

        // === Hanger === //
        // Vertical
        if (controller2.getRawButton(k.X))
            hanger.raise();
        else if (controller2.getRawButton(k.Y))
            hanger.lower();
        else
            hanger.stop();

        if (controller2.getRawButtonPressed(k.START))
            hanger.resetEncoders();

        // Angle
        if (controller2.getRawButtonPressed(k.A))
        {
            if (hanger._Angle != Hanger.Angle.FORWARD)
                hanger.forward();
            else if (hanger._Angle != Hanger.Angle.REST)
                hanger.rest();
        }
        hanger.printData();
        
        /*
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
    public void testPeriodic() { }

    /*
    public void delay(int ms){

        try{
            Thread.sleep(ms);
        }
        catch(Exception e1){
            e1.printStackTrace();
        }
    }
    */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DrivetrainEx.kEx.sVolts,
                    DrivetrainEx.kEx.vVoltSecondsPerMeter,
                    DrivetrainEx.kEx.aVoltSecondsSquaredPerMeter),
                    DrivetrainEx.kEx.DriveKinematics,
                10);
    
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    DrivetrainEx.kEx.MaxSpeedMetersPerSecond,
                    DrivetrainEx.kEx.MaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DrivetrainEx.kEx.DriveKinematics)
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
                dt::getPose,
                new RamseteController(DrivetrainEx.kEx.RamseteB, DrivetrainEx.kEx.RamseteZeta),
                new SimpleMotorFeedforward(
                    DrivetrainEx.kEx.sVolts,
                    DrivetrainEx.kEx.vVoltSecondsPerMeter,
                    DrivetrainEx.kEx.aVoltSecondsSquaredPerMeter),
                    DrivetrainEx.kEx.DriveKinematics,
                    dt::getWheelSpeeds,
                new PIDController(DrivetrainEx.kEx.PDriveVel, 0, 0),
                new PIDController(DrivetrainEx.kEx.PDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                dt::tankDriveVolts,
                dt);
    
        // Reset odometry to the starting pose of the trajectory.
        dt.resetOdometry(exampleTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> dt.tankDriveVolts(0, 0));
    }
}