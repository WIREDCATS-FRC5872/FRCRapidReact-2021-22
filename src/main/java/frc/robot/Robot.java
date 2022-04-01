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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyor.BlockerState;
import frc.robot.subsystems.Hanger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Robot extends TimedRobot 
{
    private static enum Auto
    {
        LEFT,
        RIGHT;
    }
    private static enum NumToScore
    {
        ZERO,
        ONE,
        TWO;
    }

    private final Auto auto = Auto.LEFT;
    private final int AUTO_DELAY = 0;    // in seconds
    private final NumToScore NUMBER_TO_SCORE = NumToScore.TWO;

    private static class k
    {
        private static final int LX_ID = 0, LY_ID = 1, RX_ID = 4, RY_ID = 5;
        private static final int LT = 2, RT = 3;
        private static final int A = 1, B = 2, X = 3, Y = 4, LB = 5, RB = 6,
            BACK = 7, START = 8, L_STICK = 9, R_STICK = 10;
        private static final int UP = 0, RIGHT = 90, DOWN = 180, LEFT = 270;
        private static final int CONTROLLER1_ID = 0, CONTROLLER2_ID = 1;
        private static final int PCM_ID = 0; // default node ID

        // Drive
        private static final double 
        SECONDS_PER_INCH = 1/40.0f;
        private static final double SECONDS_PER_DEGREE = (1/219.0f) * 103.0/90.0;
    }

    private final Joystick controller1 = new Joystick(k.CONTROLLER1_ID);
    private final Joystick controller2 = new Joystick(k.CONTROLLER1_ID);    // Should be 2
    private final Compressor pcmCompressor = new Compressor(k.PCM_ID, PneumaticsModuleType.CTREPCM);
    private final Timer timer = new Timer();

    // For the intake
    long runIntakeTime, raiseIntakeTime;
    private final long UNQUEUED = Long.MAX_VALUE;    // Sentinel for prev line's vars

    // === Subsystems === //
    Drivetrain drivetrain;
    Conveyor conveyor;
    Intake intake;
    Hanger hanger;
    //Vision vision;
    //private static UsbCamera cam;

    // Autonomous tracker
    private boolean isFinished;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        SmartDashboard.updateValues();
        pcmCompressor.enableDigital();
        drivetrain = new Drivetrain();
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
        isFinished = false;

        conveyor.init();
        hanger.init();
        intake.init();
        drivetrain.init(true);
        
        timer.reset();
        timer.start();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        // Check for finished
        if (isFinished)
            return;

        // Add initial delay
        if (timer.get() < AUTO_DELAY)
            return;

        // Just park - 82/40 seconds
        if (NUMBER_TO_SCORE == NumToScore.ZERO)
        {
            // Move off the tarmac
            if (timer.get() < AUTO_DELAY +  82 * k.SECONDS_PER_INCH)
            {
                // Drive
                drivetrain.move(1, 0);
            }
            else
            {
                // Auto is finished
                drivetrain.stop();
                isFinished = true;
            }
        }
        // Score preloaded - 138/40 + 5.5 seconds
        else if (NUMBER_TO_SCORE == NumToScore.ONE)
        {
            
            if (timer.get() < AUTO_DELAY + 48 * k.SECONDS_PER_INCH)
            {
                drivetrain.move(-1, 0);
            }
            else if (timer.get() < AUTO_DELAY + 48 * k.SECONDS_PER_INCH + 5)
            {
                drivetrain.stop();
                conveyor.open(false);
                conveyor.run(false);
            }
            else if (timer.get() < AUTO_DELAY + 48 * k.SECONDS_PER_INCH + 5.5)
            {
                conveyor.stopBelt();
            }
            else if (timer.get() < AUTO_DELAY + 138 * k.SECONDS_PER_INCH + 5.5)
            {
                drivetrain.move(1, 0);
            }
            else
            {
                // Auto is finished
                drivetrain.stop();
                isFinished = true;
            }
        }
        // Two-ball auto - 150/40 + 15/219.0f * 103.0/90.0 + 5.5
        else if (NUMBER_TO_SCORE == NumToScore.TWO)
        {
            if (timer.get() < AUTO_DELAY +  30 * k.SECONDS_PER_INCH)
            {
                // Lower intake and close blocker
                intake.lower();
                conveyor.close(false);

                // Move towards ball
                drivetrain.move(1, 0);
            }
            else if (timer.get() < AUTO_DELAY + 60 * k.SECONDS_PER_INCH)
            {
                // Pick up ball
                intake.run();
                conveyor.run(true);

                drivetrain.move(1, 0);
            }
            else if (timer.get() < AUTO_DELAY + 60 * k.SECONDS_PER_INCH + 0.5)
            {
                conveyor.stopBelt();
                intake.stop();
                drivetrain.stop();
            }
            else if (timer.get() < AUTO_DELAY + 60 * k.SECONDS_PER_INCH + 15 * k.SECONDS_PER_DEGREE + 0.5)
            {                
                // Drive to goal
                if (auto == Auto.LEFT)
                    drivetrain.move(0, -1);
                else // RIGHT
                    drivetrain.move(0, 1);
            }
            else if (timer.get() < AUTO_DELAY + 60 * k.SECONDS_PER_INCH + 15 * k.SECONDS_PER_DEGREE + 5)
            {
                // Outtake both balls

                drivetrain.stop();
                conveyor.open(false);
                conveyor.run(false);
            }
            else if (timer.get() < AUTO_DELAY + 60 * k.SECONDS_PER_INCH + 15 * k.SECONDS_PER_DEGREE + 5.5)
            {
                conveyor.stopBelt();
            }
            else if (timer.get() < AUTO_DELAY + 150 * k.SECONDS_PER_INCH + 15 * k.SECONDS_PER_DEGREE + 5.5)
            {
                // Exit tarmac
                drivetrain.move(-1, 0);
            }
            else
            {
                // Auto is finished
                drivetrain.stop();
                isFinished = true;
            }
        }
    }

    /** This function is called once each time the robot enters teleoperated mode. */
    @Override
    public void teleopInit() {
        
        conveyor.init();
        hanger.init();
        intake.init();
        drivetrain.init(false);

        runIntakeTime = UNQUEUED;
        raiseIntakeTime = UNQUEUED;
    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        long currTime = System.currentTimeMillis();
        
        // ==== DRIVETRAIN ==== //
        
        // Gear
        if (controller1.getRawAxis(k.RT) > 0.2)
        {
            if (drivetrain._Gear != Drivetrain.Gear.HIGH)
                drivetrain.setHighGear();
        }
        else if (drivetrain._Gear != Drivetrain.Gear.LOW)
            drivetrain.setLowGear();

        // Brake B Button
        if (controller1.getRawButton(k.B))
        {
            // Set to break
            if (drivetrain.nm != NeutralMode.Brake)
                drivetrain.setBrake();
        }
        else
        {
            // Swich to coast if necessary
            if (drivetrain.nm != NeutralMode.Coast)
                drivetrain.setCoast();
        }

        // Drive
        if (controller1.getRawButton(k.RB))  // Slow mode
            drivetrain.arcadeDrive(-controller1.getRawAxis(k.LY_ID)/2, -controller1.getRawAxis(k.RX_ID)/2);
        else
            drivetrain.arcadeDrive(-controller1.getRawAxis(k.LY_ID), -controller1.getRawAxis(k.RX_ID));
        
        // ==== Intake ==== //
        
        // Raise/Lower
        // These actions queue further actions to ensure the intake does not destroy the wires in our beloved robot
        if (controller1.getRawAxis(k.LT) > 0.2)
        {
            if (intake._Position == Intake.Position.UP)
            {
                intake.lower();
                runIntakeTime = currTime + Intake.DELAY;
            }
            else if (intake._RunState == Intake.RunState.ON)
            {
                intake.stop();
                conveyor.stopBelt();
                raiseIntakeTime = currTime + Intake.DELAY;
            }
        }

        // Act on the queues
        // Run intake
        if (currTime >= runIntakeTime && intake._Position == Intake.Position.DOWN)
        {
            intake.run();
            conveyor.run(true);
            runIntakeTime = UNQUEUED; // Return to sentinel
        }
        // Raise intake
        if (currTime >= raiseIntakeTime && intake._RunState == Intake.RunState.STOP)
        {
            intake.raise();
            raiseIntakeTime = UNQUEUED;   // Return to sentinel
        }

        // ==== CONVEYOR ==== //

        // Outtake
        /*
        if (controller1.getRawButton(k.LB))
        {
            if (conveyor._BeltState != Conveyor.BeltState.ON)
            {
                conveyor.run(false);
                conveyor.open(false);
            }
            else
                conveyor.stopBelt();
        }
        */

        if (controller1.getRawButton(k.LB) && intake._Position == Intake.Position.UP)
        {
            conveyor.run(false);
            if (!conveyor.isClosed())
                conveyor.stopBlocker();
            else
                conveyor.open(false);
        }
        // Then stop belt UNLESS intake is running (in which case let it be)
        else if (intake._Position != Intake.Position.DOWN)
        {
            conveyor.stopBelt();
            if (conveyor.isClosed())
                conveyor.stopBlocker();
            else
                conveyor.close(false);
        }

        // Telemetry
        conveyor.printData();

        // === Hanger === //

        // Vertical
        if (controller2.getRawButton(k.Y))
            hanger.raise();
        else if (controller2.getRawButton(k.X))
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
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {

        teleopInit();
        hanger.testInit();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        teleopPeriodic();
    }
}