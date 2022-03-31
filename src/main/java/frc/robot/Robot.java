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
import frc.robot.subsystems.Hanger;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.ejml.ops.ConvertMatrixData;

public class Robot extends TimedRobot 
{
    // AUTO "CHOOSER" QUICK & DIRTY METHOD
    private enum Auto
    {
        TEST,

        SIMPLERIGHT,
        RIGHTMOST,
        RIGHTMOST_CENTER_BALLS,
        //RIGHTMOST_CENTER_BALLS_ALT,

        SIMPLELEFT,
        LEFTMOST;
    }
    // CHANGE THIS TO SELECT AUTO
    private final Auto auto = Auto.SIMPLELEFT;

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
    Drivetrain drivetrain;
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
        conveyor.init();
        hanger.init();
        intake.init();

        auto_timer.reset();
        auto_timer.start();
        drivetrain.autoInit();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        //drivetrain.resetEncoders();

        SmartDashboard.putString("AUTO RUNNING", auto.toString());

        double TARMAC_L = 7.0*12;

        // === RIGHT SIMPLE === //
        if (auto == Auto.SIMPLERIGHT)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.close();

            // Let the neighbor robot move out of the way
            Timer.delay(3);

            // Move & score pre-loaded
            drivetrain.forward(2.0);

            /*
            drivetrain.rotateRight(90);
            drivetrain.backward(3);
            conveyor.open();
            conveyor.up();
            Timer.delay(2);
            conveyor.stop();
            conveyor.close();

            // Exit tarmac
            drivetrain.forward(TARMAC_L*2.0/1.4);

            // Fin.
            */
        }
        
        // === RIGHTMOST SIDE - HOLDS 2 BALLS AT A TIME === //
        else if (auto == Auto.RIGHTMOST)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.close();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            Timer.delay(3);

            /*
            // Move & score pre-loaded
            drivetrain.forward(2);
            drivetrain.rotateRight(90);
            drivetrain.backward(3);
            conveyor.open();
            conveyor.up();
            Timer.delay(2);
            conveyor.stop();
            conveyor.close();
            
            // Move & pick up next ball
            drivetrain.forward(TARMAC_L*2.0/1.4);
            drivetrain.rotateLeft(90);
            intake.on();
            conveyor.up();
            drivetrain.forward(TARMAC_L/2.0);
            intake.off();
            conveyor.stop();

            // Get 3rd & final ball, the one on the other alliance side
            rotateLeft(30);
            drivetrain.forward(TARMAC_L*0.8);
            intake.on();
            conveyor.up();
            drivetrain.forward(TARMAC_L*0.4);
            intake.off();
            conveyor.stop();

            // Return & score both
            drivetrain.backward(TARMAC_L*1.2);
            drivetrain.rotateRight(30);
            drivetrain.backward(TARMAC_L/2.0);
            drivetrain.rotaateLeft(90);
            drivetrain.backward(TARMAC_L*2.0/1.4);
            conveyor.open();
            conveyor.up();
            Timer.delay(4);
            conveyor.stop();
            conveyor.close();

            // Leave again
            drivetrain.forward(TARMAC_L*2.0/1.4);
            */
        }
        
        /*
        // === BLUE/RED RIGHTMOST SIDE - less preferred auto === //
        else if (auto == Auto.RIGHTMOST_CENTER_BALLS_ALT)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.close();

            // Let the neighbor robot move out of the way
            Timer.delay(3);

            // Move & score pre-loaded
            drivetrain.forward(2);
            /*
            drivetrain.rotateRight(90);
            drivetrain.backward(3);
            conveyor.open();
            conveyor.up();
            Timer.delay(2);
            conveyor.stop();
            conveyor.close();
            
            // Move & pick up next ball
            drivetrain.forward(TARMAC_L*2.0/1.4);
            drivetrain.rotateLeft(90);
            intake.on();
            conveyor.up();
            drivetrain.forward(TARMAC_L*0.7/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            drivetrain.backward(TARMAC_L*0.7/1.4);
            drivetrain.rotateRight(90);
            drivetrain.backward(TARMAC_L*2.0/1.4);
            conveyor.open();
            conveyor.up();
            Timer.delay(2);
            conveyor.stop();
            conveyor.close();

            // Get 3rd & final ball
            drivetrain.forward(TARMAC_L*1.6/1.4);
            drivetrain.rotateLeft(90);
            drivetrain.forward(TARMAC_L*1.0/1.4);
            intake.on();
            conveyor.up();
            drivetrain.forward(TARMAC_L*0.7/1.4);
            intake.off();
            conveyor.stop();

            // Return & score it
            drivetrain.backward(TARMAC_L*1.7/1.4);
            drivetrain.rotateRight(90);
            drivetrain.backward(TARMAC_L*1.6/1.4);
            conveyor.open();
            conveyor.up();
            Timer.delay(2);
            conveyor.stop();
            conveyor.close();

            // Leave again
            drivetrain.forward(TARMAC_L*2.0/1.4);
        }
        */
        
        // === RIGHTMOST SIDE - HOLDS 2 BALLS AT A TIME === //
        else if (auto == Auto.RIGHTMOST_CENTER_BALLS)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            intake.lower();
            conveyor.close();

            // Let the neighbor robot move out of the way
            // "GET OUT DA WAY!!!"
            Timer.delay(3);

            /*
            // Move & score pre-loaded
            drivetrain.forward(2);
            drivetrain.rotateRight(90);
            drivetrain.backward(3);
            conveyor.open();
            conveyor.up();
            Timer.delay(2);
            conveyor.stop();
            conveyor.close();
            
            // Move & pick up next ball
            drivetrain.forward(TARMAC_L*2.0/1.4);
            drivetrain.rotateLeft(90);
            intake.on();
            conveyor.up();
            drivetrain.forward(TARMAC_L/2.0);
            intake.off();
            conveyor.stop();

            // Get 3rd & final ball
            rotateRight(180);
            drivetrain.forward(TARMAC_L);
            intake.on();
            conveyor.up();
            drivetrain.forward(TARMAC_L*0.4);
            intake.off();
            conveyor.stop();

            // Return & score both
            drivetrain.backward(TARMAC_L*0.9);
            drivetrain.rotateRight(90);
            drivetrain.backward(TARMAC_L*2.0/1.4);
            conveyor.open();
            conveyor.up();
            Timer.delay(4);
            conveyor.stop();
            conveyor.close();

            // Leave again
            drivetrain.forward(TARMAC_L*2.0/1.4);
            */
        }
        
        // === LEFT SIMPLE === //
        else if (auto == Auto.SIMPLELEFT)
        {
            scorePreloaded(auto);
        }
        
        // === LEFTMOST SIDE === //
        else if (auto == Auto.LEFTMOST)
            leftmostAuto();

        else if (auto == Auto.TEST)
        {
            drivetrain.forward(24);
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
        drivetrain.init();
        drivetrain.setCoast();

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
            // Stop motion
            //drivetrain.arcadeDrive(0, 0);

            // Set to break
            if (drivetrain.nm != NeutralMode.Brake)
                drivetrain.setBrake();
        }
        else
        {
            // Drive
            if (controller1.getRawButton(k.RB))  // Slow mode
                drivetrain.arcadeDrive(controller1.getRawAxis(k.LY_ID)/2, controller1.getRawAxis(k.RX_ID)/2);
            else
                drivetrain.arcadeDrive(controller1.getRawAxis(k.LY_ID), controller1.getRawAxis(k.RX_ID));

            // Swich to coast if necessary
            if (drivetrain.nm != NeutralMode.Coast)
                drivetrain.setCoast();
        }

        // Telemetry
        //drivetrain.printData();
        
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
                conveyor.stop();
                conveyor.open();
                raiseIntakeTime = currTime + Intake.DELAY;
            }
        }

        // Act on the queues
        // Run intake
        if (currTime >= runIntakeTime && intake._Position == Intake.Position.DOWN)
        {
            intake.on();
            conveyor.run(false, false);
            conveyor.close();
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
        if (controller1.getRawButtonPressed(k.LB))
        {
            if (conveyor._BeltState != Conveyor.BeltState.UP)
            {
                conveyor.run(false, false);
                conveyor.open();
            }
            else
            {
                conveyor.stop();
                /*
                if (intake._RunState == Intake.RunState.ON)
                {
                    intake.stop();
                    raiseIntakeTime = currTime + Intake.DELAY;
                }
                */
            }
        }

        if (conveyor._BeltState == Conveyor.BeltState.UP)
            conveyor.run(false, false);

        // Listen to Blocker Sensors
        //if (conveyor.isClosed() || conveyor.isOpen())
            //conveyor.stopBlocker();
        
        if (conveyor._BlockerState == Conveyor.BlockerState.OPENING && conveyor.isOpen())
            conveyor.stopBlocker();

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
        
        /*
        // === Cameras === //
        if (controller1.getRawButtonPressed(controls.shiftCam))
            Vision.toggle();
        */
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


    // === AUTO METHODS === //
    public void scorePreloaded(Auto auto)
    {
        // Left side
        if (auto == Auto.LEFTMOST || auto == Auto.SIMPLELEFT)
        {
            // Init position is ball pre-loaded, facing perpendicular to the hub
            //intake.lower();
            
            conveyor.close();
            conveyor.run(false, true);
            
            Timer.delay(2);

            drivetrain.forward(30);
            intake.on();
            drivetrain.intakeBall(conveyor.beltMotor, 30);

            Timer.delay(2);
            conveyor.stop();

            drivetrain.rotate(-15);
            drivetrain.forward(-90);

            conveyor.open();
            conveyor.run(true, false);

            // Exit tarmac - 2 pts
            drivetrain.forward(90);

            // Fin
        }
        // Right side
        else if (auto == Auto.RIGHTMOST || auto == Auto.RIGHTMOST_CENTER_BALLS || auto == Auto.SIMPLERIGHT)
        {
            // TODO
        }
    }

    public void leftmostAuto()
    {
        // Score preloaded ball from starting position
        scorePreloaded(Auto.LEFTMOST);
        
        // Second ball (first non-preloaded) : 

        // Move
        //drivetrain.forward(7);
        // Intake ball (conveyor is closed)
        //intake.on();
        //conveyor.run(true, 0.5);
        //drivetrain.forward(3);
        // Stop intake
        //intake.stop();
        //conveyor.stop();

        /*
        // Third ball (second non-preloaded): the one on the opposite alliance side
        drivetrain.forward(TARMAC_L*2.0/1.4);
        drivetrain.rotateRight(45);
        drivetrain.forward(TARMAC_L);
        drivetrain.rotateRight(90);
        intake.on();
        conveyor.up();
        drivetrain.forward(TARMAC_L*2.0/1.4);
        intake.off();
        conveyor.stop();

        // Return & score it
        drivetrain.backward(TARMAC_L*2.0/1.4);
        drivetrain.rotateLeft(90);
        drivetrain.backward(TARMAC_L);
        drivetrain.rotateLeft(45);
        drivetrain.backward(TARMAC_L*2.0/1.4);
        conveyor.open();
        conveyor.up();
        Timer.delay(2);
        conveyor.stop();
        conveyor.close();

        // Leave again
        drivetrain.forward(TARMAC_L*2.0/1.4);
        */
    }

    public void intakeBall()
    {

    }
}