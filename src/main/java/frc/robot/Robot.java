// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// By Rachel Leela Dzwonkowski & Allen Dominic B. Sarmiento, Spring 2022

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Conveyor;
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
        private static final int conveyorFwd = k.A;
        private static final int conveyorRev = k.B;
        private static final int conveyorUp = k.UP;
        private static final int conveyorDown = k.DOWN;
        
        // Hanger
        private static final int hangerUp = k.UP;
        private static final int hangerDown = k.DOWN;
        private static final int hangerFwd = k.LEFT;
        private static final int hangerRest = k.RIGHT;

        // Vision
        private static final int shiftCam = k.LB;
    }

    private final Joystick controller1 = new Joystick(k.CONTROLLER1_ID);
    private final Joystick controller2 = new Joystick(k.CONTROLLER2_ID);
    // private final PigeonIMU pigeon = new PigeonIMU(k.PIGEON_ID);
    private static final Compressor pcmCompressor = new Compressor(k.PCM_ID, PneumaticsModuleType.CTREPCM);
    private final Timer auto_timer = new Timer();
    // private double rawHeading = 0, absHeading = 0;

    // Vision
    private static UsbCamera cam;
    //private static UsbCamera cam = new UsbCamera("Test camera", 0);
    //private static NetworkTableEntry camSelect;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // SmartDashboard.updateValues();
        // pcmCompressor.enableDigital();
        // Drivetrain.init();
        DrivetrainEx.init();
        // Intake.init();   
        // Vision.init();
        cam = CameraServer.startAutomaticCapture();
        cam.setResolution(100, 100);
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit()
    {
        auto_timer.reset();
        auto_timer.start();
        DrivetrainEx.zeroHeading();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {
        DrivetrainEx.updateOdometry();
        DrivetrainEx.printData();
    }

    /** This function is called once each time the robot enters teleoperated mode. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        // ==== Drive control ==== //
        if (controller1.getRawButton(controls.slowMode))  // Slow mode
            DrivetrainEx.arcadeDrive(controller1.getRawAxis(k.LY_ID)/2, controller1.getRawAxis(k.RX_ID)/2);
        else
            DrivetrainEx.arcadeDrive(controller1.getRawAxis(k.LY_ID), controller1.getRawAxis(k.RX_ID));

        // DT TELEMENTRY
        DrivetrainEx.printData();
        /*DrivetrainEx.printEncoders();
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

        // Raise/Lower
        if (controller1.getRawButtonPressed(controls.intakeRaise) && Intake._Position == Intake.Position.UP)
            Intake.lower();
        else if (controller1.getRawButtonPressed(controls.intakeRaise))
            Intake.raise();

        // Spin
        if (controller1.getRawButtonPressed(controls.intakeFwd) && Intake._RunState != Intake.RunState.FORWARD)
            Intake.forward();
        else if (controller1.getRawButtonPressed(controls.intakeRev) && Intake._RunState != Intake.RunState.REVERSE)
            Intake.reverse();
        else if (controller1.getRawButtonPressed(controls.intakeFwd) || controller1.getRawButtonPressed(controls.intakeRev))
            Intake.stop();

        
        /*
        // ==== Conveyor ==== //

        if (controller2.getRawButtonPressed(controls.conveyorFwd) && Conveyor._RunState != Conveyor.RunState.FORWARD)
            Conveyor.forward();
        else if (controller2.getRawButtonPressed(controls.conveyorRev) && Conveyor._RunState != Conveyor.RunState.REVERSE)
            Conveyor.reverse();
        else if (controller2.getRawButtonPressed(controls.conveyorFwd) || controller2.getRawButtonPressed(controls.conveyorRev))
            Conveyor.stop();

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
}