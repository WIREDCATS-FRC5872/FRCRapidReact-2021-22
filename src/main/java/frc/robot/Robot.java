// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
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
        private static final int CONTROLLER_ID = 0;
        private static final int PIGEON_ID = 0;
        private static final int PCM_ID = 0; // default node ID
    }

    private final Joystick controller = new Joystick(k.CONTROLLER_ID);
    private final PigeonIMU pigeon = new PigeonIMU(k.PIGEON_ID);
    private static final Compressor pcmCompressor = new Compressor(k.PCM_ID, PneumaticsModuleType.CTREPCM);
    private final Timer auto_timer = new Timer();
    private double rawHeading = 0, absHeading = 0;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        pcmCompressor.enableDigital();
        Drivetrain.init();
        Intake.init();
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit()
    {
        auto_timer.reset();
        auto_timer.start();
        pigeon.setYaw(0);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {

    }

    /** This function is called once each time the robot enters teleoperated mode. */
    @Override
    public void teleopInit() 
    { 

    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic()
    {
        // ==== Drive control ==== //
        if (controller.getRawButton(k.RB))  // Slow mode
            Drivetrain.arcadeDrive(controller.getRawAxis(k.LY_ID)/2, controller.getRawAxis(k.RX_ID)/2);
        else
            Drivetrain.arcadeDrive(controller.getRawAxis(k.LY_ID), controller.getRawAxis(k.RX_ID));

        // ==== Intake ==== //

        // Raise/Lower
        if (controller.getRawButtonPressed(k.X) && Intake._Position == Intake.Position.UP)
            Intake.lower();
        else if (controller.getRawButtonPressed(k.X))
            Intake.raise();

        // Spin
        if (controller.getRawButtonPressed(k.A) && Intake._RunState != Intake.RunState.FORWARD)
            Intake.forward();
        else if (controller.getRawButtonPressed(k.B) && Intake._RunState != Intake.RunState.REVERSE)
            Intake.reverse();
        else if (controller.getRawButtonPressed(k.A) || controller.getRawButtonPressed(k.B))
            Intake.stop();

        // === Hanger === //

        // Vertical
        if (controller.getRawButton(k.UP))
            Hanger.raise();
        else if (controller.getRawButton(k.DOWN))
            Hanger.lower();
        else
            Hanger.stop();

        // Angle
        if (controller.getRawButtonPressed(k.RIGHT) && Hanger._Angle != Hanger.Angle.FORWARD)
            Hanger.forward();
        else if (controller.getRawButtonPressed(k.LEFT) && Hanger._Angle != Hanger.Angle.REST)
            Hanger.rest();

        // ==== Pigeon ==== //

        // Conversion
        rawHeading = -pigeon.getYaw();
        absHeading = rawHeading%360;
        if (absHeading < 0)
            absHeading += 360;

        // ==== Telemetry ==== //
        Drivetrain.printEncoderCount();
        SmartDashboard.putNumber("Raw Heading", rawHeading);
        SmartDashboard.putNumber("Abs Heading", absHeading);
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {

    }
}
