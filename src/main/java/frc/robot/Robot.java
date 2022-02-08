// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot 
{
    private static class k
    {
        private static final int LX_ID = 0, LY_ID = 1;
        private static final int A = 1, B = 2, X = 3, Y = 4, LB = 5, RB = 6,
            BACK = 7, START = 8, L_STICK = 9, R_STICK = 10;
        private static final int CONTROLLER_ID = 0;
        private static final int PIGEON_ID = 999; // temp
    }

    private final Joystick controller = new Joystick(k.CONTROLLER_ID);
    // private final PigeonIMU pigeon = new PigeonIMU(k.PIGEON_ID);
    private final Timer auto_timer = new Timer();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        Drivetrain.init();
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit()
    {
        auto_timer.reset();
        auto_timer.start();
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
        Drivetrain.curvatureDrive(controller.getRawAxis(k.LY_ID), controller.getRawAxis(k.LX_ID), true);
        
        // SmartDashboard.putNumber("Heading", pigeon.getYaw());
        // ==== Drive control ==== //
        /*
        if (controller.getRawButton(k.RB))  // Slow mode
            Drivetrain.curvatureDrive(controller.getRawAxis(k.LY_ID)/3, controller.getRawAxis(k.LX_ID)/3, true);
        else
            Drivetrain.curvatureDrive(controller.getRawAxis(k.LY_ID), controller.getRawAxis(k.LX_ID), true);
        */
        
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
