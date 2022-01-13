// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.test_subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot 
{
    private static class k
    {
        private static final int LX_ID = 0, LY_ID = 1;
        private static final int Controller_ID = 0;
    }

    private final Joystick controller = new Joystick(k.Controller_ID);
    private final Timer auto_timer = new Timer();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        Drivetrain.init();
        Pneumatics.init();
        Falcon.init();
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
        // Drive for 2 seconds
        if (auto_timer.get() < 2.0)
        {
            Drivetrain.curvatureDrive(1, 0, true);
        }
        else
        {
            Drivetrain.curvatureDrive(0, 0, true);
        }
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

        SmartDashboard.putNumber("Position", Falcon.motor.getSelectedSensorPosition());
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        if (controller.getRawButtonPressed(1))
            Falcon.motor.set(ControlMode.Position, 4096);
        if (controller.getRawButtonPressed(2))
            Falcon.motor.set(ControlMode.Position, -4096);

        // SmartDashboard manual outputs will NOT work in test mode
        System.out.println("Position: " + Falcon.motor.getSelectedSensorPosition());
    }
}
