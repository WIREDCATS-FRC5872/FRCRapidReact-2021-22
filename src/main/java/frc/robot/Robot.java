// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.*;

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

    private static WPI_TalonSRX encodedTalon;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        Drivetrain.init();
        Pneumatics.init();
        encodedTalon = new WPI_TalonSRX(5);
        encodedTalon.configFactoryDefault();
        encodedTalon.setNeutralMode(NeutralMode.Brake);
        encodedTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
        encodedTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
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
        Drivetrain.curvatureDrive(controller.getRawAxis(k.LY_ID), controller.getRawAxis(k.LX_ID), false);
    }

    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        encodedTalon.set(ControlMode.Position, 4096);
    }
}
