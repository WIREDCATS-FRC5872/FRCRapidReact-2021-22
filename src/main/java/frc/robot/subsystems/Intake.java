package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public abstract class Intake
{
    private static class k
    {
        private static final int MOTOR_ID = 99; // temp
    }

    // ===== MEMBERS ===== //

    private static final WPI_TalonSRX motor = new WPI_TalonSRX(k.MOTOR_ID);
    private static final DoubleSolenoid latch = new DoubleSolen
    

    // ===== METHODS ===== //

    public static void init()
    {
        motor.configFactoryDefault();
        motor.setInverted(true);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public static void raise()
    {

    }

    public static void lower()
    {

    }
}