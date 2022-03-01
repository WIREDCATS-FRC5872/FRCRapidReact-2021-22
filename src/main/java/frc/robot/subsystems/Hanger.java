package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Hanger
{
    private static class k
    {
        private static final int MOTOR_ID = 89; // temp

        private static final DoubleSolenoid.Value forward = DoubleSolenoid.Value.kForward;
        private static final DoubleSolenoid.Value rest = DoubleSolenoid.Value.kReverse;
        private static final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;
        
        private static final float speed = 1f;
        private static final float TICKS_PER_REV = 1024f;
        private static final float GEAR_RATIO = 100f;
        private static final float INCHES_PER_REV = 0f; // TEMP
        private static final float TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (INCHES_PER_REV);
    }

    public static enum Angle
    {
        FORWARD,
        REST;
    }

    // ===== MEMBERS ===== //

    public static Hanger.Angle _Angle;
    private static final WPI_TalonSRX motor = new WPI_TalonSRX(k.MOTOR_ID);
    private static final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    

    // ===== METHODS ===== //

    public static void init()
    {
        motor.configFactoryDefault();
        motor.setInverted(false); // temp
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public static void printData()
    {
        SmartDashboard.putString("Hanger Angle", _Angle.name());
        SmartDashboard.putString("Solenoid Value", solenoid.get().toString());
        SmartDashboard.putNumber("Motor Power", motor.get());
    }

    public static void raise()
    {
        motor.set(ControlMode.PercentOutput, k.speed);
    }

    public static void lower()
    {
        motor.set(ControlMode.PercentOutput, -1 * k.speed);
    }

    public static void stop()
    {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public static void forward()
    {
        solenoid.set(k.forward);
        _Angle = Angle.FORWARD;
    }

    public static void rest()
    {
        solenoid.set(k.rest);
        _Angle = Angle.REST;
    }
}