package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Intake
{
    private static class k
    {
        private static final int MOTOR_ID = 99; // temp

        private static final DoubleSolenoid.Value forward = DoubleSolenoid.Value.kForward;
        private static final DoubleSolenoid.Value reverse = DoubleSolenoid.Value.kReverse;
        private static final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;
        
        private static final float speed = 1;
    }

    public static enum RunState
    {
        FORWARD,
        REVERSE,
        STOP;
    }

    public static enum Position
    {
        UP,
        DOWN;
    }

    // ===== MEMBERS ===== //

    public static Intake.RunState _RunState; 
    public static Intake.Position _Position;
    private static final WPI_TalonSRX motor = new WPI_TalonSRX(k.MOTOR_ID);
    private static final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    

    // ===== METHODS ===== //

    public static void init()
    {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public static void printData()
    {
        SmartDashboard.putString("Intake RunState", _RunState.name());
        SmartDashboard.putString("Intake Position", _RunState.name());
        SmartDashboard.putString("Solenoid Value", solenoid.get().toString());
        SmartDashboard.putNumber("Motor Power", motor.get());
    }

    public static void raise()
    {
        stop();
        solenoid.set(k.forward);
        _Position = Position.UP;
    }

    public static void lower()
    {
        solenoid.set(k.reverse);
        _Position = Position.DOWN;
        forward();
    }

    public static void forward()
    {
        if (_Position == Position.DOWN)
        {
            motor.set(ControlMode.PercentOutput, k.speed);
            _RunState = RunState.FORWARD;
        }
    }

    public static void reverse()
    {
        if (_Position == Position.DOWN)
        {
            motor.set(ControlMode.PercentOutput, -k.speed);
            _RunState = RunState.FORWARD;
        }
    }

    public static void stop()
    {
        motor.set(ControlMode.PercentOutput, 0);
        _RunState = RunState.STOP;
    }
}