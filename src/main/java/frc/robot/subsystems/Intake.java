package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake
{
    private static class k
    {
        private static final int MOTOR_ID = 10;
        private static final int FWD_ID = 1, REV_ID = 6;  // TEMP

        private static final DoubleSolenoid.Value forward = DoubleSolenoid.Value.kForward;
        private static final DoubleSolenoid.Value reverse = DoubleSolenoid.Value.kReverse;
        private static final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;
        
        private static final float speed = -0.5f;
    }

    public enum RunState
    {
        ON,
        REVERSE,
        STOP;
    }

    public enum Position
    {
        UP,
        DOWN;
    }

    // ===== MEMBERS ===== //
    public Intake.RunState _RunState; 
    public Intake.Position _Position;
    private final WPI_TalonSRX motor = new WPI_TalonSRX(k.MOTOR_ID);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, k.FWD_ID, k.REV_ID);
    public static final long DELAY = 500; // ms

    // ===== METHODS ===== //

    public Intake()
    {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setNeutralMode(NeutralMode.Coast);
    }
    
    public void init()
    {
        stop();
        lower();
    }

    public void printData()
    {
        SmartDashboard.putString("Intake RunState", _RunState.name());
        SmartDashboard.putString("Intake Position", _RunState.name());
        SmartDashboard.putString("Solenoid Value", solenoid.get().toString());
        SmartDashboard.putNumber("Motor Power", motor.get());
    }

    public void raise()
    {
        solenoid.set(k.forward);
        _Position = Position.UP;
    }

    public void lower()
    {
        solenoid.set(k.reverse);
        _Position = Position.DOWN;
    }

    public void on()
    {
        if (_Position == Position.DOWN) // TODO: Test that this doesn't cause problems
        {
            motor.set(ControlMode.PercentOutput, k.speed);
            _RunState = RunState.ON;
        }
    }

    /*
    public void reverse()
    {
        motor.set(ControlMode.PercentOutput, -k.speed);
        _RunState = RunState.REVERSE;
    }
    */

    public void stop()
    {
        motor.set(ControlMode.PercentOutput, 0);
        _RunState = RunState.STOP;
    }
}