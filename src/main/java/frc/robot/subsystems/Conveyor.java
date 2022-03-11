package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Conveyor
{
    private static class k
    {
        private static final int LEFT_MOTOR_ID = 51;
        private static final int RIGHT_MOTOR_ID = 52;

        private static final float speed = 1;
    }

    public static enum RunState
    {
        FORWARD,
        REVERSE,
        STOP;
    }

    // ===== MEMBERS ===== //

    public static Conveyor.RunState _RunState;
    private static final WPI_TalonSRX leftMotor = new WPI_TalonSRX(k.LEFT_MOTOR_ID);
    private static final WPI_TalonSRX rightMotor = new WPI_TalonSRX(k.RIGHT_MOTOR_ID);
    private static final WPI_TalonSRX[] motors = {leftMotor, rightMotor};

    // ===== METHODS ===== //

    public static void init()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Coast);
        }
        rightMotor.setInverted(true);

        stop();
    }

    public static void printData()
    {
        SmartDashboard.putString("Conveyor RunState", _RunState.name());
        SmartDashboard.putNumber("L-Motor Power", leftMotor.get());
        SmartDashboard.putNumber("R-Motor Power", rightMotor.get());
    }

    public static void forward()
    {
        leftMotor.set(ControlMode.PercentOutput, k.speed);
        rightMotor.set(ControlMode.PercentOutput, k.speed);
        _RunState = RunState.FORWARD;
    }

    public static void reverse()
    {
        leftMotor.set(ControlMode.PercentOutput, -k.speed);
        rightMotor.set(ControlMode.PercentOutput, -k.speed);
        _RunState = RunState.REVERSE;
    }

    public static void stop()
    {
        leftMotor.set(ControlMode.PercentOutput, 0);
        rightMotor.set(ControlMode.PercentOutput, 0);
        _RunState = RunState.STOP;
    }
}
