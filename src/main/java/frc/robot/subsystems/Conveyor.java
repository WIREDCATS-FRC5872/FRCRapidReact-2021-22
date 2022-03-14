package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake.RunState;

public class Conveyor
{
    private static class k
    {
        private static final int LEFT_MOTOR_ID = 51;
        private static final int RIGHT_MOTOR_ID = 52;

        private static final float speed = -1.0f;
    }

    public static enum RunState
    {
        UP,
        DOWN,
        STOP;
    }

    // ===== MEMBERS ===== //

    public Conveyor.RunState _RunState;
    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(k.LEFT_MOTOR_ID);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(k.RIGHT_MOTOR_ID);
    private final WPI_TalonSRX[] motors = {leftMotor, rightMotor};

    // ===== METHODS ===== //

    public Conveyor()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
        }
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        // Initial position
        _RunState = RunState.STOP;
    }

    public void printData()
    {
        SmartDashboard.putString("Conveyor RunState", _RunState.name());
        SmartDashboard.putNumber("L-Motor Power", leftMotor.get());
        SmartDashboard.putNumber("R-Motor Power", rightMotor.get());
    }

    public void up()
    {
        leftMotor.set(ControlMode.PercentOutput, k.speed);
        rightMotor.set(ControlMode.PercentOutput, k.speed);
        _RunState = RunState.UP;
    }

    /*
    public void down()
    {
        leftMotor.set(ControlMode.PercentOutput, -k.speed);
        rightMotor.set(ControlMode.PercentOutput, -k.speed);
        _RunState = RunState.DOWN;
    }
    */

    public void stop()
    {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        //leftMotor.set(ControlMode.PercentOutput, 0.0f);
        //rightMotor.set(ControlMode.PercentOutput, 0.0f);
        _RunState = RunState.STOP;
    }
}
