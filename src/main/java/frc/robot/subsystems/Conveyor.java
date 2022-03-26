package frc.robot.subsystems;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conveyor
{
    private static class k
    {
        private static final int BLOCKER_MOTOR_ID = 51;
        private static final int MAIN_MOTOR_ID = 52;
        private static final int OPEN_SENSOR_ID = 9,
                                CLOSED_SENSOR_ID = 8;
        private static final float speed = 1.0f;
        private static final double MAX_BLOCKER_VOLTAGE = 0;
    }

    public static enum RunState
    {
        UP,
        DOWN,
        STOP;
    }

    public static enum OpenState
    {
        CLOSED,
        OPEN;
    }

    // ===== MEMBERS ===== //

    public Conveyor.RunState _RunState;
    public OpenState _OpenState;

    private final WPI_TalonSRX mainMotor = new WPI_TalonSRX(k.MAIN_MOTOR_ID);
    private final WPI_TalonSRX blockerMotor = new WPI_TalonSRX(k.BLOCKER_MOTOR_ID);
    private final WPI_TalonSRX[] motors = {mainMotor, blockerMotor};
    private final DigitalInput openSensor = new DigitalInput(k.OPEN_SENSOR_ID);
    private final DigitalInput closeSensor = new DigitalInput(k.CLOSED_SENSOR_ID);
    private final Timer auto_timer = new Timer();

    // ===== METHODS ===== //

    public Conveyor()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
        }
        mainMotor.setInverted(false);
        blockerMotor.setInverted(false);
    }

    public void init()
    {
        stop();
    }

    public boolean isOpen()
    {
        return openSensor.get();
    }

    public boolean isClosed()
    {
        return closeSensor.get();
    }
    
    public void printData()
    {
        SmartDashboard.putString("Conveyor RunState", _RunState.name());
        SmartDashboard.putNumber("Main-Motor Power", mainMotor.get());
        SmartDashboard.putNumber("Blocker-Motor Power", blockerMotor.get());
        SmartDashboard.putNumber("Blocker Stator Current", blockerMotor.getStatorCurrent());
        SmartDashboard.putNumber("Blocker Supply Current", blockerMotor.getSupplyCurrent());
        SmartDashboard.putBoolean("CONVEYOR CLOSED", closeSensor.get());
        SmartDashboard.putBoolean("CONVEYOR OPEN", openSensor.get());
    }

    public void up()
    {
        mainMotor.set(ControlMode.PercentOutput, k.speed);
        _RunState = RunState.UP;
    }

    public void autoRun(double seconds)
    {
        mainMotor.set(ControlMode.PercentOutput, 0.75);

        auto_timer.reset();
        auto_timer.start();
        while (auto_timer.get() < 5) {}

        stop();
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
        mainMotor.stopMotor();
        //leftMotor.set(ControlMode.PercentOutput, 0.0f);
        //rightMotor.set(ControlMode.PercentOutput, 0.0f);
        _RunState = RunState.STOP;
    }

    public void close()
    {
        blockerMotor.set(ControlMode.PercentOutput, 0.2);
        //System.out.println("SHUT UR MOUF");
        _OpenState = OpenState.CLOSED;
    }

    public void open()
    {
        blockerMotor.set(ControlMode.PercentOutput, -0.2);
        //System.out.println("OPEN UP ITS THE POLICE");
        _OpenState = OpenState.OPEN;
    }

    public void stopBlocker()
    {
        blockerMotor.set(ControlMode.PercentOutput, 0);
    }
}