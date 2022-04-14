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
        private static final int BELT_MOTOR_ID = 52, BLOCKER_MOTOR_ID = 51;
        private static final int OPEN_SENSOR_ID = 1, CLOSED_SENSOR_ID = 0;

        private static final double BELT_WAIT = 5;
        private static final double BELT_POWER = 1;
        private static final double BELT_LOW_VOLTAGE = 3;

        private static final double BLOCKER_POWER = 1;
    }

    public static enum BeltState
    {
        ON,
        OFF;
    }

    public static enum BlockerState
    {
        CLOSING,
        OPENING,
        STOPPED;
    }

    // ===== MEMBERS ===== //

    public BeltState _BeltState;
    public BlockerState _BlockerState;

    public final WPI_TalonSRX beltMotor = new WPI_TalonSRX(k.BELT_MOTOR_ID);
    public final WPI_TalonSRX blockerMotor = new WPI_TalonSRX(k.BLOCKER_MOTOR_ID);
    private final WPI_TalonSRX[] motors = {beltMotor, blockerMotor};
    private final DigitalInput openSensor = new DigitalInput(k.OPEN_SENSOR_ID);
    private final DigitalInput closeSensor = new DigitalInput(k.CLOSED_SENSOR_ID);
    private final Timer timer = new Timer();

    // ===== METHODS ===== //

    public Conveyor()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
        }
        beltMotor.setInverted(false);
        blockerMotor.setInverted(false);
        blockerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void init()
    {
        stopBelt();
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
        //SmartDashboard.putString("Conveyor BeltState", _BeltState.name());
        //SmartDashboard.putNumber("Belt-Motor Power", beltMotor.get());
        SmartDashboard.putNumber("Blocker-Motor Power", blockerMotor.get());
        SmartDashboard.putNumber("Blocker Stator Current", blockerMotor.getStatorCurrent());
        SmartDashboard.putNumber("Blocker Supply Current", blockerMotor.getSupplyCurrent());
        SmartDashboard.putBoolean("Blocker Closed", isClosed());
        SmartDashboard.putBoolean("Blocker Open", isOpen());
        //SmartDashboard.putString("Blocker State", _BlockerState.name());
    }

    public void run(boolean runAtLowVoltage)
    {
        if (runAtLowVoltage)
            beltMotor.setVoltage(5.5);
        else
            beltMotor.set(ControlMode.PercentOutput, k.BELT_POWER);
        _BeltState = BeltState.ON;
    }

    public void runReverse()
    {
        beltMotor.set(ControlMode.PercentOutput, -k.BELT_POWER);
        _BeltState = BeltState.ON;
    }

    public void stopBelt()
    {
        //beltMotor.stopMotor();
        beltMotor.set(ControlMode.PercentOutput, 0);
        _BeltState = BeltState.OFF;
    }
    
    /**
     * Runs the belt moter for a set time at a set percent ouput.
     */
    public void outtake()
    {
        beltMotor.set(ControlMode.PercentOutput, k.BELT_POWER);
        /*
        timer.start();
        beltMotor.set(ControlMode.PercentOutput, k.BELT_POWER);
        while (timer.get() < k.BELT_WAIT) {}
        timer.stop();
        timer.reset();
        stopBelt();
        */
    }

    /**
     * Closes the blocker motor.
     * @param isAuto If true, blocker motor power is set to 0 (with brake neutral mode) when isClosed() is detected.
     * Else, blocker motor will not be stopped and _BlockerState will be updated to "CLOSING".
     */
    public void close(boolean isAuto)
    {
        if (isAuto)
        {
            while (!isClosed())
                blockerMotor.set(ControlMode.PercentOutput, -k.BLOCKER_POWER);
            blockerMotor.set(ControlMode.PercentOutput, 0);
        }
        else
        {
            blockerMotor.set(ControlMode.PercentOutput, -k.BLOCKER_POWER);
            _BlockerState = BlockerState.CLOSING;
        }
    }

    /**
     * Opens the blocker motor.
     * @param isAuto If true, blocker motor power is set to 0 (with brake neutral mode) when isOpen() is detected.
     * Else, blocker motor will not be stopped and _BlockerState will be updated to "OPENING".
     */
    public void open(boolean isAuto)
    {
        if (isAuto)
        {
            while (!isClosed())
                blockerMotor.set(ControlMode.PercentOutput, k.BLOCKER_POWER);
            blockerMotor.set(ControlMode.PercentOutput, 0);
        }
        else
        {
            blockerMotor.set(ControlMode.PercentOutput, k.BLOCKER_POWER);
            _BlockerState = BlockerState.OPENING;
        }
        
    }

    public void stopBlocker()
    {
        blockerMotor.set(ControlMode.PercentOutput, 0);
        _BlockerState = Conveyor.BlockerState.STOPPED;
    }
}