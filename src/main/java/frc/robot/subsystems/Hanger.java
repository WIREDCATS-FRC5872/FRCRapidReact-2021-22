package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Hanger
{
    private static class k
    {
        private static final int LEFT_MOTOR_ID = 16;
        private static final int RIGHT_MOTOR_ID = 17;

        private static final DoubleSolenoid.Value forward = DoubleSolenoid.Value.kForward;
        private static final DoubleSolenoid.Value rest = DoubleSolenoid.Value.kReverse;
        private static final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;
        
        private static final float upSpeed = 1.0f;
        private static final float downSpeed = 1.0f;

        private static final float TICKS_PER_REV = 1024f;
        private static final float GEAR_RATIO = 100.0f;
        private static final float INCHES_PER_REV = 0f; // TEMP
        private static final float TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (INCHES_PER_REV);

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int PIDLoopIDx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int TimeoutMs = 30;
    }

    public static enum Angle
    {
        FORWARD,
        REST;
    }

    // ===== MEMBERS ===== //

    public static Hanger.Angle _Angle;
    private static final WPI_TalonSRX lMotor = new WPI_TalonSRX(k.LEFT_MOTOR_ID);
    private static final WPI_TalonSRX rMotor = new WPI_TalonSRX(k.RIGHT_MOTOR_ID);
    private static final WPI_TalonSRX[] motors = new WPI_TalonSRX[]{lMotor, rMotor};
    private static final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    // ===== METHODS ===== //

    public static void init()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, k.PIDLoopIDx, k.TimeoutMs);
        }
        lMotor.setInverted(false);
        rMotor.setInverted(true);
    }

    public static void printData()
    {
        SmartDashboard.putString("Hanger Angle", _Angle.name());
        SmartDashboard.putString("Solenoid Value", solenoid.get().toString());
        SmartDashboard.putNumber("L Motor Power", lMotor.get());
        SmartDashboard.putNumber("R Motor Power", rMotor.get());

        SmartDashboard.putNumber("L Motor Position", lMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("R Motor Position", rMotor.getSelectedSensorPosition());
    }

    public static void raise()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.setNeutralMode(NeutralMode.Coast);
            motor.set(ControlMode.PercentOutput, k.upSpeed);
        }
    }

    public static void lower()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.setNeutralMode(NeutralMode.Brake);
            motor.set(ControlMode.PercentOutput, -1 * k.downSpeed);
        }
    }

    public static void stop()
    {
        for (WPI_TalonSRX motor : motors)
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