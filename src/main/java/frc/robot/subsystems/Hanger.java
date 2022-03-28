package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger
{
    private static class k
    {
        private static final int LEFT_MOTOR_ID = 16;
        private static final int RIGHT_MOTOR_ID = 17;
        private static final int FWD_ID = 2, REV_ID = 5;

        private static final DoubleSolenoid.Value forward = DoubleSolenoid.Value.kForward;
        private static final DoubleSolenoid.Value rest = DoubleSolenoid.Value.kReverse;
        private static final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;
        
        private static final float upSpeed = -0.5f;
        private static final float downSpeed = 1.0f;
        private static final int MAX_HEIGHT = 1200000;
        public static int MIN_HEIGHT;

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

    public Hanger.Angle _Angle;
    private final WPI_TalonSRX lMotor = new WPI_TalonSRX(k.LEFT_MOTOR_ID);
    private final WPI_TalonSRX rMotor = new WPI_TalonSRX(k.RIGHT_MOTOR_ID);
    private final WPI_TalonSRX[] motors = new WPI_TalonSRX[]{lMotor, rMotor};
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, k.FWD_ID, k.REV_ID);

    // ===== METHODS ===== //

    public Hanger()
    {
        for (WPI_TalonSRX motor : motors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, k.PIDLoopIDx, k.TimeoutMs);
        }
        lMotor.setInverted(true);
        rMotor.setInverted(false);
        resetEncoders();
    }

    public void init()
    {
        stop();
        rest();
        k.MIN_HEIGHT = 0;
    }

    public void testInit()
    {
        k.MIN_HEIGHT = Integer.MIN_VALUE;
    }

    public void printData()
    {
        SmartDashboard.putString("Hanger Angle", _Angle.name());
        SmartDashboard.putString("Hanger Solenoid Value", solenoid.get().toString());
        //SmartDashboard.putNumber("L Motor Power", lMotor.get());
        //SmartDashboard.putNumber("R Motor Power", rMotor.get());

        SmartDashboard.putNumber("Hanger L Motor Position", lMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Hanger R Motor Position", rMotor.getSelectedSensorPosition());
    }

    public void raise()
    {
        if (getAverageEncoderDistance() < k.MAX_HEIGHT)
            for (WPI_TalonSRX motor : motors)
            {
                motor.setNeutralMode(NeutralMode.Brake);
                motor.set(ControlMode.PercentOutput, k.upSpeed);
            }
        else
            stop();
    }

    public void lower()
    {
        if (getAverageEncoderDistance() > k.MIN_HEIGHT)
            for (WPI_TalonSRX motor : motors)
            {
                motor.setNeutralMode(NeutralMode.Brake);
                motor.set(ControlMode.PercentOutput, k.downSpeed);
            }
        else
            stop();
    }

    public void stop()
    {
        for (WPI_TalonSRX motor : motors)
            motor.set(ControlMode.PercentOutput, 0);
    }

    public void forward()
    {
        solenoid.set(k.forward);
        _Angle = Angle.FORWARD;
    }

    public void rest()
    {
        solenoid.set(k.rest);
        _Angle = Angle.REST;
    }

    /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    
    lMotor.setSelectedSensorPosition(0);
    rMotor.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (lMotor.getSelectedSensorPosition() + rMotor.getSelectedSensorPosition()) / 2.0;
  }
}