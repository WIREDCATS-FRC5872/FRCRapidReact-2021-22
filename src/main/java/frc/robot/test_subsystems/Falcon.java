package frc.robot.test_subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public abstract class Falcon
{
    private static class k
    {
        private static final int MOTOR_ID = 11;

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

    private static class Gains
    {
        public static final double kP = 0.205;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final int kIzone = 0;
        public static final double kPeakOutput = 1.0;
    }

    public static final WPI_TalonFX motor = new WPI_TalonFX(k.MOTOR_ID);

    public static void init()
    {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, k.PIDLoopIDx, k.TimeoutMs);
        motor.configNominalOutputForward(0, k.TimeoutMs);
		motor.configNominalOutputReverse(0, k.TimeoutMs);
		motor.configPeakOutputForward(1, k.TimeoutMs);
		motor.configPeakOutputReverse(-1, k.TimeoutMs);
        motor.configAllowableClosedloopError(0, k.PIDLoopIDx, k.TimeoutMs);
        motor.config_kP(k.PIDLoopIDx, Gains.kP, k.TimeoutMs);
		motor.config_kI(k.PIDLoopIDx, Gains.kI, k.TimeoutMs);
		motor.config_kD(k.PIDLoopIDx, Gains.kD, k.TimeoutMs);
        motor.config_kF(k.PIDLoopIDx, Gains.kF, k.TimeoutMs);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setSelectedSensorPosition(0);
    }
}
