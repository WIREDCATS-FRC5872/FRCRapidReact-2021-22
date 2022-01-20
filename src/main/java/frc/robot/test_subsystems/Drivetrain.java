package frc.robot.test_subsystems;
<<<<<<< Updated upstream
=======
<<<<<<< HEAD

import java.util.ArrayList;
=======
>>>>>>> d86abb10be89667d9f4571686345668cd8d16dae
>>>>>>> Stashed changes

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public abstract class Drivetrain
{
    private static class k
    {
        private static final int FL_ID = 0, FR_ID = 1;
        private static final int BL_ID = 2, BR_ID = 3;
    
        private static final double WHEEL_DIAMETER = 4.0;
        private static final double GEAR_RATIO = 1.0;
        private static final int TICKS_PER_REV = 4096;
        private static final double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

        // Set to zero to skip waiting for confirmation, set to nonzero to wait and
        // report to DS if action fails.
        private static final int TimeoutMs = 30;

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int SlotIDx = 0;

        // Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
        // now we just want the primary one.
        private static final int PIDLoopIDx = 0;
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

    // ===== MEMBERS ===== //

    private static final WPI_TalonSRX L_Master = new WPI_TalonSRX(k.FL_ID);
    private static final WPI_TalonSRX L_Slave = new WPI_TalonSRX(k.BL_ID);
    private static final WPI_TalonSRX R_Master = new WPI_TalonSRX(k.FR_ID);
    private static final WPI_TalonSRX R_Slave = new WPI_TalonSRX(k.BR_ID);
    private static ArrayList<WPI_TalonSRX> DriveMotors = new ArrayList<WPI_TalonSRX>();
    private static final DifferentialDrive drive = new DifferentialDrive(L_Master, R_Master);

    // ===== METHODS ===== //

    public Drivetrain() { }

    public static void init()
    {
        DriveMotors.add(L_Master);
        DriveMotors.add(L_Slave);
        DriveMotors.add(R_Master);
        DriveMotors.add(R_Slave);

        for(WPI_TalonSRX motor : DriveMotors)
        {
            motor.configFactoryDefault();
            motor.setInverted(true);
            motor.setNeutralMode(NeutralMode.Brake);

            /* Set relevant frame periods to be at least as fast as periodic rate */
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, k.TimeoutMs);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, k.TimeoutMs);

            motor.configNeutralDeadband(0.001, k.TimeoutMs);

            motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, k.PIDLoopIDx, k.TimeoutMs);
            motor.configNominalOutputForward(0, k.TimeoutMs);
            motor.configNominalOutputReverse(0, k.TimeoutMs);
            motor.configPeakOutputForward(1, k.TimeoutMs);
            motor.configPeakOutputReverse(-1, k.TimeoutMs);
            motor.configAllowableClosedloopError(0, k.PIDLoopIDx, k.TimeoutMs);
            
            motor.selectProfileSlot(k.SlotIDx, k.PIDLoopIDx);
            motor.config_kP(k.SlotIDx, Gains.kP, k.TimeoutMs);
            motor.config_kI(k.SlotIDx, Gains.kI, k.TimeoutMs);
            motor.config_kD(k.SlotIDx, Gains.kD, k.TimeoutMs);
            motor.config_kF(k.SlotIDx, Gains.kF, k.TimeoutMs);
            
            motor.configMotionCruiseVelocity(0, k.TimeoutMs);
            motor.configMotionAcceleration(0, k.TimeoutMs);

            motor.setSelectedSensorPosition(0, k.PIDLoopIDx, k.TimeoutMs);
        }

        // Set followers
        L_Slave.follow(L_Master);
        R_Slave.follow(R_Master);
    }

    public static void curvatureDrive(double leftY, double leftX, boolean isQuickTurn)
    {
        drive.curvatureDrive(leftY, leftX, isQuickTurn);
    }
}
