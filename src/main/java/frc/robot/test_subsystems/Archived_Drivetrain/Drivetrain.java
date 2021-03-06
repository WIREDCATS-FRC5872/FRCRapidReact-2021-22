package frc.robot.test_subsystems.Archived_Drivetrain;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
    protected static class k
    {
        public static final int FL_ID = 0, FR_ID = 1;
        public static final int BL_ID = 2, BR_ID = 3;
    
        protected static final double WHEEL_DIAMETER = 4.0;
        protected static final double GEAR_RATIO = 7.0; // High gear = 7 motor rots = 1 shaft rot
        // Not listed: Low gear ratio
        protected static final int TICKS_PER_REV = 2048;
        protected static final double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

        // Set to zero to skip waiting for confirmation, set to nonzero to wait and
        // report to DS if action fails.
        protected static final int TimeoutMs = 30;

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int SlotIDx = 0;

        // Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
        // now we just want the primary one.
        protected static final int PIDLoopIDx = 0;
    }

    protected static class Gains
    {
        public static final double kP = 0.205;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final int kIzone = 0;
        public static final double kPeakOutput = 1.0;
    }

    // ===== MEMBERS ===== //

    protected WPI_TalonFX L_Master = new WPI_TalonFX(k.FL_ID);
    protected WPI_TalonFX R_Master = new WPI_TalonFX(k.FR_ID);
    protected WPI_TalonFX L_Slave = new WPI_TalonFX(k.BL_ID);
    protected WPI_TalonFX R_Slave = new WPI_TalonFX(k.BR_ID);
    protected DifferentialDrive drive;
    // Left motors even, right odd
    protected WPI_TalonFX[] DriveMotors = new WPI_TalonFX[]{L_Master, R_Master, L_Slave, R_Slave};

    // ===== METHODS ===== //

    public Drivetrain()
    {
        for (WPI_TalonFX motor : DriveMotors)
        {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);

            /* Set relevant frame periods to be at least as fast as periodic rate */
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, k.TimeoutMs);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, k.TimeoutMs);

            motor.configNeutralDeadband(0.001, k.TimeoutMs);

            motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, k.PIDLoopIDx, k.TimeoutMs);
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

        // Invert as necessary & assign to diff drive 
        L_Master.setInverted(false);
        R_Master.setInverted(true);

        // Set followers
        L_Slave.follow(L_Master);
        R_Slave.follow(R_Master);

        L_Slave.setInverted(InvertType.FollowMaster);
        R_Slave.setInverted(InvertType.FollowMaster);

        drive = new DifferentialDrive(L_Master, R_Master);
    }

    /**
    * Drives the robot using arcade controls.
    *
    * @param fwd the commanded forward movement
    * @param rot the commanded rotation
    */
    public void arcadeDrive(double leftY, double leftX)
    {
        drive.arcadeDrive(-leftY, leftX);
    }

    public void printData()
    {
        SmartDashboard.putNumber("L-Master Position", L_Master.getSelectedSensorPosition());
        SmartDashboard.putNumber("L-Slave Position", L_Slave.getSelectedSensorPosition());
        SmartDashboard.putNumber("R-Master Position", R_Master.getSelectedSensorPosition());
        SmartDashboard.putNumber("R-Slave Position", R_Slave.getSelectedSensorPosition());

        printPowers();
    }

    public void printPowers()
    {
        SmartDashboard.putNumber("L-Master Power", L_Master.get());
        SmartDashboard.putNumber("L-Slave Power", L_Slave.get());
        SmartDashboard.putNumber("R-Master Power", R_Master.get());
        SmartDashboard.putNumber("R-Slave Power", R_Slave.get());
    }
}
