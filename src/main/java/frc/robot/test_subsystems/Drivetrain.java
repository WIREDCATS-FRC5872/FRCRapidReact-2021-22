package frc.robot.test_subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public abstract class Drivetrain
{
    private static class k
    {
        private static final int FL_ID = 0, FR_ID = 1;
        private static final int BL_ID = 2, BR_ID = 3;
        private static final double TELEOP_SPEED = 1; // temp

        private static final boolean L_SENSOR_PHASE = true; // temp
        private static final boolean R_SENSOR_PHASE = false; // temp
    
        private static final double WHEEL_DIAMETER = 4.0;
        private static final double GEAR_RATIO = 1.0;
        private static final int TICKS_PER_REV = 4096;
        private static final double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

        // Set to zero to skip waiting for confirmation, set to nonzero to wait and
        // report to DS if action fails.
        private static final int timeoutMs = 30; // temp

        // Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
        // now we just want the primary one.
        private static final int PIDLoopIdx = 0;
    }

    // ===== MEMBERS ===== //

    private static WPI_TalonSRX L_Master;
    private static WPI_TalonSRX L_Slave;
    private static WPI_TalonSRX R_Master;
    private static WPI_TalonSRX R_Slave;
    private static DifferentialDrive drive;

    // ===== METHODS ===== //

    public Drivetrain() { }

    public static void init()
    {
        // Assign motors to ports
        L_Master = new WPI_TalonSRX(k.FL_ID);
        R_Master = new WPI_TalonSRX(k.FR_ID);
        L_Slave = new WPI_TalonSRX(k.BL_ID);
        R_Slave = new WPI_TalonSRX(k.BR_ID);
        drive = new DifferentialDrive(L_Master, R_Master);

        // Factory Default all hardware to prevent unexpected behaviour
        L_Master.configFactoryDefault();
        R_Master.configFactoryDefault();
        L_Slave.configFactoryDefault();
        R_Slave.configFactoryDefault();

        // Set followers
        L_Slave.follow(L_Master);
        R_Slave.follow(R_Master);

        // Reverse motors
        L_Master.setInverted(true);
        R_Master.setInverted(true);
        L_Slave.setInverted(InvertType.FollowMaster);
        R_Slave.setInverted(InvertType.FollowMaster);

        // Set motor neutral mode
        L_Master.setNeutralMode(NeutralMode.Brake);
        R_Master.setNeutralMode(NeutralMode.Brake);
        L_Slave.setNeutralMode(NeutralMode.Brake);
        R_Slave.setNeutralMode(NeutralMode.Brake);
    }

    public static void curvatureDrive(double leftY, double leftX, boolean isQuickTurn)
    {
        drive.curvatureDrive(leftY, leftX, isQuickTurn);
    }
}
