package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainEx extends Drivetrain {

  private static class kEx {

    private static final int PIGEON_ID = 0;
    private static final int xAxis = 0, yAxis = 1, zAxis = 2;
    private static final int rotAxis = xAxis;
    
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double sVolts = 0.22;  // TEMP - FIX IT!
    public static final double vVoltSecondsPerMeter = 1.98;  // TEMP - FIX IT!
    public static final double aVoltSecondsSquaredPerMeter = 0.2;  // TEMP - FIX IT!

    // Example value only - as above, this must be tuned for your drive!
    public static final double PDriveVel = 8.5;

    public static final double TrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics DriveKinematics =
        new DifferentialDriveKinematics(TrackwidthMeters);

    // For Ramsete controller. Only tune if needed.
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RamseteB = 2;
    public static final double RamseteZeta = 0.7;

    // Speed & Acceleration
    public static final double MaxSpeedMetersPerSecond = 3;
    public static final double MaxAccelerationMetersPerSecondSquared = 3;

    // “Pulse” refers to a full encoder cycle (i.e. four edges),
    // and thus will be 1/4 the value that was specified in the SysId config. (meters)
    public static final double EncoderDistancePerPulse = 0;  // TEMP - SysId
  }

  // The motors on the left side of the drive.
  private static final MotorControllerGroup leftMotors =
      new MotorControllerGroup(
          new PWMSparkMax(k.FL_ID),
          new PWMSparkMax(k.BL_ID));

  // The motors on the right side of the drive.
  private static final MotorControllerGroup rightMotors =
      new MotorControllerGroup(
          new PWMSparkMax(k.FR_ID),
          new PWMSparkMax(k.BR_ID));

  // The left-side drive encoder
  private static final Encoder leftEncoder =
      new Encoder(
          k.FL_ID,
          k.FR_ID,
          false);

  // The right-side drive encoder
  private static final Encoder rightEncoder =
      new Encoder(
          k.BL_ID,
          k.BL_ID,
          true);

  // The gyro sensor
  private static final PigeonIMU imu = new PigeonIMU(kEx.PIGEON_ID);

  // Odometry class for tracking robot pose
  private static DifferentialDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public static void init() {

    Drivetrain.init();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // Done in DT
    // rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(kEx.EncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(kEx.EncoderDistancePerPulse);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(getRotation2d());
  }

  public static void updateOdometry() {
    // Update the odometry in the periodic block
    odometry.update(getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public static Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public static void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public static void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public static void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public static double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public static Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public static Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public static void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public static void zeroHeading() {
    imu.setYaw(0);
  }

  public static void printEncoders()
  {
    SmartDashboard.putNumber("LEFT ENCODER", leftEncoder.get());
    SmartDashboard.putNumber("RIGHT ENCODER", rightEncoder.get());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public static double getHeading() {

    double res = getRawHeading()%360;
    if (res < 0)
      res += 360;
    return res;
  }

  public static double getRawHeading() {
    return -imu.getYaw();
  }

  /**
   * @return the heading of the robot as a Rotation2d
   */
  public static Rotation2d getRotation2d() {

    return Rotation2d.fromDegrees(getRawHeading());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public static double getTurnRate() {
    
    double[] res = new double[3];
    imu.getRawGyro(res);

    return res[kEx.rotAxis];
  }
}