package frc.robot.test_subsystems.Archived_Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainEx extends Drivetrain {

  public static class kEx {

    private static final int PIGEON_ID = 0;
    private static final int FWD_ID = 4, REV_ID = 3;

    private static final DoubleSolenoid.Value high = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value low = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;

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
    public static final double TrackWidthInches = 22;
    public static final double RobotTrackCircumference = 2 * TrackWidthInches * Math.PI;
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

  // The gyro sensor
  private final PigeonIMU imu = new PigeonIMU(kEx.PIGEON_ID);
  private final DoubleSolenoid shifter
    = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kEx.FWD_ID, kEx.REV_ID);

  public static enum Gear
  {
      HIGH,
      LOW;
  }

  public DrivetrainEx.Gear _Gear;
  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public DrivetrainEx() {

    super();
    setHighGear();

    // TODO: Instead of this, we need to just multiply by the distance per native unit.
    //leftEncoder.setDistancePerPulse(kEx.EncoderDistancePerPulse);
    //rightEncoder.setDistancePerPulse(kEx.EncoderDistancePerPulse);

    odometry = new DifferentialDriveOdometry(getRotation2d());
  }

  public void autoInit()
  {
    setHighGear();
    zeroHeading();
  }

  public void setHighGear()
  {
    _Gear = Gear.HIGH;
    shifter.set(kEx.high);
  }

  public void setLowGear()
  {
    _Gear = Gear.LOW;
    shifter.set(kEx.low);
  }

  public void updateOdometry() {
    // Update the odometry in the periodic block
    odometry.update(getRotation2d(), L_Master.getSelectedSensorPosition(), R_Master.getSelectedSensorPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      L_Master.getSelectedSensorVelocity(), R_Master.getSelectedSensorVelocity()
    );
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    L_Master.setVoltage(leftVolts);
    R_Master.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    
    L_Master.setSelectedSensorPosition(0);
    R_Master.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (L_Master.getSelectedSensorPosition() + R_Master.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.setYaw(0);
  }

  public void printEncoders()
  {
    SmartDashboard.putNumber("LEFT ENCODER", L_Master.getSelectedSensorPosition());
    SmartDashboard.putNumber("RIGHT ENCODER", R_Master.getSelectedSensorPosition());

    SmartDashboard.putNumber("LEFT VELOCITY", L_Master.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RIGHT VELOCITY", R_Master.getSelectedSensorVelocity());
  }

  public void printData()
  {
    printPowers();
    printEncoders();
    printIMUData();
  }

  public void printIMUData()
  {
    SmartDashboard.putNumber("Raw Heading", getRawHeading());
    SmartDashboard.putNumber("Abs Heading", getHeading());
    SmartDashboard.putNumber("Turn rate", getTurnRate());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {

    double res = getRawHeading()%360;
    if (res < 0)
      res += 360;
    return res;
  }

  public double getRawHeading() {
    return -imu.getYaw();
  }

  /**
   * @return the heading of the robot as a Rotation2d
   */
  public Rotation2d getRotation2d() {

    return Rotation2d.fromDegrees(getRawHeading());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    
    double[] res = new double[3];
    imu.getRawGyro(res);

    return res[kEx.rotAxis];
  }
}