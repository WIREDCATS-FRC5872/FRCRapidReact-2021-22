package frc.robot.test_subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public abstract class Pneumatics
{
    private static class k
    {
        private static final int COMPRESSOR_ID = 6;
    }

    private static final Compressor compressor = new Compressor(k.COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);
    public static final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public static void init()
    {
        compressor.enableDigital();
    }

    public static void toggleSolenoids() {
        doubleSolenoid.toggle();
    }

    public static Value getPosition()
    {
        return doubleSolenoid.get();
    }
}
