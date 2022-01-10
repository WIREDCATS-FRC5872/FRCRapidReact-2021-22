package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

public abstract class Pneumatics
{
    private static class k
    {
        private static final int COMPRESSOR_ID = 6;
    }

    private static final Compressor compressor = new Compressor(k.COMPRESSOR_ID);

    public static void init()
    {
        compressor.setClosedLoopControl(true);
        compressor.start();
    }
}
