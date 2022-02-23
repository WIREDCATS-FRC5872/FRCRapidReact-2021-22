package frc.robot;

public class RobotException extends Exception
{
    public RobotException() { };
    public RobotException(String message)
    {
        super(message);
    }
}
