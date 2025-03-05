package frc.robot.subsystems;

public class ArmConstants {
  // Define Arm position constants
  public static Double positionIntakeCoral = 0.422;
  public static Double positionClimbEnd = 0.368;
  public static Double positionIntakeAlgae = 0.348;
  public static Double positionRemoveAlgaeLow = 0.3083;
  public static Double positionClimbStart = 0.233;
  public static Double positionRemoveAlgaeHigh = 0.1;

  // Define Arm position limits
  public static Double armFrontLimit = 0.422;
  public static Double armRearLimit = 0.05;

  // Define Arm velocity limit
  public static Double armVelocityLimit = 0.8;

  // Define Arm PID constants
  public static Double armkP = 17.5;
  public static Double armkI = 0.0;
  public static Double armkD = 0.8;
}
