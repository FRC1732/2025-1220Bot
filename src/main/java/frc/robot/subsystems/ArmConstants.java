package frc.robot.subsystems;

import frc.lib.team6328.util.LoggedTunableNumber;

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
  public static final LoggedTunableNumber armkP =
          new LoggedTunableNumber("Arm/kP", 17.5);
  public static final LoggedTunableNumber armkI =
          new LoggedTunableNumber("Arm/XYStdDevCoefficient", 0.0);
  public static final LoggedTunableNumber armkD =
          new LoggedTunableNumber("Arm/XYStdDevCoefficient", 0.8);

  //Define Arm Speeds
  public static final LoggedTunableNumber armIntakeSpeed =
          new LoggedTunableNumber("Arm/IntakeSpeed", -.5);
  public static final LoggedTunableNumber armScoringSpeed =
          new LoggedTunableNumber("Arm/ScoringSpeed", .5);
  public static final LoggedTunableNumber armUpSpeed =
          new LoggedTunableNumber("Arm/UpSpeed", .75);
  public static final LoggedTunableNumber armDownSpeed =
          new LoggedTunableNumber("Arm/DownSpeed", -.75);
  public static final LoggedTunableNumber armBrakeSpeed =
          new LoggedTunableNumber("Arm/DownSpeed", 0.0);

}
