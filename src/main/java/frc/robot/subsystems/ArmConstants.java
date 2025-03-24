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
  public static LoggedTunableNumber armTroughAngle =
      new LoggedTunableNumber("Arm/TroughAngle", .81); // 61.0 degrees
  public static LoggedTunableNumber armL2Angle =
      new LoggedTunableNumber("Arm/L2Angle", Math.PI / 2.0); // 89.0 degrees]
  public static LoggedTunableNumber armFloorAngle = new LoggedTunableNumber("Arm/FloorAngle", 0.0);
  // Define Arm position limits
  public static Double armFrontLimit = Math.PI / 2;
  public static Double armRearLimit = 0.0;

  // Define Arm velocity limit
  public static Double armVelocityLimit = 0.8;

  // Define Arm PID constants
  public static final LoggedTunableNumber armkP = new LoggedTunableNumber("Arm/kP", 6.0);
  public static final LoggedTunableNumber armkI = new LoggedTunableNumber("Arm/kI", 0.0);
  public static final LoggedTunableNumber armkD = new LoggedTunableNumber("Arm/kD", 0.0);

  // Define Arm Speeds
  public static final LoggedTunableNumber armIntakeSpeed =
      new LoggedTunableNumber("Arm/IntakeSpeed", -.2);
  public static final LoggedTunableNumber armScoringSpeed =
      new LoggedTunableNumber("Arm/ScoringSpeed", .5);
  public static final LoggedTunableNumber armUpSpeed = new LoggedTunableNumber("Arm/UpSpeed", .75);
  public static final LoggedTunableNumber armDownSpeed =
      new LoggedTunableNumber("Arm/DownSpeed", -.75);
  public static final LoggedTunableNumber armBrakeSpeedTheta =
      new LoggedTunableNumber("Arm/BrakeSpeed", 0.0);
  public static final LoggedTunableNumber armAngleOffset =
      new LoggedTunableNumber("Arm/Offset", 0.638);
}
