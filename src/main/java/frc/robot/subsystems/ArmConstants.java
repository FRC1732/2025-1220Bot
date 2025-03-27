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

  public static final double armStartingAngleDegrees = 74.0;
  public static final double armL1CoralAngleDegrees = 48.0;
  public static final double armL2AlgaePluckDegrees = 58.0;
  public static final double armFloorAngleDegrees = 0.7;
  public static final double armClimbingDegrees = 63.0;
  public static final double lowClimbAngleDegrees = 20.0;
  public static final double armAglaeCompletePluck = 83.0;
  public static final double armAglaeCarry = 35.0;
  public static final double armAglaeFloorPickup = 33.0;

  // Define Arm position limits
  public static Double armFrontLimitRadians = Math.PI / 2;
  public static Double armRearLimitRadians = 0.0;

  // Define Arm velocity limit
  public static Double armVelocityLimit = 0.8;

  // Define Arm PID constants
  public static final LoggedTunableNumber armkP = new LoggedTunableNumber("Arm/kP", 0.1);
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

  public static final double climbSpeed = -0.25;
  public static final String SUBSYSTEM_NAME = "Arm/";
}
