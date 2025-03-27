// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team3061.RobotConfig;
import frc.robot.configs.CompRobotConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmConstants;
import frc.robot.subsystems.ArmPose;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NewIntake;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  @SuppressWarnings("unused")
  private RobotConfig config;

  private OperatorInterface oi = new OperatorInterface() {};

  private Alliance lastAlliance = Field2d.getInstance().getAlliance();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final NewIntake newIntake = new NewIntake();
  public final Arm arm = new Arm();
  public final Climber climber = new Climber();

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1.0; // kSpeedAt12Volts desired top
  // speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1.0)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to
  // ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other
     * objects
     * that use it directly or indirectly. If this isn't done, a null pointer
     * exception will result.
     */
    createRobotConfig();
    defineSubsystems();

    // create the subsystems

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();
  }

  private void elasticSetup() {
    // TODO
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    switch (Constants.getRobot()) {
      case ROBOT_COMPETITION:
        config = new CompRobotConfig();
        break;
      default:
        break;
    }
  }

  private void defineSubsystems() {
    // CoralScoring instanciated above
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    Field2d.getInstance().setRegions(new Region2d[] {});
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }
    System.out.println(oi.getClass());

    // clear the list of composed commands since we are about to rebind them to
    // potentially new
    // triggers
    CommandScheduler.getInstance().clearComposedCommands();
    configureButtonBindings();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();

    configureSubsystemCommands();

    configureVisionCommands();

    // interrupt all commands by running a command that requires every subsystem.
    // This is used to
    // recover to a known state if the robot becomes "stuck" in a command.
    /*
     * oi.getInterruptAll()
     * .onTrue(
     * Commands.parallel(
     * Commands.runOnce(() -> subsystem.setMotorVoltage(0)),
     * new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY,
     * oi::getRotate)));
     */
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Event Markers
    NamedCommands.registerCommand("scoreCoral", newIntake.forwardIntake(() -> 1.0));

    new EventTrigger("Marker").onTrue(Commands.print("reached event marker"));
    new EventTrigger("ZoneMarker").onTrue(Commands.print("entered zone"));
    new EventTrigger("ZoneMarker").onFalse(Commands.print("left zone"));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
  }

  private void configureDrivetrainCommands() {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to
     * percentage of the maximum velocities. The velocities may be specified from
     * either the robot's frame of reference or the field's frame of reference. In
     * the robot's frame of reference, the positive x direction is forward; the
     * positive y direction, left; position rotation, CCW. In the field frame of
     * reference, the origin of the field to the lower left corner (i.e., the corner
     * of the field to the driver's right). Zero degrees is away from the driver and
     * increases in the CCW direction. This is why the left joystick's y axis
     * specifies the velocity in the x direction and the left joystick's x axis
     * specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -oi.getTranslateX() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -oi.getTranslateY() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -oi.getRotate()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // slow-mode toggle
    /*
     * oi.getTranslationSlowModeButton()
     * .onTrue(
     * Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain)
     * .withName("enable translation slow mode"));
     * oi.getTranslationSlowModeButton()
     * .onFalse(
     * Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain)
     * .withName("disable translation slow mode"));
     * oi.getRotationSlowModeButton()
     * .onTrue(
     * Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain)
     * .withName("enable rotation slow mode"));
     * oi.getRotationSlowModeButton()
     * .onFalse(
     * Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain)
     * .withName("disable rotation slow mode"));
     */

    // reset gyro to 0 degrees
    oi.getResetGyroButton()
        .onTrue(
            new PrintCommand("Reset gyro button pressed")
                .andThen(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d()))));

    // x-stance
    oi.getXStanceButton().whileTrue((drivetrain.applyRequest(() -> brake)));
  }

  private void configureSubsystemCommands() {
    // coral scoring
    oi.getCoralScoreTrigger()
        .whileTrue(newIntake.forwardIntake(ArmConstants.armScoringSpeed::get))
        .whileFalse(newIntake.forwardIntake(() -> 0.0).withName("coral stopping"));
    oi.getCoralReverseTrigger()
        .whileTrue(
            newIntake.forwardIntake(ArmConstants.armIntakeSpeed::get).withName("coral reverse"))
        .whileFalse(newIntake.forwardIntake(() -> 0.0).withName("coral stopping"));
    // oi.getUpArmTrigger().whileTrue(arm.upArm(ArmConstants.armUpSpeed.get()));
    // oi.getDownArmTrigger().whileTrue(arm.upArm(ArmConstants.armDownSpeed.get()));

    oi.operatorScoreTrigger()
        .whileTrue(newIntake.forwardIntake(ArmConstants.armScoringSpeed::get))
        .whileFalse(newIntake.forwardIntake(() -> 0.0));
    oi.operatorIntakeTrigger()
        .whileTrue(newIntake.forwardIntake(ArmConstants.armIntakeSpeed::get))
        .whileFalse(newIntake.forwardIntake(() -> 0.0));

    // arm control
    oi.getMoveToTroughTrigger()
        .onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.SCORE_CORAL)));
    oi.getMoveToL2Trigger().onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.ALGAE_L2)));
    oi.getMoveToFloorPickupTrigger()
        .onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.FLORAL)));

    oi.carryAlgaeTrigger().onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.CARRY_ALGAE)));
    oi.operatorCarryAlgaeTrigger()
        .onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.CARRY_ALGAE)));
    oi.operatorFloorPickupTrigger()
        .onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.ALGAE_FLOOR_PICKUP)));

    oi.pluckAglaeTrigger()
        .whileTrue(
            Commands.sequence(
                new InstantCommand(() -> arm.setArmPose(ArmPose.ALGAE_L2)),
                new WaitUntilCommand(() -> arm.isAtGoal()),
                newIntake.pluckAlgae(),
                newIntake.stop(),
                new InstantCommand(() -> arm.setArmPose(ArmPose.COMPLETE_ALGAE_PLUCK))));

    oi.operatorPluckAlgaeTrigger()
        .whileTrue(
            Commands.sequence(
                new InstantCommand(() -> arm.setArmPose(ArmPose.ALGAE_L2)),
                new WaitUntilCommand(() -> arm.isAtGoal()),
                newIntake.pluckAlgae(),
                newIntake.stop(),
                new InstantCommand(() -> arm.setArmPose(ArmPose.COMPLETE_ALGAE_PLUCK))));
    // new WaitUntilCommand(() -> arm.isAtGoal()),
    // new WaitCommand(1),
    // new InstantCommand(() -> arm.setArmPose(ArmPose.CARRY_ALGAE))));

    // Climbing
    oi.engageClimberWindmill().whileTrue(climber.runOnce(() -> climber.toggleWindmill()));
    oi.goToStartClimbPosition().onTrue(new InstantCommand(() -> arm.setArmPose(ArmPose.CLIMBING)));
    oi.goToRetractClimbPosition()
        .whileTrue(new InstantCommand(() -> arm.enableClimb(true)))
        .onFalse(new InstantCommand(() -> arm.enableClimb(false)));
  }

  private void configureVisionCommands() {
    // enable/disable vision

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {
    // add robot-wide periodic code here
  }

  public void disablePeriodic() {
    // empty for now
  }

  public void autonomousInit() {
    // add robot-wide code here that will be executed when autonomous starts
  }

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot
    // power cycled
    // during a match, this would be the first opportunity to check the alliance
    // color based on FMS
    // data.
    this.checkAllianceColor();
  }
}
