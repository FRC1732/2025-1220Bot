package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class Arm extends SubsystemBase {
  private SparkMax armMotor1, armMotor2, climbMotor;
  private SparkMaxConfig armMotor1Config, armMotor2Config, climbMotorConfig;

  private PIDController armP;
  private boolean isInDefaultPosition;

  private AbsoluteEncoder armAbsoluteEncoder;

  private Map<ArmPose, Double> armPoseMap;

  private boolean isClimbing = false;

  /** Creates a new Arm. */
  public Arm() {
    armMotor1 = new SparkMax(60, MotorType.kBrushless);
    armMotor2 = new SparkMax(61, MotorType.kBrushless);

    climbMotor = new SparkMax(62, MotorType.kBrushless);

    armMotor1Config = new SparkMaxConfig();
    armMotor2Config = new SparkMaxConfig();

    climbMotorConfig = new SparkMaxConfig();

    armMotor1.configure(
        armMotor1Config.inverted(true).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    armMotor2.configure(
        armMotor2Config.follow(armMotor1, true).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    climbMotor.configure(
        climbMotorConfig.idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters, // TODO: add reset params
        PersistMode.kPersistParameters);

    armAbsoluteEncoder = armMotor1.getAbsoluteEncoder();
    armP =
        new PIDController(
            ArmConstants.armkP.get(), ArmConstants.armkI.get(), ArmConstants.armkD.get());

    armPoseMap =
        Map.of(
            ArmPose.STARTING, ArmConstants.armStartingAngleDegrees,
            ArmPose.FLORAL, ArmConstants.armFloorAngleDegrees,
            ArmPose.ALGAE_L2, ArmConstants.armL2AngleDegrees,
            ArmPose.CLIMBING, ArmConstants.armClimbingDegrees,
            ArmPose.SCORE_CORAL, ArmConstants.armL1CoralAngleDegrees);

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    armTab.add(armP);
    armTab.addDouble("Arm Position", this::getPosition);
    armTab.addDouble("Absolute Arm Position", armAbsoluteEncoder::getPosition);
    armTab.addDouble("Setpoint", armP::getSetpoint);
  }

  public Command upArm(Double velocity) {
    return run(
        () -> {
          armMotor1.set(velocity);
        });
  }

  @Deprecated
  private Command breakArm() {
    double dTheta =
        Math.sin(armMotor1.getAbsoluteEncoder().getPosition() * 2 * Math.PI)
            * ArmConstants.armBrakeSpeedTheta.get();
    return run(
        () -> {
          armMotor1.set(dTheta);
        });
  }

  public Command downArm(Double velocity) {
    return run(
        () -> {
          armMotor1.set(-velocity);
        });
  }

  public void setArmPose(ArmPose pose) {
    armP.setSetpoint(armPoseMap.get(pose));
  }

  double CLIMB_TURNING_TIME = 1.0;

  public Command turnClimbMotor() {
    Timer a = new Timer();
    return run(() -> {
          if (isInDefaultPosition) {
            climbMotor.set(-0.1);
          } else {
            climbMotor.set(0.1);
          }
        })
        .onlyWhile(() -> a.get() < CLIMB_TURNING_TIME);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      // armP.reset(getPosition());
    }

    if (isClimbing) {
      if (getPosition() > ArmConstants.lowClimbAngleDegrees) {
        armMotor1.set(ArmConstants.climbSpeed);
      } else {
        isClimbing = false;
        armP.setSetpoint(ArmConstants.lowClimbAngleDegrees);
        armMotor1.set(MathUtil.clamp(armP.calculate(getPosition()), -0.5, 0.5));
      }
    } else {
      armMotor1.set(MathUtil.clamp(armP.calculate(getPosition()), -0.5, 0.5));
    }

    doLogging();
  }

  private void doLogging() {}

  private double getPosition() {
    return MathUtil.inputModulus((armAbsoluteEncoder.getPosition() - 0.64) * 360.0, 0, 360);
  }

  public void enableClimb(boolean isClimbing) {
    this.isClimbing = isClimbing;
  }
}
