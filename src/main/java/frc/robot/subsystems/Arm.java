package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  SparkMax armMotor1, armMotor2, climbMotor;
  SparkMaxConfig armMotor1Config, armMotor2Config, climbMotorConfig;
  DutyCycleEncoder encoder;
  PIDController armP;
  Double armFrontLimit, armRearLimit, armVelocityLimit;
  boolean isInDefaultPosition;


  /** Creates a new Arm. */
  public Arm() {
    armMotor1 = new SparkMax(60, MotorType.kBrushed);
    armMotor2 = new SparkMax(61, MotorType.kBrushed);

    climbMotor = new SparkMax(61, MotorType.kBrushless);

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
            ResetMode.kNoResetSafeParameters, //TODO: add reset params
            PersistMode.kPersistParameters);

    encoder = new DutyCycleEncoder(0);
    armP = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);
  }

public Command forwardArm(Double velocity) {
  return run(
    () -> {
      armMotor1.set(velocity);
      armMotor2.set(velocity);
    });
}

public Command reverseArm(Double velocity) {
  return run(
          () -> {
            armMotor1.set(-velocity);
            armMotor2.set(-velocity);
          });
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
    }).onlyWhile(() -> a.get() < CLIMB_TURNING_TIME);

  }
  
  // public Command moveArmToPosition(Double position) {
  //   return run(
  //       () -> {

  //         // Get the target position, clamped to (limited between) the lowest and highest arm
  //         // positions
  //         Double target =
  //             MathUtil.clamp(position, ArmConstants.armRearLimit, ArmConstants.armFrontLimit);

  //         // Calculate the PID result, and clamp to the arm's maximum velocity limit.
  //         Double result =
  //             MathUtil.clamp(
  //                 armP.calculate(encoder.get(), target),
  //                 -1 * ArmConstants.armVelocityLimit,
  //                 ArmConstants.armVelocityLimit);

  //         armMotor1.set(result);
  //       });
  // }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
