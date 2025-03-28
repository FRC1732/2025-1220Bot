package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class NewIntake extends SubsystemBase {

  SparkMax topIntakeMotor, bottomIntakeMotor;
  SparkMaxConfig intakeMotor1Config, intakeMotor2Config;

  /** Creates a new Intake. */
  public NewIntake() {
    topIntakeMotor = new SparkMax(51, MotorType.kBrushed);
    bottomIntakeMotor = new SparkMax(53, MotorType.kBrushed);

    intakeMotor1Config = new SparkMaxConfig();
    intakeMotor2Config = new SparkMaxConfig();

    topIntakeMotor.configure(
        intakeMotor1Config.inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    bottomIntakeMotor.configure(
        intakeMotor2Config.inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");
  }

  public Command forwardIntake(DoubleSupplier velocity) {
    // Inline construction of command goes here.
    return run(
        () -> {
          topIntakeMotor.set(velocity.getAsDouble());
          bottomIntakeMotor.set(velocity.getAsDouble());
        });
  }

  public Command scoreForever(DoubleSupplier velocity) {
    // Inline construction of command goes here.
    return runOnce(
        () -> {
          System.out.println("Running forever:" + -velocity.getAsDouble());
          topIntakeMotor.set(-velocity.getAsDouble());
          bottomIntakeMotor.set(-velocity.getAsDouble());
        });
  }

  public Command reverseIntake(Double velocity) {
    // Inline construction of command goes here.
    return run(
        () -> {
          topIntakeMotor.set(-velocity);
          bottomIntakeMotor.set(-velocity);
        });
  }

  public Command pluckAlgae() {
    return run(() -> topIntakeMotor.set(-0.2)).withTimeout(1.0);
  }

  public Command stop() {
    return runOnce(
        () -> {
          topIntakeMotor.set(0);
          bottomIntakeMotor.set(0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
