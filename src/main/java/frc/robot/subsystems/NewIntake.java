package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewIntake extends SubsystemBase {
  SparkMax intakeMotor1, intakeMotor2;
  SparkMaxConfig intakeMotor1Config, intakeMotor2Config;

  /** Creates a new Intake. */
  public NewIntake() {
    intakeMotor1 = new SparkMax(8, MotorType.kBrushed);
    intakeMotor2 = new SparkMax(9, MotorType.kBrushed);

    intakeMotor1Config = new SparkMaxConfig();
    intakeMotor2Config = new SparkMaxConfig();

    intakeMotor1.configure(
        intakeMotor1Config.inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeMotor2.configure(
        intakeMotor2Config.idleMode(IdleMode.kBrake).follow(8),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public Command moveIntake(Double velocity) {
    // Inline construction of command goes here.
    return run(
        () -> {
          intakeMotor1.set(velocity);
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
