// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.CompRobotConfig;

public class CoralScoring extends SubsystemBase {
  private SparkMax coralMotor;

  /** Creates a new CoralScoring. */
  public CoralScoring() {
    coralMotor = new SparkMax(50, SparkBase.MotorType.kBrushless);
  }

  public void scores() {

    coralMotor.set(-CompRobotConfig.FORWARD_SCORE_PERCENT);
  }

  public void stop() {
    coralMotor.set(0);
  }

  public void reverse() {
    coralMotor.set(CompRobotConfig.BACKWARD_SCORE_PERCENT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
