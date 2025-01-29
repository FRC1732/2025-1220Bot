// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralScoring extends SubsystemBase {
  private SparkMax coralMotor;

  /** Creates a new CoralScoring. */
  public CoralScoring() {
    coralMotor = new SparkMax(50, SparkBase.MotorType.kBrushed);
  }

  public void scores() {
    coralMotor.set(.30);
  }

  public void stop() {
    coralMotor.set(0);
  }

  public void reverse() {
    coralMotor.set(-.30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
