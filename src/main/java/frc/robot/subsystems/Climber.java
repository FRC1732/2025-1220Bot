// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private SparkMax windmillMotor;
  private RelativeEncoder windmillRelativeEncoder;
  private boolean windmillEngaged;

  public static final double WINDMILL_TOLERANCE = 3.0;
  public static final double WINDMILL_FULLY_ENGAGED_SETPOINT = 90.0;
  public static final double RETREAT_TO_SAFE_BOUNDS_TIME = 0.2;
  public static final LoggedTunableNumber WINDMILL_SPEED =
      new LoggedTunableNumber("windmill speed", .50);

  public static final double WINDMILL_DEGREES_PER_ROTATION = 90.0;
  // degrees per motor revolution (360 / reduction = 360 / 4)
  public static final double WINDMILL_RPM_TO_DEGREES_PER_SECOND = 1.5;
  // RPM to deg/sec (360 / reduction / 60 = 360 / 4 / 60)

  public static final String SUBSYSTEM_NAME = "Climber";

  /** Creates a new Climber. */
  public Climber() {
    windmillMotor = new SparkMax(62, MotorType.kBrushless);

    SparkMaxConfig windmillConfig = new SparkMaxConfig();
    windmillConfig.inverted(true);
    windmillConfig.idleMode(IdleMode.kBrake);

    EncoderConfig windmillEncoderConfig = new EncoderConfig();
    windmillEncoderConfig.positionConversionFactor(WINDMILL_DEGREES_PER_ROTATION);
    windmillEncoderConfig.velocityConversionFactor(WINDMILL_RPM_TO_DEGREES_PER_SECOND);

    windmillConfig.apply(windmillEncoderConfig);
    windmillMotor.configure(
        windmillConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    windmillRelativeEncoder = windmillMotor.getEncoder();
    windmillRelativeEncoder.setPosition(0);
    windmillMotor.stopMotor();

    windmillEngaged = false;

    setupShuffleboard();
  }

  public double getWindmillPosition() {
    return windmillRelativeEncoder.getPosition();
  }

  public void runWindmill() {
    // System.out.println("Running windmill");
    windmillMotor.set(WINDMILL_SPEED.get());
  }

  public void reverseWindmill() {
    windmillMotor.set(-WINDMILL_SPEED.get());
  }

  public void stopWindmill() {
    // System.out.println("Stopping windmill");
    windmillMotor.stopMotor();
  }

  public void engageWindmill() {
    // System.out.println("Engaging windmill");
    windmillEngaged = true;
  }

  public void toggleWindmill() {
    windmillEngaged = !windmillEngaged;
  }

  @Override
  public void periodic() {
    if (windmillEngaged
        && getWindmillPosition() < WINDMILL_FULLY_ENGAGED_SETPOINT - WINDMILL_TOLERANCE) {
      runWindmill();
    } else if (!windmillEngaged && getWindmillPosition() > WINDMILL_TOLERANCE) {
      reverseWindmill();
    } else {
      stopWindmill();
    }

    doLogging();
  }

  private void doLogging() {
    Logger.recordOutput(SUBSYSTEM_NAME + "/Windmill Engaged", windmillEngaged);
    Logger.recordOutput(SUBSYSTEM_NAME + "/Windmill Position", getWindmillPosition());
    Logger.recordOutput(SUBSYSTEM_NAME + "/Windmill Speed", windmillMotor.get());
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    tab.addDouble("windmill Position", this::getWindmillPosition);
    tab.addBoolean("Windmill Engaged", () -> windmillEngaged);
    tab.addDouble("Windmill Speed", windmillMotor::get);
  }
}
