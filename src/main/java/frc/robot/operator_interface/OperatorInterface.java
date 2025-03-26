// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.*;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getCoralScoreTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getCoralReverseTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getUpArmTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getDownArmTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToTroughTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToL2Trigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToFloorPickupTrigger() {
    return new Trigger(() -> false);
  }

  // drivetrain, game-specific

  // miscellaneous
  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  public default Trigger goToStartClimbPosition() {
    return new Trigger(() -> false);
  }

  public default Trigger goToRetractClimbPosition() {
    return new Trigger(() -> false);
  }

  public default Trigger engageClimberWindmill() {
    return new Trigger(() -> false);
  }

  public default Trigger pluckAglaeTrigger() {
    return new Trigger(() -> false);
  }
}
