// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final CommandJoystick controller;
  private final Trigger[] triggerButtons;

  public SingleHandheldOI(int port) {
    controller = new CommandJoystick(port);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.triggerButtons = new Trigger[11];

    for (int i = 1; i < triggerButtons.length; i++) {
      triggerButtons[i] = controller.button(i);
    }
  }

  @Override
  public double getTranslateX() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double getTranslateY() {
    return -controller.getRawAxis(0);
  }

  @Override
  public double getRotate() {
    return -controller.getRawAxis(4);
  }

  @Override
  public Trigger getResetGyroButton() {
    return triggerButtons[4];
  }

  @Override
  public Trigger getXStanceButton() {
    return triggerButtons[3];
  }

  @Override
  public Trigger getCoralScoreTrigger() {
    // double value1 = controller.getRawAxis(2);
    // double value2 = controller.getRawAxis(3);
    return new Trigger(() -> controller.getRawAxis(2) > 0.5 || controller.getRawAxis(3) > 0.5);
  }

  @Override
  public Trigger getCoralReverseTrigger() {
    return triggerButtons[5].or(triggerButtons[6]);
  }
}
