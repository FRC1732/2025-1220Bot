// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, 1 Xbox controller, and 1 operator button
 * panel.
 */
public class FullOperatorConsoleOI extends DualJoysticksOI {
  private final CommandXboxController operatorController;

  private final CommandJoystick operatorPanel;
  private final Trigger[] operatorPanelButtons;

  public FullOperatorConsoleOI(
      int translatePort, int rotatePort, int operatorControllerPort, int operatorPanelPort) {
    super(translatePort, rotatePort);
    operatorController = new CommandXboxController(operatorControllerPort);
    operatorPanel = new CommandJoystick(operatorPanelPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.operatorPanelButtons = new Trigger[13];
    for (int i = 1; i < operatorPanelButtons.length; i++) {
      operatorPanelButtons[i] = operatorPanel.button(i);
    }
  }

  // Operator Controller
  @Override
  public Trigger getInterruptAll() {
    return operatorController.start();
  }

  @Override
  public Trigger getSysIdDynamicForward() {
    return operatorController.back().and(operatorController.y());
  }

  @Override
  public Trigger getSysIdDynamicReverse() {
    return operatorController.back().and(operatorController.x());
  }

  @Override
  public Trigger getSysIdQuasistaticForward() {
    return operatorController.start().and(operatorController.y());
  }

  @Override
  public Trigger getSysIdQuasistaticReverse() {
    return operatorController.start().and(operatorController.x());
  }

  // Operator Panel

  @Override
  public Trigger getVisionIsEnabledSwitch() {
    return operatorPanelButtons[10];
  }
}
