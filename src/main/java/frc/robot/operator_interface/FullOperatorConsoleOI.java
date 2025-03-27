// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, 1 Xbox controller, and 1 operator button
 * panel.
 */
public class FullOperatorConsoleOI extends DualJoysticksOI {

  private final CommandJoystick operatorPanel;
  private final Trigger[] operatorPanelButtons;
  private final Trigger[] operatorPanelPOV;

  public FullOperatorConsoleOI(int translatePort, int rotatePort, int operatorPanelPort) {
    super(translatePort, rotatePort);
    operatorPanel = new CommandJoystick(operatorPanelPort);

    // buttons use 1-based indexing such that the index matches the button number;
    // leave index 0 set
    // to null
    this.operatorPanelButtons = new Trigger[13];
    for (int i = 1; i < operatorPanelButtons.length; i++) {
      operatorPanelButtons[i] = operatorPanel.button(i);
    }

    this.operatorPanelPOV = new Trigger[8];
    for (int i = 0; i < 8; i++) {
      operatorPanelPOV[i] = operatorPanel.pov(i * 45);
    }
  }

  // Operator Panel
  @Override
  public Trigger goToStartClimbPosition() {
    return operatorPanelButtons[2];
  }

  @Override
  public Trigger goToRetractClimbPosition() {
    return operatorPanelButtons[3];
  }

  @Override
  public Trigger engageClimberWindmill() {
    return operatorPanelButtons[1];
  }

  @Override
  public Trigger operatorPluckAlgaeTrigger() {
    return operatorPanelPOV[0]; // UP
  }

  @Override
  public Trigger operatorCarryAlgaeTrigger() {
    return operatorPanelPOV[6]; // Down
  }

  @Override
  public Trigger operatorIntakeTrigger() {
    return operatorPanelButtons[6];
  }

  @Override
  public Trigger operatorScoreTrigger() {
    return operatorPanelButtons[5];
  }

  @Override
  public Trigger operatorFloorPickupTrigger() {
    return operatorPanelPOV[4]; // Down
  }
}
