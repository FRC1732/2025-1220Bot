package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionTracking implements Subsystem {
  private final String NAME = "limelight-tabltop";

  PIDController rotationPID;
  NetworkTableEntry txEntry, tvEntry;

  double rotation = 0.0;

  public VisionTracking() {
    txEntry = NetworkTableInstance.getDefault().getTable(NAME).getEntry("tx");
    tvEntry = NetworkTableInstance.getDefault().getTable(NAME).getEntry("tv");

    rotationPID = new PIDController(1.0, 0.0, 0.0);
  }

  public double getTx() {
    return txEntry.getDouble(0.0);
  }

  public boolean getTv() {
    return 1.0 == tvEntry.getDouble(0.0);
  }

  public double getRotation(double def) {
    return getTv() ? MathUtil.clamp(rotation, -1.0, 1.0) : def;
  }

  @Override
  public void periodic() {
    if (getTv()) {
      rotation = rotationPID.calculate(getTx());
    } else {
      rotation = 0.0;
    }
    NetworkTableInstance.getDefault()
        .getTable("Teleoperated")
        .putValue("visionTarget", NetworkTableValue.makeBoolean(getTv()));
    NetworkTableInstance.getDefault()
        .getTable("Teleoperated")
        .putValue("visionRotation", NetworkTableValue.makeDouble(rotation));
  }
}
