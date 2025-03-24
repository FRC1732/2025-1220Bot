package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedNumber {
  private static final String TABLE_KEY = "/Logging";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private DoubleSupplier supplier;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public LoggedNumber(String dashboardKey, DoubleSupplier supplier) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.supplier = supplier;

    dashboardNumber = new LoggedNetworkNumber(key, 0.0);
  }

  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return TUNING_MODE ? dashboardNumber.get() : defaultValue;
    }
  }

  public boolean hasChanged(int id) {
    double currentValue = supplier.getAsDouble();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }
}
