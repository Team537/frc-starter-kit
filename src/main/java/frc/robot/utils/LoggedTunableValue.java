// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import frc.robot.config.YAMLDataHolder;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or
 * value not in dashboard.
 */
public class LoggedTunableValue {
  private static final String tableKey = "TunableNumbers";
  private YAMLDataHolder yamlDataHolder = YAMLDataHolder.getInstance();

  private final String key;
  private final String yamlKey;
  private boolean hasDefault = false;
  private Object defaultValue;
  private LoggedDashboardNumber dashboardNumber;
  private LoggedDashboardString dashboardString;
  private LoggedDashboardBoolean dashboardBoolean;
  private Map<Integer, Object> lastHasChangedValues = new HashMap<>();
  private boolean TUNING_MODE = yamlDataHolder.isTuningMode();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableValue(String dashboardKey, String yamlKey) {
    this.key = tableKey + "/" + dashboardKey;
    this.yamlKey = yamlKey;
    initDefault(yamlDataHolder.getProperty(yamlKey));
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(Object defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      if (defaultValue instanceof Double) {
        dashboardNumber = new LoggedDashboardNumber(key, (double) defaultValue);
      }

      if (defaultValue instanceof Integer) {
        dashboardNumber = new LoggedDashboardNumber(key, (int) defaultValue);
      }

      if (defaultValue instanceof Boolean) {
        dashboardBoolean = new LoggedDashboardBoolean(key, (boolean) defaultValue);
      }

      if (defaultValue instanceof String) {
        dashboardString = new LoggedDashboardString(key, (String) defaultValue);
      }

    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double getDouble() {

    if (!hasDefault) {
      return 0.0;
    } else if (TUNING_MODE) {
      return dashboardNumber.get();
    } else {
      setDouble((double) defaultValue);
      return (double) defaultValue;
    }
  }

  public int getInteger() {

    if (!hasDefault) {
      return 0;
    } else if (TUNING_MODE) {
      return (int) dashboardNumber.get();
    } else {
      setInteger((int) defaultValue);
      return (int) defaultValue;
    }
  }

  public String getString() {

    if (!hasDefault) {
      return "";
    } else if (TUNING_MODE) {
      return dashboardString.get();
    } else {
      setString(defaultValue.toString());
      return defaultValue.toString();
    }
  }

  public Boolean getBool() {

    if (!hasDefault) {
      return false;
    } else if (TUNING_MODE) {
      return dashboardBoolean.get();
    } else {
      setBool((boolean) defaultValue);
      return (boolean) defaultValue;
    }
  }

  public void setDouble(double value) {
    dashboardNumber.set(value);

  }

  public void setInteger(int value) {
    dashboardNumber.set(value);

  }

  public void setString(String value) {
    dashboardString.set(value);

  }

  public void setBool(boolean value) {
    dashboardBoolean.set(value);

  }

  public void periodic() {
    TUNING_MODE = yamlDataHolder.isTuningMode();

    if (defaultValue instanceof Double) {
      yamlDataHolder.setProperty(yamlKey, getDouble());
    }
    if (defaultValue instanceof Integer) {
      yamlDataHolder.setProperty(yamlKey, getInteger());
    }
    if (defaultValue instanceof Boolean) {
      yamlDataHolder.setProperty(yamlKey, getBool());
    }
    if (defaultValue instanceof String) {
      yamlDataHolder.setProperty(yamlKey, getString());
    }

  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared
   *           between multiple
   *           objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was
   *         called, false
   *         otherwise.
   */
  public boolean hasChanged(int id) {

    Object currentValue;

    if (defaultValue instanceof Double) {
      currentValue = getDouble();
    }
    if (defaultValue instanceof Integer) {
      currentValue = getInteger();
    }
    if (defaultValue instanceof Boolean) {
      currentValue = getBool();
    }

    if (defaultValue instanceof String) {
      currentValue = getString();
    } else {
      currentValue = null;
    }

    Object lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);

      return true;
    }

    return false;
  }
}