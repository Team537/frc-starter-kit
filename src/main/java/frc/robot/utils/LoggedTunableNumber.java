// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//


package frc.robot.utils;


import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.robot.config.YAMLDataHolder;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
  private static final String tableKey = "TunableNumbers";
  private YAMLDataHolder m_constants = YAMLDataHolder.getInstance();

  private final String key;
  private final String yamlKey;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedDashboardNumber dashboardNumber;
  private Map<Integer, Object> lastHasChangedValues = new HashMap<>();
  private boolean tuningMode = true;
  
  

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
    this.yamlKey = dashboardKey;
    initDefault((double) m_constants.getProperty(yamlKey));
  }

 


  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      
        dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
      
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public Object get() {
    
    if (!hasDefault) {
      return 0.0;
    } else {
      return tuningMode ? dashboardNumber.get() : defaultValue;
    }
  }

  public void set( double value) {
    dashboardNumber.set(value);
   
  }

  public void periodic() {
    m_constants.setProperty(yamlKey, get());
  }

  

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    Object currentValue = get();
    Object lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
    
      return true;
    }

    return false;
  }
}