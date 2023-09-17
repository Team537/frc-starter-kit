package frc.robot.utils;

import java.util.*;




/**
 * Contains functions to convert {@link Map}s with {@link ModulePosition} keys to and from arrays so
 * that it's easier to use WPILib swerve functions.
 */
public class ModuleMap {

 
    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
      }

  @SafeVarargs
  public static <V> Map<ModulePosition, V> of(V... values) {
    Map<ModulePosition, V> map = new HashMap<>();
    for (int i = 0; i < ModulePosition.values().length; i++) {
      map.put(ModulePosition.values()[i], values[i]);
    }
    return map;
  }

 
  public static <V> List<V> orderedValuesList(Map<ModulePosition, V> map) {
    ArrayList<V> list = new ArrayList<>();
    for (ModulePosition i : ModulePosition.values()) {
      list.add(map.get(i));
    }
    return list;
  }

 
  public static <V> V[] orderedValues(Map<ModulePosition, V> map, V[] array) {
    return orderedValuesList(map).toArray(array);
  }
}
