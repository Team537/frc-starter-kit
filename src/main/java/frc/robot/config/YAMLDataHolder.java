package frc.robot.config;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.yaml.snakeyaml.Yaml;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class YAMLDataHolder {
   
    private Map<String, Object> properties  = loadPropertiesFromFile();
    private boolean tuningMode = (boolean) getProperty("tuningMode");


  

    public Object getProperty(String key) {
        return properties.get(key);
    }

    public void setProperty(String key, Object value) {
        properties.put(key, value);
    }

    public void saveData() {
        savePropertiesToFile();
        
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> loadPropertiesFromFile() {
        try {
            Yaml yaml = new Yaml();
            FileReader reader = new FileReader(Filesystem.getDeployDirectory() + "/resources/Constants.yaml");
            return yaml.load(reader);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return new HashMap<>(); // Return an empty map if loading fails
    }

    private void savePropertiesToFile() {
        try {
            Yaml yaml = new Yaml();
            FileWriter writer = new FileWriter(Filesystem.getDeployDirectory() + "/resources/Constants.yaml");
            yaml.dump(properties, writer);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public void init() {
   

    }

    public void periodic() {

       
        
        if(tuningMode){
      
        }
        // saveData();


    }
}
