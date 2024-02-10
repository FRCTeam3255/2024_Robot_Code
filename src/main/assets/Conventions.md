# Naming Conventions
These are general conventions and do not need to be exactly followed. Follow these conventions when it makes sense to. We have 6 weeks to build a robot; if it works it works 🤷‍♂️

## General
- Classes should be UpperCamelCase
- Methods should be lowerCamelCase
- Local & Instance Variables should be lowerCamelCase

---
## Units
| Unit Type | Preferred Unit to Use |
| ---------- | ------------ |
| Distance | Meters |
| Distance per Time  | Meters per Second |
| Angle | Degrees |
| Angle per Time | Degrees per Second |
| Time | Seconds |

- If the unit does not fall under any of these types and is not readily apparent from the variable type/name, add a JavaDoc for that variable specifying it's unit. Avoid unnecessarily specifying units in the variable name.

Example: ❌
```
public static final double CURRENT_LIMIT_FLOOR_AMPS = 1; // Floor: What we limit it to
```
Example: ✔
```
/**
* Floor: What we limit it to
* Unit: Amps
*/
public static final double CURRENT_LIMIT_FLOOR = 1; 
```

- If you need to do any unit conversions to follow these conventions, do them in code using the Units class whenever possible. Avoid converting values outside of the code project.

---
## Logging Values
- In a subsystem, values should be logged with the string beginning with **"SubsystemName/"**. This creates groups in SmartDashboard. </p>
Example: 
```
SmartDashboard.putNumber("Drivetrain/Roll", getRoll());
```

---
## Constants
- The Constants class should contain subclasses for each subsystem. The subclasses should be in alphabetical order.

Example:
```
public final class Constants {
    public static final class constControllers {
        public static final double DRIVER_LEFT_STICK_X_DEADBAND = 0.05;
  }
}
```
- All Constants should be **SCREAMING_SNAKE_CASE** 
- Variable names should avoid specifying which subsystem they belong to.

Example: ❌
```
public static final class constWrist {
    /**
    * Floor: What we limit it to
    * Unit: Amps
    */
    public static final double WRIST_CURRENT_LIMIT_FLOOR = 1; 
}
```
Example: ✔
```
public static final class constWrist {
    /**
    * Floor: What we limit it to
    * Unit: Amps
    */
    public static final double CURRENT_LIMIT_FLOOR = 1; 
}
```

---
## RobotMap
- The RobotMap class should contain subclasses for each subsystem. The subclasses should be in alphabetical order.

Example:
```
public class RobotMap {
    public static final class mapDrivetrain {
    }
}
```
- All ports should be **SCREAMING_SNAKE_CASE**. 
- Each port should specify which type of port it is in its variable name. Example types: CAN, DIO, USB 
- Variable names should avoid specifying which subsystem they belong to. 

Example: ❌
```
public static final class mapWrist {
    public static final int WRIST_MOTOR_CAN = 50;
}
```
Example: ✔
```
public static final class mapWrist {
    public static final int MOTOR_CAN = 50;
}
```

---
## RobotPreferences
- The RobotPreferences class should contain subclasses for each subsystem. The subclasses should be in alphabetical order.

Example:
```
public class RobotPreferences {
    public static final class prefWrist {
    }
}
```
- All preferences should be **lowerCamelCase**. 
- Variable names **must** specify which subsystem they belong to, as they need to be unique. 
- Variable names **must** match their SmartDashboard name. 

Example: ❌
```
public static final class mapWrist {
    public static final SN_DoublePreference ANGLE_TOLERANCE = new SN_DoublePreference("AngleTolerance", 2);
}
```
Example: ✔
```
public static final class mapWrist {
    public static final SN_DoublePreference wristAngleTolerance = new SN_DoublePreference("wristAngleTolerance", 2);
}
```

---
## RobotContainer
Controller bindings in configureBindings() should be separated by Controller. If possible, reference a diagram of the controls in the separator.

Example:
```
private void configureBindings() {
    // Driver Controller
    // Diagram: assets\driverControls23.png

    conDriver.btn_Back... (more controls here)

    // Operator Controller
    // Diagram: assets\operatorControls23.png

    conOperator.btn_Back... (more controls here)
}
```
