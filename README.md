[![Game Manual](https://soflofrc.firstinflorida.org/wp-content/uploads/sites/23/2023/09/FIRST-IN-SHOW_CRESCENDO_FRC_SocialHQPDP_FB_Cover-1.png)](https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf)

[`src/main/java/frc/robot`](src/main/java/frc/robot) shortcut
![Robot Image](images/robot.gif)
____

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 


This repository is entirely student-created and maintained.
We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, and be sure to star this repo if you found anything helpful!

### [See how we did!](https://www.statbotics.io/team/4738)


___

## Highlights
  - April Tag interpretation using Limelight
  - Auto alignment and shooting for notes
  - Field-centric swerve drive
  - Modular autonomous routines
  - LED lights that tell drive team whether or not the robot is in the right place to start autonomous and makes the robot look pretty!

## Major Package Functions
  - [`robot/subsystems`](src/main/java/frc/robot/subsystems)
    - Contains all subsystems used in the robot, such as [`Intake.java`](src/main/java/frc/robot/subsystems/Intake.java) which can detect if we have a game piece, and [`Climber.java`](src/main/java/frc/robot/subsystems/Climber.java) which can conform to the chain to keep the robot level.

   - [`robot/commands`](src/main/java/frc/robot/commands)
     -  Contains all the commands that control the robot.
 
  - [`robot/util`](src/main/java/frc/robot/util)

    - Utility folder that manages things like [`PatriBoxController.java`](src/main/java/frc/robot/util/PatriBoxController.java) and a special [`Neo.java`](src/main/java/frc/robot/util/Neo.java) which is capable of running in simulation as well as normally.
    
    Don't forget about [Constants](src/main/java/frc/robot/util/Constants.java)!
