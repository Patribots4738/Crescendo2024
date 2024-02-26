[![Game Manual](https://soflofrc.firstinflorida.org/wp-content/uploads/sites/23/2023/09/FIRST-IN-SHOW_CRESCENDO_FRC_SocialHQPDP_FB_Cover-1.png)](https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf)

[`src/main/java/frc/robot`](src/main/java/frc/robot) shortcut
![Robot Image](images/robot.gif)
____

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 


This repository is entirely student-created and maintained.
Attached to this repository is a poject called [Crescendo 2024](<https://github.com/orgs/Patribots4738/projects/3>) which we utilize as a Scrum framework to organize our workflow. Using Scrum, we visualize the season by dividing the given nine weeks into a week long period of time called a sprint. As a team we agree upon & decide what must be accomplished in each sprint(week). We declare each assignment by making an "issue" with an assosiated branch. We conduct research for an issue in a "spike." An estimated priority & size is asigned to each issue/assignment which is then filtered into five catogories:
  - **Backlog** -> Issues that have no status. Essientailly a large to-do list.
  - **Ready** -> Issues that are assigned to a programmer & are ready to begin.
  - **In Progress** -> Issues that are currently being worked on by a programmer.
  - **In Review** -> Issues where the assigned programmer has requested for revision by collegues.
  - **Done** -> Resolved issues with corresponding branches which have merged into our master branch called [`main`](https://github.com/Patribots4738/Crescendo2024/tree/main/src/main).
We also love [drawing boards](<https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=5486,-1359,3403,1540&page=page:9NTiPVa29oqzjElya5D6n>)! :D
    
We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, & be sure to star this repository if you found anything helpful or interesting!

### [See how we did!](https://www.statbotics.io/team/4738)


___

## Highlights
  - April Tag interpretation & note detection using two Limelights
  - Auto alignment to amp with auto shooting to speaker while driving
  - Trap placement
  - Field-centric swerve drive
  - Modular autonomous routines
  - Under bumper intake
  - LED lights that tell drive team whether or not the robot is in the right place to start autonomous and makes the robot look pretty!

### Autonomous
  #### Path Generation
  We use PathPlanner to construct a modular autonomous. In PathPlanner, we use waypoints, scheduled commands, & bezier curves to generate a singular auto path between a starting position, preferable shooting position, or note location. We then link multiple auto paths togethor to make one predetermined autonomous. Using note detection & logic, this also allows us to make a modular autonomous path that can prevent us from going to a note position that has no note detected.
  [Drawing Board](<>)

  #### LED Position Indicator (LPI)
  We have a command callled LPI which 

### Teleoperated


## **Major Class Functions ** 🤩
<img src="https://github.com/Patribots4738/Crescendo2024/assets/148731136/5d6d1ea1-1e16-48b8-b9d4-facfed37a290" width="150" height="150">

Our code is formated in a <ins> command based</ins> system on VS Code.
 
  - **Subsystems** [`robot/subsystems`](src/main/java/frc/robot/subsystems) Folder containing class file for each subsystem on the robot.
    - **Intake** [`robot/subsystems/intake`](src/main/java/frc/robot/subsystems/intake) Code for under bumper intake using
    - **Swerve** [`robot/subsystems/swerve`](src/main/java/frc/robot/subsystems/swerve) Code for swerve drivetrain with four swerve modules using 4 Neo Vortexs.
    - **Shooter** [`robot/subsystems/shooter`](src/main/java/frc/robot/subsystems/shooter) Code for shooter that uses 2 Neo Vortexs & pivot which uses 1 Neo 550 with an absolute encoder.
    - **Elevator** [`robot/subsystems/elevator`](src/main/java/frc/robot/subsystems/elevator) Code for elevator & trap placement which both use 2 NeoV1.1s
    - **Indexer** [`robot/subsystems/indexer`](src/main/java/frc/robot/subsystems/indexer) Code for indexer between intake & shooter which uses 1 Neo 550.
    - **Climb** [`robot/subsystems/climb`](src/main/java/frc/robot/subsystems/climb) Code for climb which conforms to the curve of the unoccupied chain on stage to keep the robot level with the ground. Uses 2 Neo Vortexs.
    - **LEDS** [`robot/subsystems/limelight`](src/main/java/frc/robot/subsystems/leds)  Code for WS2812B LED strip.
    - **Limelight** [`robot/subsystems/limelight`](src/main/java/frc/robot/subsystems/limelight) Code for Limelight 3.0 & 2.0+
      

   - **Commands** [`robot/commands`](src/main/java/frc/robot/commands) Fodler containing command files that control the robot.
     - **LED Position Indicator (LPI)** [`robot/commands/lpi`](src/main/java/frc/robot/commands/misc/lpi) Command created to aid the technicion in placing the robot on the field before auto. The command cycles through the autonomous starting positions listed in [`robot/commands/autonomous/PathPlannerStorage`](src/main/java/frc/robot/commands/autonomous/PathPlannerStorage) & isolates the position closest to the robots current position. It seperates the surrounding area into defined zones & changes the LEDs to reflect the zone it is currently in. Once it is in the correct position, it displays a pattern to help the technicion rotate the robot in the correct direction.
     - **Alignment Commands** [`robot/commands/subsystemHelpers/alignmentCmds`](src/main/frc/robot/commands/subsystemHelpers/alignmentCmds) File of commands that help that auto align the robot to be locked in certain axis when relative to a field object such as the stage or amp. This helps the driver with steering & alignment.
     - **Network Table PID Tuner** [`robot/commands/subsystemHelpers/NTPIDTuner`](src/main/frc/robot/commands/subsystemHelpers/NTPIDTuner) Command file that allows us to alter & tune the PID values in Advantage Scope's Network Tables for ease of access.
 
  - **Utilities**[`robot/util`](src/main/java/frc/robot/util)
    - **Constants**[`robot/util/constants`](src/main/java/frc/robot/util/constants) contains constants used throughout the robot code to prevent a mismatch in data & hardcoding values (Ex. PIDFF values & current limits).
    - **Calc**[`robot/util/calc`](src/main/java/frc/robot/util/calc) contains the calculations required for pivot alinment & shooter speeds when shooting while driving.

  ### Hardware
   - Pigeon 2.0
   - 8 Neo Vortexs, 7 Neo 550s & 2 NeoV1.1s 
   - limelight 3.0 & Limelight 2.0+
   - WS2812B LEDs
   - RoboRio 2.0
   - PDH, MPM & RPM
   - Radio OM5P-AC
   - Brain Boxes Ethernet Switch
  
  ### Software Used
   - VS Code (Java) ☕
   - PathPlanner 🛣️
   - Advantage Scope 📊
   - WPILib (the one and only ❤️)
   - Glass 👓
   - FRC Driver Station 🎮
   - REV Hardware Client 🔶
   - Pheonix Tuner X 🐦
   - Limelight Hardware Manager 🟩
   - tldraw! 📝
