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

## Highlights ✨✨✨
  - April Tag interpretation & note detection using two Limelights
  - Auto alignment to amp with auto shooting to speaker while driving
  - Trap placement
  - Field-centric swerve drive
  - Modular autonomous routines
  - Under bumper intake
  - LED lights that tell drive team whether or not the robot is in the right place to start autonomous and makes the robot look pretty!
Continue reading to learn more! :D

## Autonomous 🤖
  ### LED Position Indicator (LPI)
  We have a command callled LPI that runs when the robot is turned on & disabled. This command was created to aid the technicion in positioning the robot in the starting zone by displaying various LED patterns to convey the distance from the desired starting position & the percentage of error of the robots rotation. This allows us to oriente the robot with ease to eliminate the need of aligning to a field object such as the subwoofer, therefore giving us the liberty to start the game anywhere in the starting zone. Please view the section regarding major class functions below if you like to learn more.
  [LPI Concept Drawing Board](<https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=-3929,-1794,7449,3781&page=page:9NTiPVa29oqzjElya5D6n](https://www.tldraw.com/v/YKJloESPqAyu62wxqEQ8U?viewport=-3929,-1794,7449,3781&page=page:9NTiPVa29oqzjElya5D6n>)
  
  ### Path Generation & Modular Autonomous Paths
  We use PathPlanner to construct a modular autonomous. In PathPlanner, we use waypoints, scheduled commands, & bezier curves to generate a singular auto path between a starting position, preferable shooting position, or note location. We then link multiple auto paths togethor to make one predetermined autonomous. Using note detection & logic, this also allows us to make a modular autonomous path that can prevent us from going to a note position that has no note detected. This prevents the robot from visiting a location.
  [Modular Auto Drawing Board](<https://www.tldraw.com/v/mBaJ6QzdW6wNaRUvmB3DW?viewport=-121,-188,2715,1378&page=page:page>)

  ### Note Detection
  Using Limelight & machine learning, we can detect notes from 13 feet away. Note detection is incorperated in our modular autonomous logic, allowing us to skip-over a note location if no note is detected.


## Teleoperated 🎮
  ### Shooting While Driving
  Our robot is able to shoot notes into the speaker while moving sideways when relative to it. This feature grants us a shorter cycle time. Unfortunetly, this feature is inconsistent when moving forwards or backwards in relation to the speaker.
  [Math :D](<https://www.tldraw.com/v/mBaJ6QzdW6wNaRUvmB3DW?viewport=-121,-188,2715,1378&page=page:page>)
  
  ### Field Centric Swerve Drive
  To make the robot more user friendly for the driver, our swerve drive is field centric. When the robot is field relative in a game, the 
  
  ### Auto Alignment w/ April Tags
  Another feature for user friendliness is our robots ability to align to field objects such as the speaker, stage & amp. When aligned to the speaker, the driver can move the robot anywhere on the field whilst the shooter always faces the speaker. For the amp & stage, the robot becomes locked in a certain axis. This helps the driver with steering & alignment.


## Major Class Functions 🤩
<img src="https://github.com/Patribots4738/Crescendo2024/assets/148731136/5d6d1ea1-1e16-48b8-b9d4-facfed37a290" width="150" height="150">

Our code is formated in a <ins> command based</ins> system on VS Code using <ins>Java<ins/>.
 
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
   
## Controls 🎛️🎮
![image](https://github.com/Patribots4738/Crescendo2024/assets/148731136/461b6ea1-2418-44b1-bdd4-82bec2677c85)

![image](https://github.com/Patribots4738/Crescendo2024/assets/148731136/c11e9e46-8121-4bff-97c3-2bb4c24f7ac7)

## Components & Tools 🔨🔎🚨 
![image](https://github.com/Patribots4738/Crescendo2024/assets/148731136/9ab7df35-7143-441b-a1f6-f31ea8a77bd8)
  
![image](https://github.com/Patribots4738/Crescendo2024/assets/148731136/058e53cd-83ff-4463-ba4f-3b58a56a3ead)
