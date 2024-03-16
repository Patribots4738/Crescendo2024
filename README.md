[![Game Manual](https://soflofrc.firstinflorida.org/wp-content/uploads/sites/23/2023/09/FIRST-IN-SHOW_CRESCENDO_FRC_SocialHQPDP_FB_Cover-1.png)](https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf)

[`src/main/java/frc/robot`](src/main/java/frc/robot) shortcut

![Robot Image](images/robot.gif)


<img src="https://github.com/Patribots4738/Crescendo2024/assets/148731136/1c6c852f-4c20-4b9f-b6bc-79065b717ee7" width="100%">

____

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 


This repository is entirely student-created and maintained.
Attached to this repository is a project called [Crescendo 2024](<https://github.com/orgs/Patribots4738/projects/3>) which we utilize as the Agile framework to organize our workflow. Using Agile, we map out the season by dividing it into nine week long sprints. As a team, we agree upon & decide what must be accomplished in each sprint. We declare each assignment by making issues and then implement it in an associated branch. An estimated priority & size are assigned to each issue/assignment which is then filtered into five categories:
  - **Backlog** -> Issues that have no status. Essentially a large to-do list.
  - **Ready** -> Issues that are assigned to a programmer & are ready to begin.
  - **In Progress** -> Issues that are currently being worked on by a programmer.
  - **In Review** -> Issues where the assigned programmer has requested revision by colleagues.
  - **Done** -> Resolved issues with corresponding branches which have merged into our master branch called [`main`](https://github.com/Patribots4738/Crescendo2024/tree/main/src/main).

We also love [drawing boards](<https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=5486,-1359,3403,1540&page=page:9NTiPVa29oqzjElya5D6n>)!
    
We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, & be sure to star this repository if you find anything helpful or interesting!

### [See how we did!](https://www.statbotics.io/team/4738)


___

## ✨ Highlights ✨
  - Field-centric swerve drive
  - Under bumper intake
  - April Tag interpretation & note detection using two Limelights
  - Auto alignment to amp, speaker, chains/traps, and source
  - Shooting into the speaker while driving
  - Modular autonomous routines
  - LEDs to communicate various actions
  - Fully simulated motors

## Simulation & Testing 🪄
  It is crucial to maintain both our robot & our code. Therefore, we always run our code in a simulation using `FRC Driver Station` / `WPILibs's Sim GUI` and `Advantage Scope` to catch breaking changes on our robot before they happen in real life. After our design team has constructed the CAD model of our robot in `Onshape`, we can export its moving parts into `Advantage Scope`. We then link the CAD and the code together to simulate a robot that moves similarly to a real one. Once our code passes this unit test (working in simulation), we can expect a much more specific result on the real robot. This process is used with most of our code and was essential for code involving critical math (i.e. auto alignment & shooting while driving).

## Discretize Swerve 🛞
  Our robot uses `REV MAX Swerve`. It works great, but when turning while driving, a dt of 0.02s just isn't small enough to get precise movement that doesn't drift. To remedy this, we discretize our speeds over 0.02s to get a new trajectory that results in the robot being able to spin while driving without drifting left or right. This is critical to our autonomous, since we need our position to be as accurate as possible. You can learn more about the evolution about it here: [Swerve Drive Skew and Second Order Kinematics - Chief Delphi](<https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/33>). This code is now implemented into WPILib!

## Autonomous 🤖
  <img src="https://github.com/Patribots4738/Crescendo2024/assets/148731136/d47db1e3-67e7-4c6d-ab01-7f75e807eed5" width="70%" />
  
  ### Path Generation & Modular Autonomous Paths 📈
  We use `PathPlanner` to construct a modular autonomous. In `PathPlanner`, we use named waypoints, scheduled commands, & bezier curves to generate a singular auto path between a starting position, preferable shooting position, or note location. We then link multiple auto paths together to make one predetermined autonomous. Using note detection & logic, we are able to make quick decisions on whether or not we got the note, or if we should skip trying to get a note that isn't there. Additionally, there is no predetermined starting position when starting a chain of autonomous paths. This year's abundance of April tags allow us to generate a path from anywhere in the starting zone as long as our Limelight has a clear view of a tag. Feel free to check out our [Modular Auto Drawing Board](<https://www.tldraw.com/v/mBaJ6QzdW6wNaRUvmB3DW?viewport=-121,-188,2715,1378&page=page:page>) :>

  ### Note Detection 👀
  Using `Limelight`'s machine learning algorithm accelerated by a `Google Coral`, we can detect notes from ~13 feet away. Note detection is incorporated in our modular autonomous logic, allowing us to skip over a note location if no note is detected. Those opposing robots are fast!


## Teleoperated 🎮
  <img src="https://github.com/Patribots4738/Crescendo2024/assets/148731136/73e776c2-dc3e-4e8b-9221-655e205a701a" width="40%" />
  
  ### Shooting While Driving
  Our robot is able to shoot notes into the speaker while moving! This feature grants us a shorter cycle time and is most evident in autonomous. If you are curious to learn more, check out the [Math](<https://www.tldraw.com/v/mBaJ6QzdW6wNaRUvmB3DW? viewport=-121,-188,2715,1378&page=page:page>) behind it!
  
  ### Field Centric Swerve Drive
  To make the robot more user-friendly for the driver, our swerve drive is field-centric using our `Pigeon 2.0` gyroscope to get our orientation on the field.
  
  ### Auto Alignment w/ April Tags
  Another feature for user-friendliness is our robot's ability to align to field objects such as the speaker, stage & amp. When aligned to the speaker, the driver can move the robot anywhere on the field whilst the shooter always faces the speaker. For the amp & stage, the robot becomes locked in a certain axis. This helps the driver with steering & alignment.

_____

## Major Class Functions 🤩
<img src="https://github.com/Patribots4738/Crescendo2024/assets/148731136/5d6d1ea1-1e16-48b8-b9d4-facfed37a290" width="150" height="150">

Our code is formatted in a <ins>command-based</ins> system on VS Code using <ins>Java<ins/>.
 
  - **Subsystems** [`robot/subsystems`](src/main/java/frc/robot/subsystems) Folder containing class file for each subsystem on the robot.
    - **Intake** [`robot/subsystems/intake`](src/main/java/frc/robot/subsystems/intake) An under-the-bumper intake run by a `Neo 550`.
    - **Swerve** [`robot/subsystems/swerve`](src/main/java/frc/robot/subsystems/swerve) Drivetrain with four swerve modules using 4 `Neo Vortexs` and 4 `Neo 550`s.
    - **Shooter** [`robot/subsystems/shooter`](src/main/java/frc/robot/subsystems/shooter) A shooter that uses 2 Neo Vortexs & pivot which uses 1 `Neo 550` with an absolute encoder.
    - **Elevator** [`robot/subsystems/elevator`](src/main/java/frc/robot/subsystems/elevator) Elevator for amp & trap placement which uses 1 `Neo v1.1`.
    - **Indexer** [`robot/subsystems/indexer`](src/main/java/frc/robot/subsystems/indexer) The Indexer between intake & shooter which uses a `Neo 550`.
    - **Climb** [`robot/subsystems/climb`](src/main/java/frc/robot/subsystems/climb) Two independently driven climbs that conform to the curve of the unoccupied chain on stage to keep the robot level with the ground. Uses one `Neo Vortex` each.
    - **LEDS** [`robot/subsystems/limelight`](src/main/java/frc/robot/subsystems/leds) Our `WS2812B` LED strip.
    - **Limelight** [`robot/subsystems/limelight`](src/main/java/frc/robot/subsystems/limelight) Interaction between the `Limelight` (2 and 3) and the robot.
      

   - **Commands** [`robot/commands`](src/main/java/frc/robot/commands) Fodler containing command files that control the robot.
     - **LED Position Indicator (LPI)** [`robot/commands/lpi`](src/main/java/frc/robot/commands/misc/lpi) Command created to aid the technician in placing the robot on the field before auto. The command cycles through the autonomous starting positions listed in [`robot/commands/autonomous/PathPlannerStorage`](src/main/java/frc/robot/commands/autonomous/PathPlannerStorage) & isolates the position closest to the robot's current position. It separates the surrounding area into defined zones & changes the LEDs to reflect the zone it is currently in. Once it is in the correct position, it displays a pattern to help the technician rotate the robot in the correct direction.
     - **Alignment Commands** [`robot/commands/subsystemHelpers/alignmentCmds`](src/main/frc/robot/commands/subsystemHelpers/alignmentCmds) File of commands that help that auto-align the robot to be locked in a certain axis when relative to a field object such as the stage or amp. This helps the driver with steering & alignment.
     - **Network Table PID Tuner** [`robot/commands/subsystemHelpers/NTPIDTuner`](src/main/frc/robot/commands/subsystemHelpers/NTPIDTuner) Command file that allows us to alter & tune the PID values in Advantage Scope's Network Tables for ease of access.\n\n
 
  - **Utilities** [`robot/util`](src/main/java/frc/robot/util)
    - **Constants** [`robot/util/constants`](src/main/java/frc/robot/util/constants) contains constants used throughout the robot code to prevent a mismatch in data & hardcoding values (Ex. PIDFF values & current limits).
    - **Calc** [`robot/util/calc`](src/main/java/frc/robot/util/calc) contains the calculations required for pivot alinment & shooter speeds when shooting while driving.
   
## Controls 🎮
[![Driver](https://github.com/Patribots4738/Crescendo2024/assets/65139378/d2d0000f-54cb-42d2-9b8a-7e635ca79c18)](https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=2339,251,5877,2813&page=page:page)

[![Operator](https://github.com/Patribots4738/Crescendo2024/assets/65139378/5040695f-deac-4ac9-9ad3-ca7cbfa4748b)](https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=2339,251,5877,2813&page=page:page)

## Components & Tools 🛠️
[![Hardware](https://github.com/Patribots4738/Crescendo2024/assets/65139378/f9de2a2e-e401-4d6b-b57a-10bbf2dfd340)](https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=-4823,-6599,9853,4716&page=page:g60UEEXm6O2yBIoLYfVVB)
  
[![Software](https://github.com/Patribots4738/Crescendo2024/assets/148731136/058e53cd-83ff-4463-ba4f-3b58a56a3ead)](https://www.tldraw.com/r/EolJKYU3QEqxw71uyAqPS?viewport=-4823,-6599,9853,4716&page=page:g60UEEXm6O2yBIoLYfVVB)
