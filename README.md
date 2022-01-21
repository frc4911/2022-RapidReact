# Team 4911's 2022 Codebase
Team 4911 2022 FRC robot code is written in Java and is based off WPILib's Java control system.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries.  (make sure you're using Java 11 or greater)
1. Run `./gradlew downloadAll` to download FRC tools (ShuffleBoard, etc.)
1. Run `./gradlew tasks` to see available options
1. Have fun!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `2022-RapidReact.ipr` file with IntelliJ
1. When prompted, select import Gradle build

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details.
* Run `./gradlew assemble` to build the code without running all unit tests.  Use the `--info` flag for more details.
* Run `./gradlew test` to run all the JUnit tests.

## Code Organization
We break our code up into robot code and libraries. 

* [robot code](src/main/java/frc/robot) contains code for the 2022 competition robot.
* [cheesylib](src/main/java/libraries/cheesylib) Written by Team 254, Cheesey Poofs, for 2018-2019, it includes rotation and translation support needed for swerve drive modules courtesy of Team 1323 MadTown Robotics.
* [madtownlib](src/main/java/libraries/madtownlib) Written by Team 1323, Madtown Robotics, for 2019, it includes for swerve drive modules.  This library builds on cheesylib.
* [cyberlib](src/main/java/libraries/cyberlib) Written by Team 4911, CyberKnights, it contains utility code for i/o controllers, annotations, logging, etc.  This library builds on cheesylib
