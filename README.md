![blocks_logo](https://github.com/Team135BlackKnights/135-Blocks/assets/49589065/488cddd6-688f-4a2d-b0a6-cc395535b318)

# 135 Blocks
This is designed to be a template-based framework for FRC programming.

This is designed so that the only files that need changing to combine different subsystems is RobotContainer.java if used properly.

### Built-in Subsystems for Each Block
- Drivetrain support
  - Swerve
  - Mechanum
  - Tank
- Multi-Vendor support
  - CTRE Motors
  - REV Spark Maxes
  - REV Spark Flexes
- Custom API for interfacing with Custom AI on Orange Pi 5
  - Allows any number of inputs
  - Allows any number of outputs
  - Prevents Overfitting
  - Displays all useful statistics for human-verification
  - Essentially ZERO latency (.0003 seconds)
  - Zero knowledge of AI required
- Motor Constant Container
  - Prevent mistakes/crash-causing errors from SysID via checks
  - Put all SysID in ONE place
- FULL Sim Support
- Dynamic PID Tuning 
  - Loggable Tuning Numbers allow updates
- Drive to Pose Command
  - Slow/Fast mode, where each has custom max accel/velocity
  - Go to a given Pose2d
### Directory Structure
Preferably, do NOT touch any of the `drive/*` contents, as that could cause a merge error!
##### However, some files do need to change interactions with the base, so it is understandable in some situations.

An example branch directory could be:


    src/main/java/frc/robot/
        commands/
		drive/
			...
        	exampleBranchName/
			exampleCommand.java
        subsystems/
		drive/
			...
		exampleBranchName/
			exampleSubsystem.java
        utils/
		drive/
			...
		exampleBranchName/
			Constants.java
        Constants.java
	DataHandler.java
	Main.java
	Robot.java
	RobotContainer.java

 
## Usage

Refer to the [PyDriverStation](https://github.com/Team135BlackKnights/PyDriverStation) repository for instructions on how to use the API for the neural network.
 

## Blocks

## CTRE State Space

This block contains:

- Pre-made configurable mechanisms:
  - Double Jointed Arm
  - Elevator (cascade lifts, telescoping arms)
  - Flywheel
  - Single Jointed Arm

    
### Commands
- `CTREDoubleJointedArmC`: Update arm macro position setpoints, given a wanted elbow point direction, and grab voltages from the best possible path to that position using the OrangePi. Hold at end setpoint.
- `CTREElevatorC`: Move the elevator to a desired macro position, or move it via a joystick, and hold there.
- `CTREFlywheelC`: Accelerate the wheels to a desired RPM, or using a joystick, *returning to zero when let go*.
- `CTRESingleJointedArmC`: Move the arm to a desired macro position, or move it via a joystick, and hold there.
  
### Simulation
All mechanisms work in simulation and **require** SysId constants. For the Double Jointed Arm, the Orange Pi Server MUST be running. (Sim version supported) 

### Notes
For more details on System Identification, refer to the [WPILib System Identification documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html).
For more details on State Space, refer to the [WPILib State Space Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html).
For more details on Orange Pi Server, refer to the [OrangePi code in PyDriverStation](https://github.com/Team135BlackKnights/PyDriverStation/tree/main/OrangePi).

## Cameras, Photon-Vision, and Limelight

This block contains:

- 4 built-in bot-pose cameras.
- Rigorous confirmation of positioning before updating the robot position for maximum accuracy.
- Built-in AprilTag Trust system: 
  - Decreases trust if a tag provides incorrect estimates.
  - Increases trust back to 100% if a tag improves over time.
- Accurate Limelight Sim for tX, tY, and tV.
  - Using a limelight in Sim is 100% accurate, for better or for worse.
  - Can be used to tune Vision PIDs, as it uses the SAME tX and tY calculations.

### Commands
- `DriveToAITarget`: Use pipeline zero on the limelight, and aim towards it and accelerate towards it until a desired distance is achieved. Maps speeds so that it doesn't stop suddenly at the end.
  
### Simulation
Works entirely in simulation. Access all PhotonVision cameras via your browser. Limelight is supported in simulation mode, however it cannot be visualized via a wireframe.

- `localhost:1190/stream.mjpg`
- `localhost:1192/stream.mjpg`
- `localhost:1194/stream.mjpg`
- `localhost:1196/stream.mjpg`
  
Add these to Shuffleboard for ease of access!

## LED_Code

This block contains:

- Pre-made addressable LEDs with simulation support.
- Premade commands for:
  - Using videos from a USB stick on the RIO.
  - Displaying patterns like rainbows, breathing effects, and constant colors.

### Commands
- `LEDGifC`: Efficiently displays any image in a specified folder on the USB stick, at a given interval. Loops after the last photo.
- `LEDSineWaveC`: Displays a sin wave that moves through all the LEDS, at a given interval and color.
- `LEDBreathingC`: Breathes a certain color, gradually increasing and decreasing brightness, at a given interval.
- `LEDConstantColorC`: Displays a constant color.
- `LEDRainbowC`: Cycles through the rainbow at a given interval.

#### Notes
The system saves images on boot, so that if the USB is unplugged during the match, no problems occur.
In order to use the LEDGifC command, you must first split up the GIF into its component frame images, upload those component images into a folder

### Simulation
All LED patterns are supported in simulation.


## Servos

This block contains:

- Custom servo sim that allows you to set the bounds, and simulate a REVSmartServo’s continuous mode
- Custom enum that dictates servo type as well as which mode the servo is being run in
- Custom servoPackage util that allows for seamless transitions between real robot and simulation
#### Notes
To add more servos, create a new servoPackage object and add it to the array of existing ones located in servoC. In addition to that, create a new enumeration in the ServoNames enum, and update the case statements to include the new servoPackage. 

### Commands
- `ServoC`: Sets the servos via setServoDegrees. Automatically handles Sim.
### Simulation
All Servos are supported in simulation.

## Solenoid-Code

This block contains:

- Pre-made single-acting and double-acting solenoids.
- A (questionable) function to hold pneumatic cylinders at certain positions using a bang-bang PID-like controller.

## State Space

This block contains:

- Pre-made configurable mechanisms:
  - Single Jointed Arm
  - Elevator (cascade lifts, telescoping arms)
  - Flywheel

    
### Commands
- `ArmC`: Move the arm to a desired macro position, or move it via a joystick, and hold there.
- `ElevatorC`: Move the elevator to a desired macro position, or move it via a joystick, and hold there.
- `FlywheelC`: Accelerate the wheels to a desired RPM, or using a joystick, *returning to zero when let go*.
  
### Simulation
All mechanisms work in simulation and **require** that SysId constants be identified. 

### Notes
For more details on System Identification, refer to the [WPILib System Identification documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html).
For more details on State Space, refer to the [WPILib State Space Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html).

## Wrappers
 
 All blocks need two wrappers to be used in order to function:
 
  - `MotorConstantContainer`:  A wrapper that holds characterization values (ks, kv, ka, kp, kd) of a particular motor. Throws an error if an incorrect value is input. Used for state-space models and drivetrain simulation.
  - Individual drivetrain constant containers: Constant containers that hold values necessary to create a drivetrain to know which constants are needed (Example: `REVModuleConstantContainer`)

## MatchState 

 This value lets you see what part of the FRC Match the robot is in, including endgame.

## Getting Started
 In order to get started with this system, first make a fork of this repository. After that is done, merge the branches with the features that you need into the main branch, and tweak constants based on your robot. If you want to use simulation, make sure you are familiar with AdvantageKit, AdvantageScope and URCL as this repository relies on all three to function. 

## Workflow
 This repository is designed so that multiple programmers can merge their work from separate branches without any issues. In order to accomplish this, create a folder with the name of each branch under all of the `utils` folders.  If this is done correctly, the only merge issues that  
 need to be resolved should be in Robot.java and RobotConstantContainer.java.
