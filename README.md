# 135 Blocks
This is designed to be a template-based framework for FRC programming.

This is designed so that the only files that need changing to combine different subsystems is RobotContainer.java if used properly.

### Built-in Subsystems for Each Block

- Customizable Drive Base
  - CTRE Swerve
  - Rev Swerve
  - Tank
- Custom Data Handler (DataLog)
- Custom Neural Network API for Orange Pi 5
- AdvantageKit
  - Game piece simulation

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
## Blocks
## Cameras, Photon-Vision, and Limelight

This block contains:

- 4 built-in bot-pose cameras.
- Rigorous confirmation of positioning before updating the robot position for maximum accuracy.
- Built-in AprilTag Trust system: 
  - Decreases trust if a tag provides incorrect estimates.
  - Increases trust back to 100% if a tag improves over time.

### Simulation
Works entirely in simulation. Access the cameras via your browser:

- `localhost:1190/stream.mjpg`
- `localhost:1192/stream.mjpg`
- `localhost:1194/stream.mjpg`
- `localhost:1196/stream.mjpg`

Add these to Shuffleboard for ease of access!

## State Space

This block contains:

- Pre-made configurable mechanisms:
  - Flywheel
  - Arm
  - Elevator (cascade lifts, telescoping arms)

### Simulation
All mechanisms work in simulation and require SysId constants. 

For more details, refer to the [WPILib System Identification documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html).

# LED_Code

This block contains:

- Pre-made addressable LEDs with simulation support.
- Premade commands for:
  - Using images from a USB stick on the RIO.
  - Displaying patterns like rainbows, breathing effects, and constant colors.

### Commands

- `LEDGifC`: Efficiently displays any image in a specified folder on the USB stick.
- `LEDSineWaveC`: Handles various LED patterns with minimal processing overhead.
- `LEDBreathingC`: Breathes a certain color, gradually increasing and decreasing brightness.
- `LEDConstantColorC`: Displays a constant color.
- `LEDRainbowC`: Cycles through the rainbow at a given interval.

### Note

The system saves images so that if the USB is unplugged during the match, the last known state is preserved.

### Simulation

All LED patterns are supported in simulation.


## Solenoid-Code

This block contains:

- Pre-made single-acting and double-acting solenoids.
- A (questionable) function to hold pneumatic cylinders at certain positions using a bang-bang PID-like controller.

## Usage
Refer to the [PyDriverStation](https://github.com/Team135BlackKnights/PyDriverStation) repository for instructions on how to use the API for the neural network.

## Additional Resources
- For SysId constants used in State Space mechanisms, refer to the [WPILib System Identification documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html).
