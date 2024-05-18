package frc.robot.commands.leds;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.utils.leds.LEDConstants;

public class LEDGifC extends Command {
    // Declaring Variables
    private LEDs ledS;
    private boolean isFinished;
    private final int delayMs;
    private long lastUpdateTime;
    private int currentImageIndex = 0;
    /**
     * Sets the LEDs to display images and swaps images at each delay interval.
     */
    public LEDGifC(LEDs ledSubsystem, List<String> imagePaths, int delayMs) {
        this.ledS = ledSubsystem;
        this.delayMs = delayMs;
        addRequirements(ledS);
    }

    

    @Override
    public void initialize() {
        isFinished = false;
        currentImageIndex = 0;
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime >= delayMs) {
            // Update LEDs with the current image
            byte[][] currentLedStates = LEDConstants.imageLedStates.get(currentImageIndex);
            for (var i = 0; i < LEDs.ledBuffer.getLength(); i++) {
					 LEDs.ledBuffer.setRGB(i, Byte.toUnsignedInt(currentLedStates[i][0]), Byte.toUnsignedInt(currentLedStates[i][1]), Byte.toUnsignedInt(currentLedStates[i][2]));
            }

            // Swap to the next image
            currentImageIndex = (currentImageIndex + 1) % LEDConstants.imageLedStates.size();

            // Set data to buffer
            LEDs.leds.setData(LEDs.ledBuffer);

            lastUpdateTime = currentTime;
        }
    }

    @Override
    public void end(boolean interrupted) {
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}