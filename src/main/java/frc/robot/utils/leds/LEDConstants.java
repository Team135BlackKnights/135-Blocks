package frc.robot.utils.leds;
import java.util.List;

import frc.robot.subsystems.leds.LEDs;

public class LEDConstants {
	public static int
	//The physical port where the LED strip is plugged in 
	ledPort = 9,
	// amount of LEDs in the light strip
	ledBufferLength = 4096,
	totalGifs = 2,
	ledCols = 64,
	ledRows = 64;
	//HSV Colors with values stored in arrays
	public static int[] noteHSV = new int[] { 12, 255, 100
	}, redHSV = new int[] { 0, 255, 100
	}, blueHSV = new int[] { 120, 255, 100
	}, greenHSV = new int[] { 50, 255, 100
	}, pinkHSV = new int[] { 147, 255, 255
	}, goldHSV = new int[] { 23, 255, 100
	}, disabledHSV = new int[] { 0, 0, 0
	};
	public static List<String> imageList = List.of("gif1","gif2","debug");
	public static List<List<byte[][]> > imageLedStates = LEDs.preprocessImages(imageList);
}
