package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.IOError;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.BindException;
import java.net.ServerSocket;
import java.net.Socket;

import java.lang.reflect.Type;

/**
 * Class used for logging data. Coded as an alternative to DataLogManager (so
 * you don't get every update in NetworkTables logged, just the values you
 * want), designed to be used for polynomial regression. Saves data in columns
 * instead of rows (each value for x is represented by the values below it) Used
 * in conjunction with the 135 PyDriverStation to generate accurate models.
 */
public class DataHandler {
	private static FileOutputStream outputStream;
	private static OutputStreamWriter outputStreamWriter;
	private static int id = 0;
	private static List<String> dataBuffer = new ArrayList<String>();
	private static String diskName = "/U";
	private static String directoryName = "";
	private static File createdFile;
	private static File directory;
	public static boolean isUSBConnected = true;
	public static boolean fileCreated = false;
	private static int debounce = 0;
	private static int dumpID = 1;
	private static Gson gson;
	static Map<String, String> responseData = new HashMap<>();
	private static int port = 5802;
	private static ServerSocket serverSocket;
	private static final boolean usingLaptop = true;

	/**
	 * Creates a new Streamwriter, designed to be contingent in case of USB
	 * disconnection and reconnection
	 */
	public static void createNewWriter() {
		try {
			outputStream = new FileOutputStream(createdFile);
		}
		catch (FileNotFoundException e) {
			//e.printStackTrace();
		}
		outputStreamWriter = new OutputStreamWriter(outputStream);
	}

	/**
	 * This function allows you to customize the directory that you send a log to
	 * in simulation. Designed to be used for simulations or other applications
	 * where the directory to be sent to is different from the default roboRIO
	 * USB Directory. All other functions in this class should work with the log
	 * and reader generated by this function.
	 * 
	 * @param directory The folder where the simulated log will be written. Will
	 *                     need to be changed based on the device you're running
	 *                     simulation from and the directory where the logs are
	 *                     recorded.
	 * @see DataHandler
	 */
	public static void setSimDisk(String directory) {
		DataHandler.diskName = directory;
	}

	/**
	 * Call this in Robot.java. Starts the handler and has contingencies to use
	 * the NetworkTables, write to usb, or write to a sim disk drive
	 * 
	 * @param useNetworkTables A boolean that states whether to write to a
	 *                            physical USB or use the networktables
	 * @param simDiskName      A string that states what disk to write to in
	 *                            simulation
	 */
	public static void startHandler(String simDiskDirectory) {
		gson = new Gson();
		try {
			while (serverSocket == null) {
				try {
					serverSocket = new ServerSocket(port);
				}
				catch (BindException e) {
					System.err.println("Port " + port
							+ " is already in use. Trying next port...");
					port++; // Try the next port
				}
			}
			System.err.println(port + "GOOD!");
			if (Constants.currentMode == Constants.Mode.SIM) {
				PortForwarder.add(port, "localhost", port);
			} else {
				PortForwarder.add(port, "10.1.35.2", port);
			}
			serverSocket.setSoTimeout(1);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		if (Constants.currentMode == Constants.Mode.REAL) {
			createLogFileOnRIOUSB();
		} else {
			setSimDisk(simDiskDirectory);
			createLogFileOnRIOUSB();
		}
	}

	/**
	 * Creates a file on a folder called "logs" (located on the USB attached to
	 * the rio) to log all values to. If there is no folder called "logs", it
	 * creates a folder. Only call one time (either in RobotInit or a subsystem
	 * constructor). Using this function in simulation (instead of
	 * createLogFileInSimulation) will lead to an IOException of "the system
	 * cannot find the path specified". Use createLogFileInSimulation with a
	 * specific directory given to run this in simulation If no USB drive is
	 * present, will just throw an error and not create a file. Only include the
	 * drive and the colon (C:), NOT C://
	 */
	public static void createLogFileOnRIOUSB() {
		// "/U" is the default directory (drive name) for RoboRIO flash drives, create a new folder called "/U/Logs" if one doesn't exist
		//function based off of https://github.com/HoyaRobotics/InfiniteRecharge2020/blob/master/src/main/java/frc/robot/util/Logger.java
		//Creates the logs file in the specified drive if none is there
		directoryName = diskName + "/Logs";
		directory = new File(directoryName);
		//This code isn't TECHNICALLY needed to make the directory (it auto checks if a folder is there), but i would like to make sure this is here just in case something happens
		if (!directory.exists()) {
			directory.mkdir();
		}
		try {
			//Creates new file in the /U/Logs folder 
			String fileName = directoryName + "/Latest.txt";
			createdFile = new File(fileName);
			//if a file named "latest" exists, rename "latest" to the id in its first line
			if (createdFile.exists()) {
				Scanner renameScanner = new Scanner(createdFile);
				if (renameScanner.hasNext()) {
					try {
						id = Integer.parseInt(renameScanner.nextLine());
					}
					catch (Exception e) {
						System.out.println("ID did not exist, using default" + id);
					}
					renameScanner.close();
				}
				//Create the new file object
				File newFileName = new File(directoryName + "/Log" + id + ".txt");
				createdFile.renameTo(newFileName);
				fileCreated = true;
			}
			//Creates an actual file in the directory
			System.out.println("Path" + createdFile.getAbsolutePath());
			createdFile.createNewFile();
			//Adds 1 to the id then writes it to the first line, this is used to ensure no two logs have the same file number.
			id += 1;
			//Makes the streamwriters for the written log file
			createNewWriter();
			outputStreamWriter.write(id + "\r\n");
			outputStreamWriter.flush();
		}
		//catch any errors
		catch (Exception e) {
			//e.printStackTrace();
		}
		catch (IOError e) {
			//e.printStackTrace();
		}
	}

	/**
	 * Writes values to file and sends them through networkTables Recommended to
	 * start by logging the table heading names first, polynomial regression tool
	 * handles this. If there is a time when you want to log data but want to
	 * ignore something, put null in as the value in the array (will output a
	 * string "null"). Regression calculator currently cannot handle this
	 * exception, as well as data relationships that have more than 2 variables
	 * (y = f(x) type functions) Writes everything as a string, please convert
	 * values to strings before adding them to the array.
	 * 
	 * @param tableHeadings the array of values to be logged, can be different
	 *                         from the values declared in the setUpLogOnUsb
	 */
	public static void logData(String data, String key) {
		responseData.put(key, data); //send to network tables
		logData(data); //also log the data.
	}

	public static void logData(String data) {
		//Tries writing to the file, adds an error if it doesn't work
		try {
			outputStreamWriter.write(data + "\r\n");
			outputStreamWriter.flush();
		}
		//Catches errors
		catch (Exception e) {
			dataBuffer.add(data);
		}
		catch (IOError e) {
		}
	}

	public static void logData(String[] data, String key) {
		//String that will be output to the writer
		String lineToBeSaved = "";
		//Adds each argument in the array to the string, adds a comma for separation (regression calculator uses this as well)
		for (String heading : data) {
			lineToBeSaved += (heading + ",");
		}
		//Removes last comma at the end
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(int data, String key) {
		String dataString = Integer.toString(data);
		logData(dataString, key);
	}

	public static void logData(int[] data, String key) {
		String lineToBeSaved = "";
		for (int integer : data) {
			lineToBeSaved += (Integer.toString(integer) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(boolean data, String key) {
		String dataString = Boolean.toString(data);
		logData(dataString, key);
	}

	public static void logData(boolean[] data, String key) {
		String lineToBeSaved = "";
		for (boolean bool : data) {
			lineToBeSaved += (Boolean.toString(bool) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(double data, String key) {
		String dataString = Double.toString(data);
		logData(dataString, key);
	}

	public static void logData(double[] data, String key) {
		String lineToBeSaved = "";
		for (double num : data) {
			lineToBeSaved += (Double.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(float data, String key) {
		String dataString = Float.toString(data);
		logData(dataString, key);
	}

	public static void logData(float[] data, String key) {
		String lineToBeSaved = "";
		for (float num : data) {
			lineToBeSaved += (Float.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	//here
	public static void logData(String[] data) {
		//String that will be output to the writer
		String lineToBeSaved = "";
		//Adds each argument in the array to the string, adds a comma for separation (regression calculator uses this as well)
		for (String heading : data) {
			lineToBeSaved += (heading + ",");
		}
		//Removes last comma at the end
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved);
	}

	public static void logData(int data) {
		String dataString = Integer.toString(data);
		logData(dataString);
	}

	public static void logData(int[] data) {
		String lineToBeSaved = "";
		for (int integer : data) {
			lineToBeSaved += (Integer.toString(integer) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved);
	}

	public static void logData(boolean data) {
		String dataString = Boolean.toString(data);
		logData(dataString);
	}

	public static void logData(boolean[] data) {
		String lineToBeSaved = "";
		for (boolean bool : data) {
			lineToBeSaved += (Boolean.toString(bool) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved);
	}

	public static void logData(double data) {
		String dataString = Double.toString(data);
		logData(dataString);
	}

	public static void logData(double[] data) {
		String lineToBeSaved = "";
		for (double num : data) {
			lineToBeSaved += (Double.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved);
	}

	public static void logData(float data) {
		String dataString = Float.toString(data);
		logData(dataString);
	}

	public static void logData(float[] data) {
		String lineToBeSaved = "";
		for (float num : data) {
			lineToBeSaved += (Float.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved);
	}

	//Basically writes the entire buffer in the event the USB was disconnected. Creates a dump file to store any data lost when it disconnects 
	public static void flushBuffer() {
		File dumpFile = new File(
				directoryName + "/Log" + id + "DisconnectDump" + dumpID + ".txt");
		try {
			dumpFile.createNewFile();
			FileOutputStream outputStream = new FileOutputStream(dumpFile);
			OutputStreamWriter dumpFileOutputStreamWriter = new OutputStreamWriter(
					outputStream);
			if (dataBuffer.size() > 0) {
				for (String data : dataBuffer) {
					System.out.println(data);
					dumpFileOutputStreamWriter.write(data + "\r\n");
					dumpFileOutputStreamWriter.flush();
				}
				dataBuffer.clear();
			}
			dumpFileOutputStreamWriter.close();
		}
		catch (Exception e) {
			//e.printStackTrace();
		}
	}

	/**
	 * Flushes the writer (outputs last value) and then closes it. Does not need
	 * to be called.
	 */
	public static void closeWriter() {
		try {
			outputStreamWriter.flush();
			outputStreamWriter.close();
		}
		//Catches errors
		catch (Exception e) {
			//e.printStackTrace();
		}
		catch (IOError e) {
			//e.printStackTrace();
		}
	}

	/**
	 * Checks the USB connection status of the RIO by making sure the directory
	 * still exists
	 */
	public static void pingUSB() { isUSBConnected = directory.exists(); }

	/**
	 * Updates the state of the handler, and checks if the USB has been
	 * disconnected. Call this in the periodic function of the file you called
	 * createNewWriter in.
	 */
	static double previousTime = 0;
	static String oldModel = "";
	static String[] outputList;

	/**
	 * Get any value from the output model from Python.
	 * 
	 * @param index ZERO AS FIRST OUTPUT
	 * @return IN A DOUBLE your selected index for the model outputs.
	 */
	public static double getValue(int index) {
		return Double.parseDouble(outputList[index]);
	}

	private static List<Double> makeDoubleList(String data) {
		String formattedData = data.replace("[", "").replace("]", "").trim();
		String[] outputs = formattedData.split("\\s+");
		List<Double> outputList = new ArrayList<>();
		for (String numberString : outputs) {
			outputList.add(Double.parseDouble(numberString));
		}
		return outputList;
	}

	/**
	 * Updates state of the handler, and continually sends any data via network
	 * tables. Whenever we have a change in NetworkTables, log that as well.q
	 */
	private static int oldTime = 0;

	public static void updateHandlerState() {
		String dataHandlerJson = SmartDashboard.getString("ToRobot", "default");
		if (!dataHandlerJson.equals("default") && usingLaptop) {
			try {
				// Parse JSON string
				Type mapType = new TypeToken<Map<String, String>>() {}.getType();
				Map<String, String> dataFromPython = gson.fromJson(dataHandlerJson,
						mapType);
				if (dataFromPython.containsKey("modelUpdated")) {
					if (responseData.containsKey("shouldUpdateModel")) {
						responseData.remove("shouldUpdateModel"); //stop asking for data
					}
				}
				// Prepare response data
				responseData.put("status", "running");
				// Convert response data to JSON
				String jsonResponse = gson.toJson(responseData);
				// Send response JSON to Python
				SmartDashboard.putString("FromRobot", jsonResponse);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		} //Laptop not being used.
		try (Socket socket = serverSocket.accept();
				PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
				BufferedReader in = new BufferedReader(
						new InputStreamReader(socket.getInputStream()))) {
			// Receive JSON string from Orange Pi
			StringBuilder jsonStringBuilder = new StringBuilder();
			int character;
			while ((character = in.read()) != -1) {
				char c = (char) character;
				if (c == '\n') {
					break; // Reached delimiter, stop reading
				}
				jsonStringBuilder.append(c);
			}
			String jsonString = jsonStringBuilder.toString();
			//System.out.println("Received: " + jsonString);
			// Parse JSON string
			JsonObject receivedData = JsonParser.parseString(jsonString)
					.getAsJsonObject();
			// Process received data (optional)
			if (receivedData.has("timestamp")) {
				int time = receivedData.get("timestamp").getAsInt();
				if (time != oldTime + 1) {
					SmartDashboard.putString("PiConnection",
							"SKIPPED VIA SOCKET" + oldTime);
				}
				oldTime = time;
				//System.out.println("time: " + time);
			}
			if (receivedData.has("outputs")) {
				if (responseData.containsKey("modelInputs")) {
					responseData.remove("modelInputs");
				}
				String rawData = receivedData.get("outputs").getAsString();
				List<Double> list = makeDoubleList(rawData);
				System.out.println(list);
				//Remove brackets
			}
			responseData.put("status", "running");
			// Prepare response JSON
			String jsonResponse = gson.toJson(responseData);
			// Convert JSON to string and send as response
			out.println(jsonResponse);
			SmartDashboard.putString("PiConnection", "OK");
		}
		catch (Exception e) {
			SmartDashboard.putString("PiConnection", "PI NOT DETECTED");
		}
		//We are not using network tables
		pingUSB();
		flushBuffer();
		//If the USB is disconnected and it hasn't closed the writer yet, close it
		if (isUSBConnected == false && debounce == 0) {
			closeWriter();
			debounce = -1;
		}
		//If the USB is reconnected and the writer is closed, open a new one
		else if (isUSBConnected == true && debounce == -1) {
			createLogFileOnRIOUSB();
			debounce = 0;
		}
		//If neither condition is met, do nothing.
		else {
			return;
		}
	}
}
