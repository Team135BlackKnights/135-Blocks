package frc.robot.commands.drive;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

/*
 * Use this as explanation on how to pull DataLogs:
 * https://docs.wpilib.org/en/stable/ docs/software/telemetry/datalog-
 * download.html Should theoretically output a csv file, see if we can convert
 * it into a txt file and upload it to smth
 */
public class SwerveC extends Command {
	public ChassisSpeeds chassisSpeeds;
	private final DrivetrainS drivetrainS;
	private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
	public static double[][] variableAngleLog = new double[2][20];
	public static double variableAngleDistance = 0;
	public static double angleOutputDegrees = 0;
	//private int arrayIndex = 0;
	

	public SwerveC(DrivetrainS drivetrainS) {
		this.drivetrainS = drivetrainS;
		// These guys limit acceleration, they aren't the most necessary but it makes movement smoother
		this.xLimiter = new SlewRateLimiter(
				DriveConstants.kTeleDriveMaxAcceleration);
		this.yLimiter = new SlewRateLimiter(
				DriveConstants.kTeleDriveMaxAcceleration);
		this.turningLimiter = new SlewRateLimiter(
				DriveConstants.kTeleTurningMaxAcceleration);
		addRequirements(drivetrainS);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		switch (DriveConstants.vendor) {
		case REV_MECANUM:
		//TODO: add mecanum here
		break;
		case CTRE_SWERVE:
			drivetrainS.applyRequest();
			break;
		case REV_SWERVE:
		
		case TANK:
			// Get desired ChassisSpeeds from controller
			double xSpeed = -RobotContainer.driveController.getLeftY();
			double ySpeed = -RobotContainer.driveController.getLeftX();
			double turningSpeed = -RobotContainer.driveController.getRightX();
			xSpeed = Math.pow(xSpeed, 2) * (xSpeed < 0 ? -1 : 1);
			ySpeed = Math.pow(ySpeed, 2) * (ySpeed < 0 ? -1 : 1);
			/*if (SwerveS.autoLock == true && CameraS.aprilTagVisible() == true) {
				turningSpeed = swerveS.autoLockController
						.calculate(CameraS.getXError(), 0.0);
			}*/
			// If the desired ChassisSpeeds are really small (ie from controller drift) make
			// them even smaller so that the robot doesn't move
			xSpeed = Math.abs(xSpeed) > DriveConstants.SwerveConstants.kDeadband
					? xSpeed
					: 0.0000;
			ySpeed = Math.abs(ySpeed) > DriveConstants.SwerveConstants.kDeadband
					? ySpeed
					: 0.0000;
			/*if (SwerveS.autoLock == true && CameraS.aprilTagVisible() == true) {
				turningSpeed = Math
						.abs(turningSpeed) > DriveConstants.SwerveConstants.kAutoDeadband
								? turningSpeed
								: 0.0000;
			} else {*/
			turningSpeed = Math
					.abs(turningSpeed) > DriveConstants.SwerveConstants.kDeadband
							? turningSpeed
							: 0.0000;
			//}
			// Limit the acceleration and convert -1 to 1 from the controller into actual speeds
			xSpeed = xLimiter.calculate(xSpeed)
					* DriveConstants.kMaxSpeedMetersPerSecond;
			ySpeed = yLimiter.calculate(ySpeed)
					* DriveConstants.kMaxSpeedMetersPerSecond;
			if (DriveConstants.vendor == DriveConstants.driveTrainType.TANK) {
				turningSpeed = turningLimiter.calculate(turningSpeed)
						* DriveConstants.kMaxSpeedMetersPerSecond;
			} else {
				turningSpeed = turningLimiter.calculate(turningSpeed)
						* DriveConstants.kMaxTurningSpeedRadPerSec;
			}
			if (Robot.isRed) {
				xSpeed *= -1;
				ySpeed *= -1;
				turningSpeed *= 1;
			}
			// Convert ChassisSpeeds into the ChassisSpeeds type
			if (DriveConstants.fieldOriented) {
				chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
						ySpeed, turningSpeed, drivetrainS.getRotation2d());
			} else {
				chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
			}
			if (DriveConstants.vendor == DriveConstants.driveTrainType.TANK) {
				chassisSpeeds.vyMetersPerSecond = 0;
			}
			// set modules to proper speeds
			if (Math.abs(xSpeed) < DriveConstants.SwerveConstants.kDeadband
					&& Math.abs(ySpeed) < DriveConstants.SwerveConstants.kDeadband
					&& Math.abs(
							turningSpeed) < DriveConstants.SwerveConstants.kDeadband) {
				drivetrainS.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));//for odom
				drivetrainS.stopModules();
			} else {
				drivetrainS.setChassisSpeeds(chassisSpeeds);
			}
			break;
		}
	}
	/*Use this link to compute the regression model:https://planetcalc.com/5992/#google_vignette 
	 Each of the files has an x and y output so put those in the respective lists, or use a ti-84 stats bar*/
	/*public void printData() {
		//outputs collected distance vs angle graph to console and also sends it to the data logging file. 
		System.out.println("Distance (X)                              Angle (Y)");
		for (var i = 0; i < arrayIndex; i++) {
			String output = " " + Double.toString(variableAngleLog[0][i]) + "    "
					+ Double.toString(variableAngleLog[1][i]);
			System.out.println(output);
			DataHandler
					.logData(new String[] { Double.toString(variableAngleLog[0][i]),
							Double.toString(variableAngleLog[1][i])
					});
		}
		System.out.println("Distance (X)");
		for (var i = 0; i < arrayIndex; i++) {
			String output = Double.toString(variableAngleLog[0][i]) + " ";
			System.out.println(output);
		}
		System.out.println("Angle (Y)");
		for (var i = 0; i < arrayIndex; i++) {
			String output = Double.toString(variableAngleLog[1][i]) + " ";
			System.out.println(output);
		}
		variableAngleLog = new double[2][20];
		arrayIndex = 0;
	} */

	@Override
	public void end(boolean interrupted) { drivetrainS.stopModules(); }

	@Override
	public boolean isFinished() { return false; }
}
