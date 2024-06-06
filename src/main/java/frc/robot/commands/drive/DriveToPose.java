// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.LoggableTunedNumber;

import java.util.function.Supplier;
import frc.robot.utils.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final DrivetrainS drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  private static final LoggableTunedNumber driveKp = new LoggableTunedNumber("DriveToPose/DriveKp");
  private static final LoggableTunedNumber driveKd = new LoggableTunedNumber("DriveToPose/DriveKd");
  private static final LoggableTunedNumber thetaKp = new LoggableTunedNumber("DriveToPose/ThetaKp");
  private static final LoggableTunedNumber thetaKd = new LoggableTunedNumber("DriveToPose/ThetaKd");
  private static final LoggableTunedNumber driveMaxVelocity =
      new LoggableTunedNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggableTunedNumber driveMaxVelocitySlow =
      new LoggableTunedNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggableTunedNumber driveMaxAcceleration =
      new LoggableTunedNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggableTunedNumber thetaMaxVelocity =
      new LoggableTunedNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggableTunedNumber thetaMaxVelocitySlow =
      new LoggableTunedNumber("DriveToPose/ThetaMaxVelocitySlow");
  private static final LoggableTunedNumber thetaMaxAcceleration =
      new LoggableTunedNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggableTunedNumber driveTolerance =
      new LoggableTunedNumber("DriveToPose/DriveTolerance");
  private static final LoggableTunedNumber driveToleranceSlow =
      new LoggableTunedNumber("DriveToPose/DriveToleranceSlow");
  private static final LoggableTunedNumber thetaTolerance =
      new LoggableTunedNumber("DriveToPose/ThetaTolerance");
  private static final LoggableTunedNumber thetaToleranceSlow =
      new LoggableTunedNumber("DriveToPose/ThetaToleranceSlow");
  private static final LoggableTunedNumber ffMinRadius =
      new LoggableTunedNumber("DriveToPose/FFMinRadius");
  private static final LoggableTunedNumber ffMaxRadius =
      new LoggableTunedNumber("DriveToPose/FFMinRadius");

  static {
        driveKp.initDefault(2.0);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(5.0);
        thetaKd.initDefault(0.0);
        driveMaxVelocity.initDefault(Units.inchesToMeters(150.0));
        driveMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.01);
        driveToleranceSlow.initDefault(0.06);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
        thetaToleranceSlow.initDefault(Units.degreesToRadians(3.0));
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainS drive, Pose2d pose) {
    this(drive, false, pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainS drive, boolean slowMode, Pose2d pose) {
    this(drive, slowMode, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainS drive, Supplier<Pose2d> poseSupplier) {
    this(drive, false, poseSupplier);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainS drive, boolean slowMode, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());
    lastSetpointTranslation = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || driveToleranceSlow.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxVelocitySlow.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || thetaToleranceSlow.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? driveMaxVelocitySlow.get() : driveMaxVelocity.get(),
              driveMaxAcceleration.get()));
      driveController.setTolerance(slowMode ? driveToleranceSlow.get() : driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowMode ? thetaMaxVelocitySlow.get() : thetaMaxVelocity.get(),
              thetaMaxAcceleration.get()));
      thetaController.setTolerance(slowMode ? thetaToleranceSlow.get() : thetaTolerance.get());
    }

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger
        .recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger
        .recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger
        .recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger
        .recordOutput(
            "Odometry/DriveToPoseSetpoint",
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stopModules();
    Logger.recordOutput("Odometry/DriveToPoseSetpoint", new double[] {});
    Logger.recordOutput("Odometry/DriveToPoseGoal", new double[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}