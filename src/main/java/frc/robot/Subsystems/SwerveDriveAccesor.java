// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.team2872.drive.SwerveDrive;
import frc.team2872.drive.SwerveUnit;
import frc.team2872.drive.SwerveUnitConfig;
import frc.team2872.sensors.Pigeon2Wrapper;
 

public class SwerveDriveAccesor extends SubsystemBase implements RobotProperties {
  /** Creates a new SwerveDriveAccesor. */ 
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Pigeon2Wrapper gyro = Robot.gyro;
  public static SwerveDrive drive = Robot.swerveDrive;
  private Field2d field = new Field2d();
  
  public SwerveDriveAccesor() {
    kinematics = drive.swerveDriveKinematics;
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
    
    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(
      new PIDConstants(5, 0, 0), // Translation constants 
      new PIDConstants(5, 0, 0), // Rotation constants 
      4, 
      .49, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );
    //drive.driveInit();
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });


    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project
    
    odometry.update(gyro.getRotation2d(), getPositions());

    field.setRobotPose(getPose());

    Robot.swerveDrive.updatePose();
    
  }
  
  public void disable() {
    drive.disable();
  }
  public void SmartDashboard() {
    // Update Shuffleboard
    drive.SmartDashboard();
}
  public Pose2d getPose() {
    return drive.getPose();
  }
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drive.getRobotRelativeSpeeds();
  }

  public void resetPose(Pose2d pose) {
    drive.resetPose(pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  public double getAverageDriveSpeed(){
    return drive.getAverageDriveSpeed();
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    drive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    drive.setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    drive.setStates(targetStates);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = drive.getModuleState();
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = drive.getPositions();
    return positions;
  }
  public void zero() {
   drive.zero();
  }
  public synchronized void saveSlewCalibration(final String slewOffsets) {
       drive.saveSlewCalibration(slewOffsets);
  }
  
}
