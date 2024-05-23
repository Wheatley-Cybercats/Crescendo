// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leadscrew.Leadscrew;
import org.littletonrobotics.junction.Logger;

public class AutoLeadscrewCommand extends Command {
  // private static final double LEADSCREW_CF = 300130; // do not set to zero will return NaN
  private final Leadscrew leadscrew;
  private final Translation3d target;
  // private double leadscrewLegLength = Units.inchesToMeters(10.25);
  // private double leadscrewLegLength2 = Units.inchesToMeters(10.25);
  private double position = 0.0;

  /** Creates a new AutoLeadscrewCommand. */
  public AutoLeadscrewCommand(Leadscrew leadscrew, Translation3d target) {
    this.leadscrew = leadscrew;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leadscrew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /*
   * public static final double AMP_ANGLE = 91.5;
  public static final double PODIUM_ANGLE = 36; distance of 2.8 meters from wall 1 meter is 15 lead screw positions
  public static final double WING_ANGLE =
      21; // 17; //when front of bumpers are lined up with outer edge of stage distance of 3.9 meters from wall
  public static final double SUBWOOFER_ANGLE = 115;
   */
  @Override
  public void execute() {
    /*
       Pose2d curLoc = RobotState.getInstance().getEstimatedPose();
       Pose2d allianceAdjustedSpeaker = new Pose2d();
       if (DriverStation.getAlliance().isPresent())
         allianceAdjustedSpeaker =
                 DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                         ? new Pose2d(target.getX(), target.getY(), Rotation2d.fromDegrees(0))
                         : new Pose2d(
                         target.getX(),
                         target.getY(),
                         Rotation2d.fromDegrees(180)); // TODO: data for red alliance speaker
       else SmartDashboard.putString("System Status", "Auto-aiming alliance cannot be obtained");

       position =
               calculateSetPoint(
                       calculateDistance(
                               curLoc.getX(),
                               allianceAdjustedSpeaker.getX(),
                               curLoc.getY(),
                               allianceAdjustedSpeaker.getY()));

    */

    Logger.recordOutput("Lead Screw Position/Leadscrew Target", position);
    leadscrew.runSetpoint(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leadscrew.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leadscrew.atPosition(position);
  }

  private double calculateSetPoint(double distance) {
    return (Math.pow(distance, 1.0 / 2.33)) * 7.2;
  }

  private double calculateDistance(double x1, double x2, double y1, double y2) {
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }
}
