// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Getter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final Mode currentMode = Mode.REAL;
  public static final boolean useVision = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Create enum and assign values to each preset
  @Getter
  public static enum PresetLeadscrewAngle {
    AMP(AMP_ANGLE),
    PODIUM(PODIUM_ANGLE),
    WING(WING_ANGLE),
    SUBWOOFER(SUBWOOFER_ANGLE),
    AUTO(AUTO_ANGLE);

    private final double angle;

    PresetLeadscrewAngle(double angle) {
      this.angle = angle;
    }
  }

  public static enum PresetFlywheelSpeed {
    AMP(AMP_FLYWHEEL),
    SPEAKER(SPEAKER_FLYWHEEL),
    LOB(LOB_FLYWHEEL);

    private final double[] angle;

    PresetFlywheelSpeed(double[] angle) {
      this.angle = angle;
    }

    public double getVoltBot() {
      return angle[0];
    }

    public double getVoltTop() {
      return angle[1];
    }

    public double[] getVolt() {
      return angle;
    }
  }

  public static final double AMP_ANGLE = 81.8;
  public static final double PODIUM_ANGLE = 36;
  public static final double WING_ANGLE =
      23; // 17; //when front of bumpers are lined up with outer edge of stage
  public static final double SUBWOOFER_ANGLE = 115;
  public static final double AUTO_ANGLE = 55;
  public static final boolean tuningMode = false;
  public static final double[] AMP_FLYWHEEL = {-4.1, 0.5}; // [BOT, TOP]
  public static final double[] SPEAKER_FLYWHEEL = {-7.5, 7.5}; // [BOT, TOP]
  public static final double[] LOB_FLYWHEEL = {-5, 5}; // [BOT, TOP]

  public static final Transform3d leftCamToRobot = new Transform3d();
  public static final Transform3d rightCamToRobot = new Transform3d();
}
