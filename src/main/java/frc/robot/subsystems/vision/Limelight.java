// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LimelightHelpers;

public class Limelight extends SubsystemBase {
  public String m_limelightName;
  public Limelight(String name) {
    m_limelightName = name;
  }

  public boolean getTV() {
    return LimelightHelpers.getTV(m_limelightName);
  }

  public int getFiducialID() {
    int tagID = (int) LimelightHelpers.getFiducialID(m_limelightName);
    return tagID;
  }

  public double getTX() {
    return LimelightHelpers.getTX(m_limelightName);
  }

  public double getTY() {
    return LimelightHelpers.getTY(m_limelightName);
  }

  public double getTA() {
    return LimelightHelpers.getTA(m_limelightName);
  }

  public void setLight(boolean state) {
    if (state) {
      LimelightHelpers.setLEDMode_ForceOn(m_limelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(m_limelightName);
    }
    
  }
}
