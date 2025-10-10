package frc.lib.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
        return false;
    }
}
