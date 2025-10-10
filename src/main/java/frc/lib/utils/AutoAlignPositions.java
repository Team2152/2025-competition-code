package frc.lib.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

public class AutoAlignPositions {
    public static Pose2d getReefPoint(double anchorX, double anchorY, double angle, double offsetX, double offsetY) {
        double x = anchorX - offsetX;
        double y = anchorY - offsetY;
       
        double angleRadians = Math.toRadians(angle);

        double translatedX = x - anchorX;
        double translatedY = y - anchorY;

        double rotatedX = translatedX * Math.cos(angleRadians) - translatedY * Math.sin(angleRadians);
        double rotatedY = translatedX * Math.sin(angleRadians) + translatedY * Math.cos(angleRadians);

        double finalX = rotatedX + anchorX;
        double finalY = rotatedY + anchorY;
        
        return new Pose2d(finalX, finalY, new Rotation2d(angleRadians));
    }

    public static List<Pose2d> getReefPoints(double anchorX, double anchorY, double offsetX, double offsetY) {
        List<Pose2d> reefPoints = new ArrayList<>();

        double[] angles = {0, 30, 60, 180, 210, 240};

        for (double angle : angles) {
            reefPoints.add(getReefPoint(anchorX, anchorY, angle,  offsetX, offsetY));
            reefPoints.add(getReefPoint(anchorX, anchorY, angle, -offsetX, offsetY));
        }

        return reefPoints;
    }

    public static Pose2d findClosestPose(Pose2d target, boolean isRed) {
        Pose2d closest = null;
        List<Pose2d> poses = null;
        double minDistance = Double.MAX_VALUE;

        if (!isRed) {
            poses = DrivetrainConstants.PIDs.AutoAlign.kRedReefPositions;
        } else {
            poses = DrivetrainConstants.PIDs.AutoAlign.kBlueReefPositions;
        }
    
        for (Pose2d pose : poses) {
            double distance = pose.getTranslation().getDistance(target.getTranslation());
    
            if (distance < minDistance) {
                minDistance = distance;
                closest = pose;
            }
        }
    
        return closest;
    }
    
}
