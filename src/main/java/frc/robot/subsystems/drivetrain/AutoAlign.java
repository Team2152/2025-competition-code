package frc.robot.subsystems.drivetrain;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/// Overall:
///     Setup Poses for each position on reef to align to 
///     go to nearest Pose on reef
///     From Poses, move left/right based off an offset
/// 
/// Commands:
///     getClosestPose(Pose2d 'currentRobotPose') -> Pose2d from 'reefPositions'
///         returns the pose with the lowest difference in x/y compared to the Robot.

public class AutoAlign extends SubsystemBase{
    private final ArrayList<Pose2d> m_reefPositions = DrivetrainConstants.PIDs.AutoAlign.kReefPositions;
    
    public AutoAlign(Drivetrain m_drivetrain) {
        System.out.println(m_drivetrain.getPose());
    }

    
}
