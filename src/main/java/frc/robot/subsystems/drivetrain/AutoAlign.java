package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.math.geometry.Pose2d;

/// Overall:
///     Setup Poses for each position on reef to align to 
///     Go to nearest Pose on reef
///     From Poses, move left/right based off an offset

public class AutoAlign extends SubsystemBase{

    public AutoAlign(Drivetrain m_drivetrain) {
        System.out.println(m_drivetrain.getPose());

    }
}
