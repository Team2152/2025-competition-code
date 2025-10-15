package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// Built referencing https://shenzhen-robotics-alliance.github.io/maple-sim/swerve-sim-easy/
public class SimDrivetrain extends Drivetrain {

    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    public SimDrivetrain() {
        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();
                // // Specify gyro type (for realistic gyro drifting and error simulation)
                // .withGyro(COTS.ofPigeon2())
                // // Specify swerve module (for realistic swerve dynamics)
                // .withSwerveModule(COTS.ofMark4(
                //         DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                //         DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                //         COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                //         3)) // L3 Gear ratio
                // // Configures the track length and track width (spacing between swerve modules)
                // .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
                // // Configures the bumper size (dimensions of the robot bumper)
                // .withBumperSize(Inches.of(30), Inches.of(30));


        // Creating the SelfControlledSwerveDriveSimulation instance
        simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(3, 2, new Rotation2d()))
        );

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("Simulation", field2d);
    }

    @Override
    public void periodic() {
        simulatedDrive.periodic();
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(simulatedDrive.getOdometryEstimatedPose());
        super.periodic();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    @Override
    public void resetPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    double headingOffset = 0;

    @Override
    public Command zeroHeading() {
        return runOnce(() -> headingOffset = getHeading() + headingOffset);
    }

    @Override
    public double getHeading() {
        if (simulatedDrive == null) return 0;
        
        return gyroSim().getGyroReading().getDegrees() - headingOffset;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return simulatedDrive.getMeasuredStates();
    }

    @Override
    public void setStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    private GyroSimulation gyroSim() {
        return simulatedDrive.getDriveTrainSimulation().getGyroSimulation();
    }
}
