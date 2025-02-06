package frc.lib.utils;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveWidget {
    public SwerveWidget() {

    }

    public static void sendWidget(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight, PigeonIMU gyro) {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> frontRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> rearLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> rearLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> rearRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> rearRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> Conversions.degreesToRadians(gyro.getFusedHeading()), null);
            }
        });
    }
}