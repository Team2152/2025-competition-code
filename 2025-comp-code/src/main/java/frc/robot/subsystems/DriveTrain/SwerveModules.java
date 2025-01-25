package frc.robot.subsystems.DriveTrain;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModules {    
    
    private SparkMax max = new SparkMax(1, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
   
}

