package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    public enum ClimberStates {
        IDLE,
        DEPLOYING,
        DEPLOYED,
        CLIMBING,
        CLIMBED
    }

    private final TalonFX m_intakeMotor;
    private final TalonFX m_climberMotor;

    private ClimberStates m_currentState = ClimberStates.IDLE;

    private double m_climberTarget = 0.0;

    private static final double kDeployed = 3.5;
    private static final double kClimbed = 55.0;
    private static final double kTolerance = 0.5; // acceptable error

    public Climber() {
        m_intakeMotor = new TalonFX(Constants.CANConstants.Climber.kIntake);
        m_climberMotor = new TalonFX(Constants.CANConstants.Climber.kClimber);
    }

    @Override
    public void periodic() {
        double currentPosition = getPosition();

        SmartDashboard.putNumber("Climber Position", currentPosition);
        SmartDashboard.putString("Climber State", m_currentState.name());
        SmartDashboard.putNumber("Climber Target", m_climberTarget);

        switch (m_currentState) {
            case DEPLOYING:
            case CLIMBING:
                if (currentPosition < m_climberTarget) {
                    // Run motor
                    double power = (m_currentState == ClimberStates.DEPLOYING) ? 0.1 : 1.0;
                    m_climberMotor.set(power);
                } else {
                    // Target reached
                    m_climberMotor.set(0);

                    if (m_currentState == ClimberStates.DEPLOYING) {
                        m_currentState = ClimberStates.DEPLOYED;
                    } else if (m_currentState == ClimberStates.CLIMBING) {
                        m_currentState = ClimberStates.CLIMBED;
                    }

                    // Clear target
                    m_climberTarget = 0.0;
                }
                break;

            default:
                // Idle or finished
                m_climberMotor.set(0);
                break;
        }
    }

    private double getPosition() {
        return m_climberMotor.getPosition().getValueAsDouble();
    }

    public Command runIntakeCmd(double power) {
        return runOnce(() -> m_intakeMotor.set(power));
    }

    public Command runPivotCmd(double power) {
        return runOnce(() -> m_climberMotor.set(power));
    }

    public Command setStateCmd(ClimberStates state) {
        return runOnce(() -> {
            if (m_currentState != ClimberStates.CLIMBED) {
                m_currentState = state;
            }
        });
    }

    public Command Climb() {
        return runOnce(() -> {
            if (m_currentState == ClimberStates.IDLE) {
                // First stage: deploy
                m_climberTarget = kDeployed;
                m_currentState = ClimberStates.DEPLOYING;
            } else if (m_currentState == ClimberStates.DEPLOYED) {
                // Second stage: climb
                m_climberTarget = kClimbed;
                m_currentState = ClimberStates.CLIMBING;
            }
            // Do nothing if already climbing or climbed
        });
    }

    public void reset() {
        m_currentState = ClimberStates.IDLE;
        m_climberTarget = 0.0;
        m_climberMotor.set(0);
    }

    public Command resetCmd() {
        return runOnce(this::reset);
    }
}
