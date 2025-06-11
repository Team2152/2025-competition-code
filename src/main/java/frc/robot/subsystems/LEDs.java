package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.coralinator.Coralinator;
import frc.robot.subsystems.coralinator.Coralinator.CoralinatorStates;

public class LEDs extends SubsystemBase {
    private final DigitalOutput m_dio;

    private boolean blinkEnabled = false;
    private boolean blinking = false;
    private boolean enabled = false;

    private Coralinator m_Coralinator;


    public LEDs(int dioPort, Coralinator coralinator) {
        m_Coralinator = coralinator;
        m_dio = new DigitalOutput(dioPort);

        Thread flashThread = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(300);
                    blinking = false;
                    Thread.sleep(300);
                    blinking = true;
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        flashThread.start();
    }

    @Override
    public void periodic() {
      if (m_Coralinator.m_currentState == CoralinatorStates.INDEXING || m_Coralinator.m_currentState == CoralinatorStates.PULLBACK) {
        setBlink(LEDConstants.Intaking.blinks);
        enabled = true;
      } else if (m_Coralinator.m_currentState == CoralinatorStates.READY) {
        setBlink(LEDConstants.Ready.blinks);
        enabled = true;
      } else {
        enabled = false;
      }

        if ((!enabled) && (!blinkEnabled || (blinkEnabled && blinking))) {
            m_dio.set(true);
        } else {
            m_dio.set(false);
        }
    }

    public Command setBlinkCmd(boolean enabled) {
        return runOnce(
            () -> setBlink(enabled)
        );
    }

    public void setBlink(boolean enabled) {
      blinkEnabled = enabled;
    }
}