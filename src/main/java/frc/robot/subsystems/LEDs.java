package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.coralinator.Coralinator;
import frc.robot.subsystems.coralinator.Coralinator.CoralinatorStates;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_ledBuffer;

    private Color currentColor = LEDConstants.Default.color;

    private boolean blinkEnabled = false;
    private boolean blinking = false;

    private Coralinator m_Coralinator;


    public LEDs(int pwmPort, int ledLength, Coralinator coralinator) {
        m_Coralinator = coralinator;
        m_leds = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);

        m_leds.setLength(ledLength);
        m_leds.start();

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
        setColor(LEDConstants.Intaking.color);
      } else if (m_Coralinator.m_currentState == CoralinatorStates.READY) {
        setBlink(LEDConstants.Ready.blinks);
        setColor(LEDConstants.Ready.color);
      } else {
        setBlink(LEDConstants.Default.blinks);
        setColor(LEDConstants.Default.color);
      }
        m_leds.setData(m_ledBuffer);

        if (!blinkEnabled || (blinkEnabled && blinking)) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, currentColor);
            }
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    public Command setColor(Color color) {
        return runOnce(
            () -> currentColor = color
        );
    }

    public Command setBlinkCmd(boolean enabled) {
        return runOnce(
            () -> setBlink(enabled)
        );
    }

    public void setBlink(boolean enabled) {
      blinkEnabled = enabled;
    }

    public void enable() {
        m_leds.start();
    }

    public void disable() {
        m_leds.stop();
    }
}