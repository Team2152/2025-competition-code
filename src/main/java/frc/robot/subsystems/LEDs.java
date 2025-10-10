package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.coralinator.Coralinator;
import frc.robot.subsystems.coralinator.Coralinator.CoralinatorStates;

public class LEDs extends SubsystemBase {
    //private final DigitalOutput m_dio;
    private final AddressableLED m_led;
    public int SideLEDCount = 41; // Orange LEDs on the robot's side. Change to Constant
    private boolean blinkEnabled = false;
    private boolean blinking = false;
    private boolean enabled = false;

    //
    private final AddressableLEDBuffer m_buffer;

    //orang skib
    private final AddressableLEDBufferView m_orangeSection;
    private final AddressableLEDBufferView m_greenSection;
    private final AddressableLEDBufferView m_offSection;


    private Coralinator m_Coralinator;


    public LEDs(int pwmPort, Coralinator coralinator) {
        m_Coralinator = coralinator;
        //m_dio = new DigitalOutput(dioPort);
        m_led = new AddressableLED(pwmPort);
        m_buffer = new AddressableLEDBuffer(86);
        m_orangeSection = m_buffer.createView(0, 41);
        m_offSection = m_buffer.createView(42, 49);
        m_greenSection = m_buffer.createView(50, 86);
        // 
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
        flashThread.start(); // Change to start AddressableLED
    }

/*  chatgpt ahh code maybe works???? i dunno needs testing but yeah!!!!!!
    public class LEDController {
        private AddressableLED led;
        private AddressableLEDBuffer ledBuffer;
        private boolean isActive;
    
        public LEDController(int ledPort) {
            led = new AddressableLED(ledPort);
            ledBuffer = new AddressableLEDBuffer(led.getLength());
            led.setLength(ledBuffer.getLength());
            led.start();
        }
    
        public void setActive(boolean active) {
            isActive = active;
            updateLEDs();
        }
    
        private void updateLEDs() {
            if (isActive) {
                for (int i = 0; i <= 41; i++) {
                    ledBuffer.setRGB(i, 255, 165, 0); // Orange
                }
                for (int i = 42; i <= 49; i++) {
                    ledBuffer.setRGB(i, 0, 0, 0); // Off
                }
                for (int i = 50; i <= 82; i++) {
                    ledBuffer.setRGB(i, 0, 255, 0); // Green
                }
            } else {
                // Optional: turn off all LEDs if not active
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setRGB(i, 0, 0, 0);
                }
            }
            led.setData(ledBuffer);
        }
    }*/

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
            //set true
        } else {
            //set false
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