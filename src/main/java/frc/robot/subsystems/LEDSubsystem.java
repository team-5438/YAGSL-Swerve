package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class LEDSubsystem {
    public static AddressableLED sponsorStrip1 = new AddressableLED(0);

    public LEDSubsystem(int length) {
        AddressableLEDBuffer sponsorStrip1Buffer = new AddressableLEDBuffer(length);
        sponsorStrip1.setLength(length);
        sponsorStrip1.setData(sponsorStrip1Buffer);
        sponsorStrip1.start();
    }
}