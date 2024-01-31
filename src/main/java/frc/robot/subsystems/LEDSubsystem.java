package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class LEDSubsystem {
    public static AddressableLED sponsorStrip1;

    public LEDSubsystem(int port, int length) {
        sponsorStrip1 = new AddressableLED(port);
        AddressableLEDBuffer sponsorStrip1Buffer = new AddressableLEDBuffer(length);
        sponsorStrip1.setLength(length);
        sponsorStrip1.setData(sponsorStrip1Buffer);
    }
}