package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDCommand extends Command {
    public static AddressableLEDBuffer setStripColor(int length, int r, int g, int b) {
        AddressableLEDBuffer m_ledbuffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < m_ledbuffer.getLength(); i++)
            m_ledbuffer.setRGB(i, r, g, b);
        return m_ledbuffer;
    }
}

class FlashLEDS extends Thread {
    private int ar[][];
    private String id;
    private AddressableLED led;

    private Thread t;

    public FlashLEDS(String id, AddressableLED led, int ar[][]) {
        this.ar = ar;
        this.id = id;
        this.led = led;

        t = new Thread(this, "LED Thread");
        t.start();
    }

    @Override
    public void run() {
        int r = 0;
        int b = 0;
        int g = 0;
        int time = 0;
        for (int i = 0; i <= ar.length; i++) {
            try {
                Thread.sleep(time);
            } catch (InterruptedException e) {}
            for (int e = 0; e < 4; e++) {
                r = ar[i][0];
                g = ar[i][1];
                b = ar[i][2];
                time = ar[i][3];
            }            
            LEDSubsystem.setStrip(id, led, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), r, g, b));
        }
    }
}
