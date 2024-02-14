package frc.robot.commands;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LEDSubsystem;


public class LEDCommand implements Runnable {
    private int ar[][];
    private Thread t;

    public static AddressableLEDBuffer setStripColor(int length, int r, int g, int b) {
        AddressableLEDBuffer m_ledbuffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < m_ledbuffer.getLength(); i++) {
            m_ledbuffer.setRGB(i, r, g, b);
        }
        return m_ledbuffer;
    }

    public void flashLeds(int ar[][]) {
        this.ar = ar;
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
            LEDSubsystem.sponsorStrip1.setData(LEDCommand.setStripColor(27, r, g, b));
        }
    }
}