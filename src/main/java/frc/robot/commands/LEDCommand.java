package frc.robot.commands;
import frc.robot.subsystems.*;

import java.lang.reflect.Array;
import java.util.*;

import java.lang.Thread;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;


public class LEDCommand extends Thread {
    int[][] colors = {{255, 0, 0, 1000}, {66, 245, 144, 2000}, {245, 27, 208, 3000}};

    public void run()
    {
        try {
            flashLeds(colors);
        } catch (Exception e) {
            System.out.println("Interrupted Exception in LEDCommand");
        }
    }
    public static void main(String[] args) { 
        LEDCommand obj = new LEDCommand();
        obj.start();
    }      

    public static AddressableLEDBuffer setStripColor(int length, int r, int g, int b) {
        AddressableLEDBuffer m_ledbuffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < m_ledbuffer.getLength(); i++) {
            m_ledbuffer.setRGB(i, r, g, b);

        }
        return m_ledbuffer;
    }

    public static void flashLeds(int ar[][]) throws InterruptedException {
            int r = 0;
            int b = 0;
            int g = 0;
            int time = 0;
            for (int i = 0; i < ar.length; i++) {
                LEDSubsystem.sponsorStrip1.setData(LEDCommand.setStripColor(12, r, g, b));
                Thread.sleep(time);
                for (int e = 0; e < 4; e++) {
                    if (e == 0)
                        r = ar[i][e];
                    if (e == 1)
                        g = ar[i][e];
                    if (e == 2)
                        b = ar[i][e];
                    if (e == 3)
                        time = ar[i][e];
                }            
            }
        
    }
}