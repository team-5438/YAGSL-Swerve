package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* our lock object */
class LEDLock {
    String id;
    AddressableLED led;
}

public class LEDSubsystem extends SubsystemBase {
    public static AddressableLED led0;
    public static AddressableLEDBuffer led0Buffer;
    public static ArrayList<LEDLock> locks;

    public LEDSubsystem() {
        led0 = new AddressableLED(0);
        led0Buffer = new AddressableLEDBuffer(27);

        led0.setLength(led0Buffer.getLength());

        /* set the data */
        led0.setData(led0Buffer);
        led0.start();
    }

    /**
     * set led strip data while respecting locking
     *
     * @param id owner of led
     * @param led led object
     * @param ledBuffer led buffer to get data from
     */
    public static void setStrip(String id, AddressableLED led,
        AddressableLEDBuffer ledBuffer) {
        String locked = isLocked(led);

        if (locked != id || locked != null)
            return;

        led.setData(ledBuffer);
    }

    /**
     * Aquire a lock on a led object. This will prevent the leds from being
     * changed by others unless the objects lock is released.
     *
     * @param id owner of led
     * @param led object to lock
     * @return boolean true if lock aquired false if failed to lock
     */
    public static boolean aquireLock(String id, AddressableLED led) {
        if (isLocked(led) != null)
            return false;

        locks.get(locks.size()).led = led;
        locks.get(locks.size()).id = id;
        return true;
    }

    /**
     * Release a lock on a led object. This will allow the leds to be changed
     * by others.
     *
     * @param id owner of lock
     * @param led object to unlock
     * @return boolean true if unlocked false if still locked
     */
    public static boolean releaseLock(String id, AddressableLED led) {
        int i;

        for (i = 0; i < locks.size(); i++)
            if (locks.get(i).led == led && locks.get(i).id == id) {
                locks.remove(i);
                return true;
            }

        return true;
    }

    /**
     * Check if led is locked
     *
     * @param led led to check
     * @return String id that locked the led or null if not locked
     */
    public static String isLocked(AddressableLED led) {
        int i;

        for (i = 0; i < locks.size(); i++)
            if (locks.get(i).led == led)
                return locks.get(i).id;

        return null;
    }
}
