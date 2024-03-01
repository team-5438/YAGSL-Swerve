package frc.robot.utils;

public class Conversions {

    public static double inchToCentimete(double inch) {
        return inch * 2.54;
    }

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    public static double degreesToRad(double degrees) {
        return degrees * Math.PI / 180;
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double SparkMaxToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return SparkMax rotations
     */
    public static double degreesToSparkMax(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double sparkMaxToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = velocitycounts;
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double MetersToFalcon(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double sparkMaxToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / gearRatio);
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double MetersToSparkMax(double meters, double circumference, double gearRatio){
        return meters / (circumference / gearRatio);
    }
}