package frc.robot.util;

public class MathUtils {

    public static <T extends Comparable<T>> T clamp(T min, T max, T value){
        return value.compareTo(min) < 0 ? min : value.compareTo(max) > 0 ? max : value;
    }

}
