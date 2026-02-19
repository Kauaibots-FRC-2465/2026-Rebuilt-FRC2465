package frc.robot.utility;

public class Mutable<T> {
    T internal;

    Mutable(T first) {
        internal = first;
    }

    public void set(T rep) {
        internal = rep;
    }
}
