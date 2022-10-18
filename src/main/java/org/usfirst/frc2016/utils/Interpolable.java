package org.usfirst.frc2016.utils;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
