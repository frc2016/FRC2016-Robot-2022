package org.usfirst.frc2016.math.spline;

import org.usfirst.frc2016.math.Rotation2;
import org.usfirst.frc2016.math.Vector2;

public abstract class Spline {

    public abstract Vector2 getPoint(double t);

    public abstract Rotation2 getHeading(double t);
}
