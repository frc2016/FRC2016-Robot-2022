package org.usfirst.frc2016;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.*;

/*
 * 			1
 * 		4		2
 * 			3
 */
public class POVTrigger extends Button {
    private Direction direction;
    private GenericHID joystick;

    public enum Direction {
        Up, Left, Down, Right
    }
    

    POVTrigger(GenericHID joystick, Direction direction) {
        this.joystick = joystick;
        this.direction = direction;
    }

    public boolean get() {
        int pov;
        boolean returnValue;

        pov = joystick.getPOV();
        switch (pov) {

            /*
             * This is the case where the POV is centered, stop the intake here
             */
            case -1:
                // case 90:
                // case 270:
                returnValue = false;
                break;

            /*
             * POV is pressed forward, cause intact to retract.
             */
            case 0:
                // case 45:
                // case 315:
                returnValue = direction == Direction.Up;
                break;

            /*
             * POV is pressed left, cause the roller to intake the ball.
             */
            case 90:
                // case 45:
                // case 315:
                returnValue = direction == Direction.Left;
                break;

            /*
             * POV is back, cause the ball to enter the bot.
             */
            case 180:
                // case 135:
                // case 225:
                returnValue = direction == Direction.Down;
                break;

            case 270:
                // case 135:
                // case 225:
                returnValue = direction == Direction.Right;
                break;

            /*
             * While this isn't obvious, there are positions between the above cases. If the
             * operator drifts into one of these, the defalut code below prevents the roller
             * from stopping. If the pov continues to move and lands on one of the above
             * cases, the roller will change as needed.
             */
            default:
                returnValue = false;
                break;
        }

        return returnValue;
    }
}
