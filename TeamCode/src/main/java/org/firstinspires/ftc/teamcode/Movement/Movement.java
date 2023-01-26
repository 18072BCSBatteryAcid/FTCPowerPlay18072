package org.firstinspires.ftc.teamcode.Movement;

//import com.   qualcomm.       robotcore.  util.                  Hardware;


public class Movement {

    static int   timing = 0;
    static int   rTime  = 0;
    static float X      = 0;
    static float Y      = 0;
    static float fl     = 0;
    static float fr     = 0;
    static float bl     = 0;
    static float br     = 0;

    public static float[] Strafing(float x, float y, float speed, float cm){

        fl = ( speed* X + speed*-Y );
        bl = ( speed*-X + speed*-Y );
        fr = ( speed* X + speed* Y );
        br = ( speed*-X + speed* Y );
        return new float[]{fl, bl, fr, br};

    }

}


