package org.firstinspires.ftc.teamcode.TeleOp;

import   com.    qualcomm.       hardware.   bosch.                  BNO055IMU;
import   com.    qualcomm.       robotcore.  eventloop.              EventLoopManager;
import   com.    qualcomm.       robotcore.  eventloop.              opmode.             Disabled;
import   com.    qualcomm.       robotcore.  eventloop.              opmode.             LinearOpMode;
import   com.    qualcomm.       robotcore.  eventloop.              opmode.             TeleOp;
import   com.    qualcomm.       robotcore.  hardware.               DcMotor;
import   com.    qualcomm.       robotcore.  hardware.               DcMotorSimple;
import   com.    qualcomm.       robotcore.  hardware.               DigitalChannel;
import   com.    qualcomm.       robotcore.  hardware.               DistanceSensor;
import   com.    qualcomm.       robotcore.  hardware.                Gyroscope;
import   com.    qualcomm.       robotcore.  hardware.               HardwareMap;
import   com.    qualcomm.       robotcore.  hardware.               I2cWarningManager;
import   com.    qualcomm.       robotcore.  hardware.               Servo;
import   com.    qualcomm.       robotcore.  robot.                  Robot;
import   com.    qualcomm.       robotcore.  util.                   ElapsedTime;
//import com.    qualcomm.       robotcore.  util.                   Hardware;
import   com.    qualcomm.       robotcore.  util.                   Range;
import   com.    qualcomm.       robotcore.  util.                   RobotLog;
import   com.    qualcomm.       robotcore.  util.                   ThreadPool;
import   org.    firstinspires.  ftc.        robotcore.              external.           navigation.     AngleUnit;
import   org.    firstinspires.  ftc.        robotcore.              external.           navigation.     AxesOrder;
import   org.    firstinspires.  ftc.        robotcore.              external.           navigation.     AxesReference;
import   org.    firstinspires.  ftc.        robotcore.              external.           navigation.     Orientation;
import   org.    firstinspires.  ftc.        robotcore.              internal.           opmode.         TelemetryInternal;
import   java.   lang.           Math;
import   java.   util.           concurrent. CancellationException;
import   java.   util.           concurrent. ExecutorService;
import   java.   util.           concurrent. TimeUnit;
import   java.   util.           Stack;

@TeleOp
public   class   manualTank      extends     LinearOpMode {

    private  BNO055IMU       emu;
    private  Orientation     angles;
    private  DcMotor         FLwheel;
    private  DcMotor         FRwheel;
    private  DcMotor         BLwheel;
    private  DcMotor         BRwheel;
    private  DcMotor         lift;
    private  Servo           claw;
    //private  DigitalChannel  digitalTouch;
    //private  DistanceSensor  sensorColorRange;
    //private  Servo           servoTest;

    @Override
    public void runOpMode() {

        // this is where you initialize the motors and point them to the correct hardware
        FLwheel           = hardwareMap.get(DcMotor.class, "LFront" );
        FRwheel           = hardwareMap.get(DcMotor.class, "RFront" );
        BLwheel           = hardwareMap.get(DcMotor.class, "LRear"  );
        BRwheel           = hardwareMap.get(DcMotor.class, "RRear"  );
        lift              = hardwareMap.get(DcMotor.class, "Lift"   );
        claw              = hardwareMap.get(Servo.class,   "Claw"   );

        // this is where you initialize the gyro
        BNO055IMU.Parameters parameters = new   BNO055IMU.Parameters(   );
        parameters.angleUnit            =       BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  =       "BNO055IMUCalibration.json";
        emu = hardwareMap.get(BNO055IMU.class,  "imu"         );
        emu.initialize(parameters);

        // this is where you declare all the variables
        double  fl         =  0;
        double  fr         =  0;
        double  bl         =  0;
        double  br         =  0;
        int     speed      =  5;
        double  usedSpeed  =  0.6;
        double  direction  =  0;
        double  liftS      =  0;
        int     liftL      =  0;
        double  clawS      =  0.5D;
        int     clawL      =  0;
        boolean twitch     =  false;
        boolean dh         =  false;
        int     dt         =  0;
        boolean togCard    =  false;
        float   x          =  0;
        float   y          =  0;
        float   threshold  =  (float)Math.sqrt(2) / 2;
        boolean lbh        =  true;
        boolean togRight   =  false;
        boolean rbh        =  false;
        float   lt         =  0;
        float   rt         =  0;
        float[] temp;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        // this loop is where you put all the code that runs during the opMode
        while (opModeIsActive()) {

            // this updates the gyroscope every frame
            angles = emu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // the following if statements can increase, decrease, or reset the speed of the robot
            speed = 5;

            if (this.gamepad1.a){

                speed = 9;

            }

            if (this.gamepad1.b){

                speed = 1;

            }

            if (this.gamepad1.y){

                emu.initialize(parameters);

            }

            usedSpeed = (speed + 1) / 10.0;

            if (this.gamepad1.left_stick_button && lbh){

                togCard = !togCard;
                lbh     = false;

            }

            if (!this.gamepad1.left_stick_button){

                lbh = true;

            }

            x = this.gamepad1.left_stick_x;
            y = this.gamepad1.left_stick_y;

            if (togCard){

                if (Math.abs(x) <= threshold){

                    x = 0;

                } else {

                    x = (this.gamepad1.left_stick_x / Math.abs(this.gamepad1.left_stick_x));

                }

                if (Math.abs(y) <= threshold){

                    y = 0;

                } else {

                    y = (this.gamepad1.left_stick_y / Math.abs(this.gamepad1.left_stick_y));

                }

            }







            // these set all the motor values
            fl  = (-x  +  y  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger) * usedSpeed;
            fr  = (-x  + -y  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger) * usedSpeed;
            br  = ( x  + -y  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger) * usedSpeed;
            bl  = ( x  +  y  +  this.gamepad1.left_trigger  +  -this.gamepad1.right_trigger) * usedSpeed;

            // these move the lift
            if (this.gamepad1.right_bumper/* && liftL < 1800*/){

                liftS = -10;
                liftL++;

            } else if (this.gamepad1.left_bumper/* && liftL > 0*/){

                liftS = 10;
                liftL--;

            } else {

                liftS = 0;

            }

            if (this.gamepad1.dpad_down && !dh){

                dh = true;
                twitch = !twitch;

            }

            if (dh && !this.gamepad1.dpad_down){

                dh = false;

            }

            if (twitch){

                dt++;

                if (dt % 5 < 2){

                    clawS = 1F;

                } else {

                    clawS = 0.5F;

                }

            }

            if (this.gamepad1.dpad_left /*&& clawL > 0*/){

                clawS = 0F;
                clawL--;

            } else if (this.gamepad1.dpad_right /*&& clawL < 50*/){

                clawS = 1F;
                clawL++;

            }else if (!twitch){

                clawS = 0.5F;

            }

            // this moves the motors
            FLwheel.setPower(    fl    );
            FRwheel.setPower(    fr    );
            BLwheel.setPower(    bl    );
            BRwheel.setPower(    br    );
            lift.   setPower(    liftS );
            claw.   setPosition( clawS );

            // this displays things to the screen
            telemetry.addData("Status"    ,  "Running"   );
            telemetry.addData("Speed"     ,  usedSpeed         );
            telemetry.addData("Heading"   ,  angles.firstAngle );
            telemetry.update();

        }

    }

}