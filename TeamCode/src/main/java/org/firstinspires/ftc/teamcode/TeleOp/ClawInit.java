package  org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Claw Innit Bruv", group="British OpModes")
public class ClawInit extends LinearOpMode {

    private Servo claw;

    @Override
    public void runOpMode() {

        claw = hardwareMap.get(Servo.class, "Claw");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        claw.setPosition(0.45F);

        waitForStart();
        
    }

}