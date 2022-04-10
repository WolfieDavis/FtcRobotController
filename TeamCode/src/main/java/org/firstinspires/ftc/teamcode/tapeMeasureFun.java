package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.ServoX;

@TeleOp
public class tapeMeasureFun extends OpMode {

    private DcMotorX
            tapeExtend;
    private ServoX
            tapePan,
            tapeTilt;

    private double
            tapeExtendPower = 1, //.85
            tapePanValue = 90,
            tapeTiltValue = 90,
            panMin = 90 - 50, //left
            panMax = 90 + 50, //right
            tiltMin = 90 - 50, //down
            tiltMax = 90 + 40, //up
            tapePanMultiplier = 0.2, //.15
            tapeTiltMultiplier = 0.15; //.15

    public void init() {
        tapeExtend = new DcMotorX(hardwareMap.dcMotor.get("odoRear"));
        tapePan = new ServoX(hardwareMap.servo.get("tapePan"), 180, panMax, panMin);
        tapeTilt = new ServoX(hardwareMap.servo.get("tapeTilt"), 180, tiltMax, tiltMin);

    }// end of init

    public void loop() {
        /* ------------------------ tape measure pan/tile/extend ----------------------- */
        if (Math.abs(gamepad2.left_stick_y) > .1)
            tapeExtend.setPower(-gamepad2.left_stick_y * tapeExtendPower);
        else tapeExtend.setPower(0);

        if (Math.abs(gamepad2.right_stick_x) > .1 || Math.abs(gamepad2.right_stick_y) > .1) {
            tapePanValue += gamepad2.right_stick_x * tapePanMultiplier;
            tapeTiltValue += gamepad2.right_stick_y * -tapeTiltMultiplier;
        }

        tapePan.setAngle(tapePanValue);
        tapeTilt.setAngle(tapeTiltValue);

        telemetry.addData("pan", tapePanValue);
        telemetry.addData("tilt", tapeTiltValue);
        telemetry.update();

    }//end of loop

}