package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.State;
import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;
import org.firstinspires.ftc.teamcode.api.State;

//@TeleOp
public class jvDrive extends OpMode {


    private Drivetrain drivetrain;

    private DcMotorX
            spinner;

    private ServoX
            plow;

    public void init() {
        DcMotorX mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);
        drivetrain.setBrake(true);

        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));
        plow = new ServoX(hardwareMap.servo.get("plow"));

    }// end of init

    public void loop() {

        //joystick values for driving.
        double leftX = gamepad1.left_stick_x;
        double rightX = -gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller


        //spinner
        if (gamepad1.x) spinner.setPower(0.4);
        else if (gamepad1.b) spinner.setPower(-0.4);
        else spinner.setPower(0);

//        //plow up and down
        if (gamepad1.y) plow.setAngle(90);
        if (gamepad1.a) plow.setAngle(0);


        // Drive the robot with joysticks if they are moved (with rates)
        if (Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            drivetrain.driveWithGamepad(0.6, rightY, leftX, rightX);
//            drivetrain.driveWithGamepad(1, rateCurve(rightY, 1.7), rateCurve(leftX, 1.7)/* 0.5*leftX */, rateCurve(rightX, 1.7)); //curved stick rates
        } else {
            // If the joysticks are not pressed, do not move the bot
            drivetrain.stop();
        }


    }//end of loop

    private double rateCurve(double input, double rate) {
        return Math.pow(Math.abs(input), rate) * ((input > 0) ? 1 : -1);
    }


}
