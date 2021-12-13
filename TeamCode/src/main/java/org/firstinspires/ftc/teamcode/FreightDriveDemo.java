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

/*
*   This is similar to the freight drive program but all operations have been
*   moved to controller one so only one driver is needed to demo the bot.
*   Please update this before each demo with all current mechanisms
*   that have been added to the bot.
*/

@TeleOp
public class FreightDriveDemo extends OpMode {


    private Drivetrain drivetrain;
    private DcMotorX
            spinner,
            linear,
            intake;
    private ServoX
            outtake;

    private double power = 1;
    private TouchSensor spinLimit;
    //limit switch is named spinLimit

    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private State.Buttons lastButtons1 = new State.Buttons();
    private State.Dpad lastDpads1 = new State.Dpad();
    private State.Bumpers lastBumpers1 = new State.Bumpers();

    //misc toggle / value change variables
    int spinDirection = 1;
    int intakeToggle = 1;
    int outtakeToggle = 1;

    public void init(){
        DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        linear = new DcMotorX(hardwareMap.dcMotor.get("linear"));//motor for linear rail
        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));//motor for intake spinner
        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));//motor for carousel spinner

        outtake = new ServoX(hardwareMap.servo.get("outtake"));//servo for outtake dropper

    }// end of init

    public void loop(){

        //joystick values for driving.
        double leftX = gamepad1.left_stick_x;
        double rightX = -gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller

        //button definitions for gamepad1
        boolean a1 = gamepad1.a;
        boolean b1 = gamepad1.b;
        boolean x1 = gamepad1.x;
        boolean y1 = gamepad1.y;
        boolean dpadUp1 = gamepad1.dpad_up;
        boolean dpadDown1 = gamepad1.dpad_down;
        boolean dpadRight1 = gamepad1.dpad_right;
        boolean dpadLeft1 = gamepad1.dpad_left;
        boolean bumperLeft1 = gamepad1.left_bumper;
        boolean bumperRight1 = gamepad1.right_bumper;
        boolean xHit1 = x1 && !lastButtons1.x;
        boolean yHit1 = y1 && !lastButtons1.y;
        boolean aHit1 = a1 && !lastButtons1.a;
        boolean bHit1 = b1 && !lastButtons1.b;
        boolean dpadUpHit1 = dpadUp1 && !lastDpads1.dpad_up;
        boolean dpadDownHit1 = dpadDown1 && !lastDpads1.dpad_down;
        boolean dpadRightHit1 = dpadRight1 && !lastDpads1.dpad_right;
        boolean dpadLeftHit1 = dpadLeft1 && !lastDpads1.dpad_left;
        boolean bumperLeftHit1 = bumperLeft1 && !lastBumpers1.left_bumper;
        boolean bumperRightHit1 = bumperRight1 && !lastBumpers1.right_bumper;


        //outtake code
        if (y1){//flip the outake
            outtake.setAngle(45);
        } else if (x1){//unflips the outake
            outtake.setAngle(175);
        }


        //code for the linear rail uses the values read by the trigger.
        if (gamepad1.right_trigger > 0.01){ //raises the linear slide
            linear.setPower(-gamepad1.right_trigger);
            outtake.setAngle(165);//raises outtake to hold freight
        } else if (gamepad1.left_trigger > 0.01){ //lowers linear slide
            linear.setPower(gamepad1.left_trigger);
            outtake.setAngle(180);//drops outtake down to collect freight
        } else {
            linear.setPower(0.0);
        }

        //intake spinner is toggled if b is pressed
        if (bHit1){
            intakeToggle *= -1;
            switch (intakeToggle){
                case -1 :
                    intake.setPower(0.8);
                    break;
                case 1 :
                    intake.setPower(0.0);
                    break;
            }//end of switch case
        }

        spinDirection = (bumperLeftHit1)? spinDirection *= -1: spinDirection; //reverse the direction if left bumper  is pressed
        //carousel spinner triggered w/ a press
        if(a1){
            spinner.setPower(0.8 * spinDirection);
        } else {
            spinner.setPower(0);
        }


        // Drive the robot with joysticks if they are moved (with rates)
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            drivetrain.driveWithGamepad(1, rateCurve(rightY, 1.7),rateCurve(leftX, 1.7)/* 0.5*leftX */, rateCurve(rightX,1.7)); //curved stick rates
        }else{
            // If the joysticks are not pressed, do not move the bot
            drivetrain.stop();
        }

        // Save button states
        lastButtons1.update(a1, b1, x1, y1);
        lastDpads1.update(dpadUp1, dpadDown1, dpadRight1, dpadLeft1);
        lastBumpers1.update(bumperRight1, bumperLeft1);


    }//end of loop

    private double rateCurve(double input, double rate){
        return Math.pow(Math.abs(input),rate)*((input>0)?1:-1);
    }


}
