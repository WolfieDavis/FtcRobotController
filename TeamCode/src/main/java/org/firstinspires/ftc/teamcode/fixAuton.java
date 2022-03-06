package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.ServoX;
import org.firstinspires.ftc.teamcode.api.State;

/*
 * Robohawks ftc team 5741
 * Drive code for driver controlled period
 * contributers: Wolfie Davis, Crawford Phillips, Will Sprigg
 * ruined by: Cailean Sorce
 */

@TeleOp
public class fixAuton extends OpMode {


    private Drivetrain drivetrain;
    private DcMotorX
            spinner,
            linear,
            intake;
    //    private LimitedMotorX linear;
    private ServoX
            outtake,
            tip,
            odoL,
            odoR,
            odoB;

    private double
            outtakeTravelPos = 137.5, //137.5      //125 //120 - the bucket is in this angle when traveling
            outtakeCollectPos = 180, //175         //175 //180 - 178 is too low, 175 is too high - the bucket is in this position when collecting
            outtakeDumpPos = 85;


    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private State.Buttons lastButtons1 = new State.Buttons();
    private State.Dpad lastDpads1 = new State.Dpad();
    private State.Bumpers lastBumpers1 = new State.Bumpers();
    // Using a custom state instead of saving entire gamepad2
    private State.Buttons lastButtons2 = new State.Buttons();
    private State.Dpad lastDpads2 = new State.Dpad();
    private State.Bumpers lastBumpers2 = new State.Bumpers();


    public void init() {
        DcMotorX mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);
        drivetrain.reverse();

//        linear = new LimitedMotorX(hardwareMap.dcMotor.get("linear"), 1607, 34.76625);//13.6875); //1968, 16.25 bigger pulley   // 2900, 16.25 motor for linear rail
        linear = new DcMotorX(hardwareMap.dcMotor.get("linear"));
        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));//motor for intake spinner
        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));//motor for carousel spinner
        outtake = new ServoX(hardwareMap.servo.get("outtake"));//servo for outtake dropper
        tip = new ServoX(hardwareMap.servo.get("tip"));
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));

        drivetrain.setBrake(true);
        drivetrain.stop();

    }// end of init

    public void start() {
        linear.setBrake(true); //so that the outake motor arm will hold pos and won't "bounce"
        linear.controlVelocity();


        /* ------------------------ lift the odometry pods up ----------------------- */
        odoL.setAngle(180);
        odoR.setAngle(178);
        odoB.setAngle(155);
    }


    public void loop() {

        /* ------------- define variables to keep track of the controls ------------- */

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

        //button definitions for gamepad2
        boolean a2 = gamepad2.a;
        boolean b2 = gamepad2.b;
        boolean x2 = gamepad2.x;
        boolean y2 = gamepad2.y;
        boolean dpadUp2 = gamepad2.dpad_up;
        boolean dpadDown2 = gamepad2.dpad_down;
        boolean dpadRight2 = gamepad2.dpad_right;
        boolean dpadLeft2 = gamepad2.dpad_left;
        boolean bumperLeft2 = gamepad2.left_bumper;
        boolean bumperRight2 = gamepad2.right_bumper;
        boolean xHit2 = x2 && !lastButtons2.x;
        boolean yHit2 = y2 && !lastButtons2.y;
        boolean aHit2 = a2 && !lastButtons2.a;
        boolean bHit2 = b2 && !lastButtons2.b;
        boolean dpadUpHit2 = dpadUp2 && !lastDpads1.dpad_up;
        boolean dpadDownHit2 = dpadDown2 && !lastDpads1.dpad_down;
        boolean dpadRightHit2 = dpadRight2 && !lastDpads1.dpad_right;
        boolean dpadLeftHit2 = dpadLeft2 && !lastDpads1.dpad_left;
        boolean bumperLeftHit2 = bumperLeft2 && !lastBumpers2.left_bumper;
        boolean bumperRightHit2 = bumperRight2 && !lastBumpers2.right_bumper;



        /* ------------- move the outake linear slide ( called "linear") ------------ */
        //right and left triggers up and down

        double maxSpeed = 0.5;
        if (gamepad2.right_trigger > 0.01) {
            linear.setVelocity(gamepad2.right_trigger * maxSpeed);    // set the speed of the arm
        } else if (gamepad2.left_trigger > 0.01) {
            linear.setVelocity(-gamepad2.left_trigger * maxSpeed);
        } else {
            linear.setVelocity(0.0);
        }

        /* -------------------------- move the bucket servo ------------------------- */
        //right and left bumpers up and down

        if (y2) {
            outtake.setAngle(outtakeTravelPos);
        }



        //code to try to increment angle with the bumpers
//        if (gamepad2.right_bumper) {
//            outtake.
//        } else if (gamepad2.left_bumper) {
//
//        } else {
//
//        }


        /* ------------------------ odometry pods up and down ----------------------- */

        if (dpadUpHit1) { //up
            odoL.setAngle(180);
            odoR.setAngle(178);
            odoB.setAngle(155);
        } else if (dpadDownHit1) { //down
            odoL.setAngle(0);
            odoR.setAngle(0);
            odoB.setAngle(0);
        }

        /* ------------------------- control the drivetrain ------------------------- */
        // Drive the robot with joysticks if they are moved (with rates)
        if (Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            drivetrain.driveWithGamepad(0.8, rateCurve(-rightY, 1.7), rateCurve(-leftX, 1.7) * 0.625, rateCurve(rightX, 1.7)); //curved stick rates
        } else {
            // If the joysticks are not pressed, do not move the bot
            drivetrain.stop();
        }


        /* ------------- record button states, to be used in determining ------------ */
        /* ------------------- "pressed" vs "held" and "released" ------------------- */
        // Save button states
        lastButtons1.update(a1, b1, x1, y1);
        lastDpads1.update(dpadUp1, dpadDown1, dpadRight1, dpadLeft1);
        lastBumpers1.update(bumperRight1, bumperLeft1);

        lastButtons2.update(a2, b2, x2, y2);
        lastDpads2.update(dpadUp2, dpadDown2, dpadRight2, dpadLeft2);
        lastBumpers2.update(bumperRight2, bumperLeft2);

        /* ------- print to telemetry (used for calibration/ trouble shooting) ------ */

        telemetry.addData("Data:", linear.getPosition());
        // telemetry.addData("Dpad up: ", dpadUp2);

    }//end of loop

    /* ------------------ used to "curve" the joystick input ------------------ */
    private double rateCurve(double input, double rate) {
        return Math.pow(Math.abs(input), rate) * ((input > 0) ? 1 : -1);
    }


}
   /*
    RRRRR           b            hh                                k     k          5555555 77777777 44    44  1111
   R::::::R        b.b           h.h                              k.k   kk          5.5          7.7 4.4  4.4 1 1.1
   R::RR:::R       b.b           h.h                              k.k  k.k  sssss   5.5         7.7  4.4  4.4   1.1
   R::::::R   ooo  b..bbb   ooo  h..hhhh   aaa.a www    ww    www k.k.k.k  s.s      55555      7.7   4..44..4   1.1
   R:R R::R  o.o.o b..b..b o.o.o h......h a..a..a w.w w.ww.w w.w  k.k.k     sssss       5.5   7.7         4.4   1.1
   R:R  R::R o.o.o b..b..b o.o.o h.h  h.h a..a..a  w.w.w  w.w.w   k.k k.k      s.s      5.5  7.7          4.4   1.1
   RRR   RRR  ooo   bbbb    ooo  hhh  hhh  aaa  aa  www    www    kk   k.k ssssss   555555  7.7           444 11111111
  */


