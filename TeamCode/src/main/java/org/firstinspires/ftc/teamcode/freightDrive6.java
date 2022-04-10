package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
public class freightDrive6 extends OpMode {


    private Drivetrain drivetrain;
    private DcMotorX
            spinner,
            intake,
            tapeExtend,
            linear;
    private ServoX
            outtake,
            outtake2,
            odoL,
            odoR,
            odoB,
            tapePan,
            tapeTilt;
    private TouchSensor
            bottom,
            low,
            middle,
            top;

    private double
            //linear slide variables
            linearMaxSpeed = 1,
            linearDownSpeed = 0.75,
            linearPosSpeed = 0.3,

    //outtake servo positions
    outtake2Offset = 15 + 5, //-15 bc slide is 15 deg, and 5 for other adjustment
            outtake2CollectPos = 0 + outtake2Offset, //starting with 0 as bottom
            outtake2TravelPos = 42.5 + outtake2Offset,
            outtake2DumpPos = 95 - 10 + outtake2Offset,
            outtake2DumpPos2 = 95 - 15 + outtake2Offset,

    //capping mechanism (tape measure) positions and variables
    tapeExtendPower = 1, //.85
            tapePanValue = 90,
            tapeTiltValue = 90,
            panMin = 90 - 50, //left
            panMax = 90 + 50, //right
            tiltMin = 90 - 50, //down
            tiltMax = 90 + 40, //up
            tapePanMultiplier = 0.2, //.15
            tapeTiltMultiplier = 0.15; //.15


    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private State.Buttons lastButtons1 = new State.Buttons();
    private State.Dpad lastDpads1 = new State.Dpad();
    private State.Bumpers lastBumpers1 = new State.Bumpers();
    // Using a custom state instead of saving entire gamepad2
    private State.Buttons lastButtons2 = new State.Buttons();
    private State.Dpad lastDpads2 = new State.Dpad();
    private State.Bumpers lastBumpers2 = new State.Bumpers();

    //misc toggle and value change variables
    int spinDirection = 1;
    int intakeToggle = 1;
    int intakeSpinDir = 1;
    boolean isReversed = false;
    String lastLimitHit = null;
    String currentLimitHit = null;


    public void init() {
        DcMotorX mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);
        drivetrain.reverse();

        linear = new DcMotorX(hardwareMap.dcMotor.get("linear"));//motor for linear slide
        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));//motor for intake spinner
        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));//motor for carousel spinner
        tapeExtend = new DcMotorX(hardwareMap.dcMotor.get("odoRear"));//encoder for rear odometry pod, and motor for capping mechanism

        outtake = new ServoX(hardwareMap.servo.get("outtake"));//servo for outtake dropper
        outtake2 = new ServoX(hardwareMap.servo.get("outtake2"));//second servo for outtake dropper
        tapePan = new ServoX(hardwareMap.servo.get("tapePan"), 180, panMax, panMin);
        tapeTilt = new ServoX(hardwareMap.servo.get("tapeTilt"), 180, tiltMax, tiltMin);
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));

        bottom = hardwareMap.touchSensor.get("bottom");
        low = hardwareMap.touchSensor.get("low");
        middle = hardwareMap.touchSensor.get("middle");
        top = hardwareMap.touchSensor.get("top");

        drivetrain.setBrake(true);
        drivetrain.stop(); //stops odometry instance... I think

    }// end of init


    public void start() {
        drivetrain.stop(); //stops odometry instance again to make sure

        linear.setBrake(true); //so that the outake motor arm will hold pos and won't "bounce"
//        linear.controlVelocity();

        //lift odometry pods
        odoL.setAngle(155);
        odoR.setAngle(125);
        odoB.setAngle(155);

        //center capping mechanism
        tapePan.setAngle(90);
        tapeTilt.setAngle(90);

        //initialize bucket servos
//        outtake.setAngle(outtake2TravelPos);
//        outtake2.setAngle(outtake2TravelPos);
//        tip.setAngle(87);
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


        /* ----------------- logic for how high the bucket is ---------------- */
        if (!bottom.isPressed()) currentLimitHit = "bottom";
        else if (!low.isPressed()) currentLimitHit = "low";
        else if (!middle.isPressed()) currentLimitHit = "middle";
        else if (!top.isPressed()) currentLimitHit = "top";

        if (bottom.isPressed() && (currentLimitHit == "bottom")) lastLimitHit = "bottom";
        else if (low.isPressed() && (currentLimitHit == "low")) lastLimitHit = "low";
        else if (middle.isPressed() && (currentLimitHit == "middle")) lastLimitHit = "middle";
        else if (top.isPressed() && (currentLimitHit == "top")) lastLimitHit = "top";


        /* ------------- move the outake linear slide ( called "linear") ------------ */
        // first check if the triggers have been pressed (for manual movement). if they have been and the arm is not at the end of its travel, move the arm at the speed indicated by the trigger.
        if (gamepad2.right_trigger > 0.01 && top.isPressed()) {
            if (lastLimitHit != "middle")
                linear.setPower(gamepad2.right_trigger * linearMaxSpeed);    // set the speed of the arm
            else linear.setPower(gamepad2.right_trigger * linearMaxSpeed * 0.8);
        } else if (gamepad2.left_trigger > 0.01 && bottom.isPressed()) {
            if (lastLimitHit != "low") linear.setPower(-gamepad2.left_trigger * linearDownSpeed);
            else linear.setPower(-gamepad2.left_trigger * Math.pow(linearDownSpeed, 2));
        } else {
            //check the dpad for automatic movement requests, and record the position requested in the linearGoToPos variable. recording the requested position like this means the driver doesn't have to keep the dpad depressed until the movement is finished, they can just press and release it.
            if (dpadUpHit2 && top.isPressed() && low.isPressed()) {
                linear.setPower(linearPosSpeed);
            } else if (dpadDownHit2 && middle.isPressed() && top.isPressed() && low.isPressed()) {
                linear.setPower(linearPosSpeed);
            } else if (dpadLeftHit2 && low.isPressed() && top.isPressed()) {
                linear.setPower(linearPosSpeed);
            } else if (dpadRightHit2 && bottom.isPressed() && top.isPressed() && low.isPressed()) {
                linear.setPower(-linearPosSpeed);
            } else {
                linear.setVelocity(0.0);
            }
        }

//        int topPressed;
//        if (top.isPressed()) topPressed = 1;
//        else topPressed = 0;
//        telemetry.addData("top", topPressed);
//        telemetry.update();


        /* ------------ set the 2 servo bucket position w/ limit logic ----------- */
        if (y2) {
            if (top.isPressed()) {
                linear.setPower(0);
                outtake.setAngle(outtake2DumpPos);
                outtake2.setAngle(outtake2DumpPos);
            } else {
                linear.setPower(0.8);
                outtake.setAngle(outtake2TravelPos);
                outtake2.setAngle(outtake2TravelPos);
            }
        } else if (x2) {
            if (top.isPressed()) {
                linear.setPower(0);
                outtake.setAngle(outtake2DumpPos2);
                outtake2.setAngle(outtake2DumpPos2);
            } else {
                linear.setPower(0.8);
                outtake.setAngle(outtake2TravelPos);
                outtake2.setAngle(outtake2TravelPos);
            }
        } else if (a2) {
            outtake.setAngle(outtake2DumpPos);
            outtake2.setAngle(outtake2DumpPos);
        } else if (!gamepad2.y && bottom.isPressed()) {
            outtake.setAngle(outtake2TravelPos);
            outtake2.setAngle(outtake2TravelPos);
        } else {
            outtake.setAngle(outtake2CollectPos);
            outtake2.setAngle(outtake2CollectPos);
        }


        /* ------------ capping mechanism (tape measure) pan/tilt/extend ----------- */
        if (Math.abs(gamepad2.left_stick_y) > .1) {
            tapeExtend.setPower(gamepad2.left_stick_y * tapeExtendPower);
        }
        else tapeExtend.setPower(0);

        if (Math.abs(gamepad2.right_stick_x) > .1 || Math.abs(gamepad2.right_stick_y) > .1) {
            tapePanValue += gamepad2.right_stick_x * tapePanMultiplier;
            tapeTiltValue += gamepad2.right_stick_y * -tapeTiltMultiplier;
        }

        tapePan.setAngle(tapePanValue);
        tapeTilt.setAngle(tapeTiltValue);


        /* ------------------------ odometry pods up and down ----------------------- */
        if (dpadUpHit1) { //up
            odoL.setAngle(155);
            odoR.setAngle(125);
            odoB.setAngle(155);
        } else if (dpadDownHit1) { //down
            odoL.setAngle(0);
            odoR.setAngle(0);
            odoB.setAngle(0);
        }

        /* -------------- set the intake spinner direction / on / off -------------- */
        intakeSpinDir = (bumperRightHit1 || bumperRightHit2) ? intakeSpinDir *= -1 : intakeSpinDir;//toggles intake direction
        //intake spinner is toggled if b is pressed
        if (bHit1 || bHit2) {
            intakeToggle *= -1;
            switch (intakeToggle) {
                case -1:
                    intake.setPower(0.7 * -intakeSpinDir);
                    break;
                case 1:
                    intake.setPower(0.0);
                    break;
            }//end of switch case
        }


        /* -------------- set the carousel spinner direction / on / off ------------- */
        spinDirection = (bumperLeftHit1 || bumperLeftHit2) ? spinDirection *= -1 : spinDirection; //reverse the direction if left bumper  is pressed
        //carousel spinner triggered w/ a press
        if (a1) {
            spinner.setPower(-0.8 * spinDirection);
        } else {
            spinner.setPower(0);
        }


        /* ------------------------- control the drivetrain ------------------------- */
        // Drive the robot with joysticks if they are moved (with rates)
        if (Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            double multiplier = (isReversed) ? -1 : 1;
            drivetrain.driveWithGamepad(0.9, rateCurve(-rightY, 1.7), rateCurve(-leftX, 1.7) * multiplier * 0.7 / 0.9, rateCurve(rightX, 1.7)); //curved stick rates
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

        /* ---------------------data to read out on phone screen -------------------- */
        telemetry.addData("last hit", lastLimitHit);
        telemetry.addData("linear power", linear.getPower());
        telemetry.addData("pan", tapePanValue);
        telemetry.addData("tilt", tapeTiltValue);
        telemetry.update();

    }//end of loop


    /* ------------------ used to "curve" the joystick input ------------------ */
    private double rateCurve(double input, double rate) {
        return Math.pow(Math.abs(input), rate) * ((input > 0) ? 1 : -1);
    }


    /* ---------- used to slow a motor down when approching target pos ---------- */
    /* ------------- returns (distance left to travel)^(1/adjuster) ------------- */
    private double fakePid(DcMotorX motor, double startPos, double targetPos, double speed, /*double exponent, /*double adjuster,*/ double stopTolerance) {
        double currentPos = motor.getPosition();
        double totalMoveDist = (targetPos - startPos);
        double distanceToMove = Math.abs(targetPos - currentPos);

        double exponent;
        if (totalMoveDist < 5) exponent = 2;
        else if (totalMoveDist < 15) exponent = 4;
        else if (totalMoveDist < 25) exponent = 6;
        else exponent = 6;

        double negAdjust;
        if ((totalMoveDist > 0) && ((currentPos - startPos) < 2)) negAdjust = (totalMoveDist - 1);
        else negAdjust = 0;

        double distanceRemaining = Math.abs(targetPos - currentPos);
        if (distanceRemaining > stopTolerance) {
//            return (-(Math.pow((((currentPos) - (totalMoveDist / 2) + negAdjust) / (totalMoveDist / 2)), (exponent))) + 1) * speed;
//            return Math.pow(distanceToMove, speed / adjuster) * (currentPos < targetPos ? 1 : -1);
            return Math.pow(distanceToMove, speed / 50) * (currentPos < targetPos ? 1 : -1);
        } else {
            return 0.0;
        }
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


