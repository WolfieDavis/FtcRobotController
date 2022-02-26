package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.ServoX;
import org.firstinspires.ftc.teamcode.api.State;
import org.firstinspires.ftc.teamcode.api.Odometry;

/*
 * Robohawks ftc team 5741
 * Drive code for driver controlled period
 * contributers: Wolfie Davis, Crawford Phillips, Will Sprigg
 * ruined by: Cailean Sorce
 */

@TeleOp
public class odometryTest extends OpMode {

    DistanceSensor detectRed;
//    LynxI2cColorRangeSensor detectL;

    private ControlledDrivetrain drivetrain;
    private ServoX
            odoL,
            odoR,
            odoB;

    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private State.Buttons lastButtons1 = new State.Buttons();
    private State.Dpad lastDpads1 = new State.Dpad();


    // Odometry parameters
    private int ticksPerRev = 8225;
    private double circumference = 15.725;
    


    public void init() {
        DcMotorX
                mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        //servos to raise and lower the odometry pods
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));

        //odometry initialization code
        Odometry positionTracker = new Odometry(
                new DcMotorX(hardwareMap.dcMotor.get("odoR"), ticksPerRev, (circumference)), //right pod
                new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference)), //left pod
                new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, -(circumference)), //back pod
                50,
                22.222, //170.556/ (2*Math.PI), //-169.076 //-6.41, //-120.63  //-2.25 / (2 * Math.PI), //2.25 2.3 // old: -41.577 / (2 * Math.PI)  //6.15 cm from the middle for old   //19.2 cm from the middle for new
                26.7385, //cm between side odometry wheels
                0, //set to 0 as in auto from last year - in documentation they were set to 5
                0,
                0
        );
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        //drivetrain.reverse();
        drivetrain.telemetry = telemetry;        // Adding logging to drivetrain (only needed for development)
        drivetrain.setActive(false);        // Start with the drivetrain off
        Thread positionTracking = new Thread(drivetrain);
        positionTracking.start();

    }// end of init

    public void start() {

    }

    public void loop() {

        /* ------------- define variables to keep track of the controls ------------- */

        detectRed = hardwareMap.get(DistanceSensor.class, "detectRed");
//        detectL = hardwareMap.get(LynxI2cColorRangeSensor.class, "detectL");

        //joystick values for driving.
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
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
        boolean xHit1 = x1 && !lastButtons1.x;
        boolean yHit1 = y1 && !lastButtons1.y;
        boolean aHit1 = a1 && !lastButtons1.a;
        boolean bHit1 = b1 && !lastButtons1.b;
        boolean dpadUpHit1 = dpadUp1 && !lastDpads1.dpad_up;
        boolean dpadDownHit1 = dpadDown1 && !lastDpads1.dpad_down;
        boolean dpadRightHit1 = dpadRight1 && !lastDpads1.dpad_right;
        boolean dpadLeftHit1 = dpadLeft1 && !lastDpads1.dpad_left;


        /* ------------------------- odometry pods up and down test ------------------------ */

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
            drivetrain.driveWithGamepad(1, rateCurve(rightY, 1.7) * 0.3, rateCurve(leftX, 1.7) * 0.35, rateCurve(rightX, 1.7) * 0.3); //curved stick rates
        } else {
            // If the joysticks are not pressed, do not move the bot
            drivetrain.stop();
        }

        if(gamepad1.right_stick_button){
            // Reset odometry to prevent error buildup
            drivetrain.positionTracker.reset(0, 0, 0);
            //drivetrain.positionTracker.wheelB.resetEncoder();
        }


        /* ------------- record button states, to be used in determining ------------ */
        /* ------------------- "pressed" vs "held" and "released" ------------------- */
        // Save button states
        lastButtons1.update(a1, b1, x1, y1);
        lastDpads1.update(dpadUp1, dpadDown1, dpadRight1, dpadLeft1);


        /* ------- print to telemetry (used for calibration/ trouble shooting) ------ */
        telemetry.addData("x", drivetrain.positionTracker.x); // Get the robot's current x coordinate
        telemetry.addData("y", drivetrain.positionTracker.y); // Get the robot's current y coordinate
        telemetry.addData("heading", (drivetrain.positionTracker.phi)); // Get the robot's current heading
        telemetry.addData("degrees", (drivetrain.positionTracker.phi*180/Math.PI));
        telemetry.addData("rear cm", ((drivetrain.positionTracker.phi)*26.9)); // Get the robot's current heading
        telemetry.addData("rear", drivetrain.positionTracker.wheelB.getPosition());
//        telemetry.addData("tics of rear:", drivetrain.mLB.getPosition());
//        telemetry.addData("rear cm tics:", drivetrain.mLB.getPosition()*circumference);

        telemetry.addData("distance", detectRed.getDistance(DistanceUnit.CM));
//        telemetry.addData("distance", detectL.getDistance(DistanceUnit.CM));
    }//end of loop

    /* ------------------ used to "curve" the joystick input ------------------ */
    private double rateCurve(double input, double rate) {
        return Math.pow(Math.abs(input), rate) * ((input > 0) ? 1 : -1);
    }

    @Override
    public void stop() {
        drivetrain.setActive(false);
        drivetrain.stop();
        drivetrain.stopController();
    }

}