package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;

import java.util.Arrays;

@Autonomous
public class BlueCloseAdapt extends LinearOpMode {

    int side = -1; //modifier for side: set to 1 for red, or -1 for blue

    // Odometry parameters
    private int ticksPerRev = 8225; //left same as last year
    private double circumference = 15.725; //left same as last year
    private double width = 26.7385; //distance between centers of odometry wheels
    private double backDistancePerRadian = 22.222; //compensates for the wheel being in the back of the bot

    private Drivetrain drivetrain;

    private DcMotorX
            mRF,
            mLF,
            mRB,
            mLB,
            spinner,
            intake,
            wheelR,
            wheelL,
            wheelB,
            linear;
    private ServoX
            outtake,
            tip,
            odoL,
            odoR,
            odoB;
    DistanceSensor detectBlue;
    DistanceSensor detectRed;
    private TouchSensor
            bottom,
            low,
            middle,
            top;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get all of the drivetrain motors
        mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF"));
        mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF"));
        mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB"));
        mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        //set up other motors
        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));
        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));
        linear = new DcMotorX(hardwareMap.dcMotor.get("linear"));
        outtake = new ServoX(hardwareMap.servo.get("outtake"));

        //servos to raise and lower the odometry pods
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));

        //sensors and limit switches
        detectBlue = hardwareMap.get(DistanceSensor.class, "detectBlue");
        detectRed = hardwareMap.get(DistanceSensor.class, "detectRed");
        bottom = hardwareMap.touchSensor.get("bottom");
        low = hardwareMap.touchSensor.get("low");
        middle = hardwareMap.touchSensor.get("middle");
        top = hardwareMap.touchSensor.get("top");

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("odoRear"), ticksPerRev, (circumference));
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference));
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, -(circumference));

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, 0, 0, 0);

        // sets up drivetrain
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        //sets initial position for the drivetrain
        double[] initialPos = {17 * side, -95.7, 0}; //x, y, phi
        positionTracker.x = initialPos[0];
        positionTracker.y = -initialPos[1];
        positionTracker.phi = initialPos[2];

        //sets up threading for odometry
        Thread positionTracking = new Thread(positionTracker);
        positionTracking.start();

        //displays on the phone that everything has initialized correctly
        telemetry.addData("Done initializing", "");
        telemetry.update();

        /* ----------- waiting for start ----------- */
        waitForStart();

        /* ------------ setup movement ------------ */
        //movement parameters
        double exponent = 4; //4 //exponent that the rate curve is raised to
        double[] speed = {0.45, 0.35, 0.35}; //todo: fix this... or leave it??
        double[] detectSpeed = {0.35, 0.2, 0.35};
        double[] stopTolerance = {4, (Math.PI / 45)}; //4 //acceptable tolerance (cm for linear, radians for turning) for the robot to be in a position
        double[] drivePower;

        //positions: in the format x, y, phi. (in cm for x and y and radians for phi) this can be declared at the top of the program
        double[] carousel = {(32.5 -5) * side, -20.75, 0}; //todo: fine tune this
        double[] ash = {118.5 * side, -104, 0}; //-102 for y
        double[] asuPark = {91.75 * side, -25, 0}; //89, -25, 0
        double[] detect2 = {53 * side, -91 - 1, 0}; //68.5 too far //location for detecting the top placement
        double[] detect1 = {53 * side, -68.5, 0}; //location for detecting the middle location

        //outtake (linear) variables
        double maxLinearPower = 0.3;

        //outtake (servo) positions
        double outtakeTravelPos = 137.5; //servo position for travel
        double outtakeDumpPos = 85; //servo position for dump
        double outtakeCollectPos = 180;

        /* --------------- move robot --------------- */
        //retract odometry pods and set buket to travel position
        outtake.setAngle(outtakeTravelPos);
        odoL.setAngle(0);
        odoR.setAngle(0);
        odoB.goToAngle(0, 2000);

        //go to detect location
        do {
            drivePower = fakePid_DrivingEdition(initialPos, detect2, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
        sleep(750);

        //detect and set target zone (based on red or blue)
        int levelTarget;
        double detectZone[];

        if (side == -1) { //if side is blue - 0,1,2 positions = low, middle, top
            double distance = detectRed.getDistance(DistanceUnit.CM);

            if (distance < 20) {
                levelTarget = 0;
                detectZone = detect2;
            } else {
                do {
                    drivePower = fakePid_DrivingEdition(detect2, detect1, positionTracker, detectSpeed, exponent, stopTolerance);
                    drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
                } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
                sleep(750);

                distance = detectBlue.getDistance(DistanceUnit.CM);
                if (distance < 20) {
                    levelTarget = 1;
                    detectZone = detect2;
                } else {
                    levelTarget = 1;
                    detectZone = detect1;
                }
            }
            if (levelTarget == 0) telemetry.addData("level", "low");
            else if (levelTarget == 1) telemetry.addData("level", "middle");
            else telemetry.addData("level", "top");
            telemetry.update();

        } else {//if side is blue - 0,1,2 positions = top, middle, low
            double distance = detectBlue.getDistance(DistanceUnit.CM);

            if (distance < 20) {
                levelTarget = 2;
                detectZone = detect2;
            } else {
                do {
                    drivePower = fakePid_DrivingEdition(detect2, detect1, positionTracker, detectSpeed, exponent, stopTolerance);
                    drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
                } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
                sleep(750);

                distance = detectBlue.getDistance(DistanceUnit.CM);
                if (distance < 20) {
                    levelTarget = 1;
                    detectZone = detect2;
                } else {
                    levelTarget = 0;
                    detectZone = detect1;
                }
            }
            if (levelTarget == 0) telemetry.addData("level", "low");
            else if (levelTarget == 1) telemetry.addData("level", "middle");
            else telemetry.addData("level", "top");
            telemetry.update();
        }
        sleep(500);


        //go to ash
        long startASH = System.currentTimeMillis();
        long timeOutASH = 2500;
        do {
            drivePower = fakePid_DrivingEdition(detectZone, ash, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startASH) < timeOutASH));
        sleep(500);


        //raise and dump
        long startDump = System.currentTimeMillis();
        long timeOutDump = 5000;

        if (levelTarget == 0) {
            do linear.setPower(maxLinearPower);
            while (!low.isPressed() && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));
            linear.setPower(0);
        } else if (levelTarget == 1) {
            do linear.setPower(maxLinearPower);
            while (!middle.isPressed() && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));
            linear.setPower(0);
        } else {
            do linear.setPower(maxLinearPower);
            while (!top.isPressed() && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));
            linear.setPower(0);
        }
        sleep(250);
        outtake.goToAngle(outtakeDumpPos, 1500);

        sleep(500);


        //go to carousel
        long startCarousel = System.currentTimeMillis();
        long timeOutCarousel = 4500;
        do {
            drivePower = fakePid_DrivingEdition(ash, carousel, positionTracker, speed, 6, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startCarousel) < timeOutCarousel));
        sleep(250);


        //spin carousel
        spin(spinner, -0.5 * side, 6000); //turns on carousel spinner at power 0.5 for 500ms (or whatever you set them to)


        //park in asu
        long startPark = System.currentTimeMillis();
        long timeOutPark = 2750;
        do {
            drivePower = fakePid_DrivingEdition(carousel, asuPark, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startPark) < timeOutPark));
        sleep(250);


        //lower slide to be ready to start teleop
        outtake.goToAngle(outtakeTravelPos, 750);
        do {
            linear.setPower(-maxLinearPower);
        } while (!bottom.isPressed() && !isStopRequested());
        sleep(500);


        /* ---------------- shut down ---------------- */
        drivetrain.setBrake(true);
        drivetrain.stop();
        positionTracker.stop();
    }//end of runOpMode


    /* ----------- backend of drive code: fake pid ----------- */
    private double[] fakePid_DrivingEdition(double[] startPos, double[] targetPos, Odometry odo, double[] speed, double exponent, double[] stopTolerance) {
        double[] distBetween = {targetPos[0] - startPos[0], targetPos[1] - startPos[1], targetPos[2] - startPos[2]};
        double totDistBetween = Math.sqrt(Math.pow(distBetween[0], 2) + Math.pow(distBetween[1], 2));
        double[] distanceToMoveRemaining = {targetPos[0] - odo.x, targetPos[1] - (-odo.y), targetPos[2] - odo.phi};
        double totalDistanceRemaining = Math.sqrt(Math.pow(distanceToMoveRemaining[0], 2) + Math.pow(distanceToMoveRemaining[1], 2));

        double[] returnPowers = {0, 0, 0};
        if (totalDistanceRemaining > stopTolerance[0]) {
            double[] powerFractions = {distanceToMoveRemaining[0] / totalDistanceRemaining, distanceToMoveRemaining[1] / totalDistanceRemaining};
            double fakePidAdjustment = (-(Math.pow((((totalDistanceRemaining) - (totDistBetween / 1.5)) / (totDistBetween / 1.5)), (exponent))) + 1); //curved as it starts and ends - experimental
            returnPowers[0] = powerFractions[0] * fakePidAdjustment * speed[0];
            returnPowers[1] = powerFractions[1] * fakePidAdjustment * speed[1];
        }
        double totalTurnDistance = Math.abs(distanceToMoveRemaining[2]);
        if (totalTurnDistance > stopTolerance[1]) {
            returnPowers[2] = speed[2] * (distanceToMoveRemaining[2] >= 0 ? 1 : -1);
        }

//        //read out current and target positions for debugging
//        telemetry.addData("x current", odo.x);
//        telemetry.addData("y current", -odo.y);
//        telemetry.addData("phi current (deg)", odo.phi * 180 / Math.PI);
//        telemetry.addData("x target", targetPos[0]);
//        telemetry.addData("y target", targetPos[1]);
//        telemetry.addData("phi target (deg)", targetPos[2]);
//        telemetry.update();

        return returnPowers;
    }

    /* ----------------- spins a motor ----------------- */
    private void spin(DcMotorX name, double power, int waitTime) {
        name.setPower(power);
        sleep(waitTime);
        name.setPower(0);
    }


}//end of linear op mode