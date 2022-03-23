package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;

import java.util.Arrays;

@Autonomous
public class FreightAutonBlueFar extends LinearOpMode {

    int side = -1; //modifier for x coordinates: set to 1 for red, or -1 for blue
    //todo: also remember to swap distance sensor to detectBlue for blue or detectRed for red

    // Odometry parameters
    private int ticksPerRev = 8225; //left same as last year
    private double circumference = 15.725; //left same as last year
    private double width = 26.7385; //distance between centers of odometry wheels
    private double backDistancePerRadian = 22.222; //compensates for the wheel being in the back of the bot
//    private final double TILE_SIZE = 60.96; //NO we're not measuring in fractional tiles this year, SAE is enough as it is

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
            wheelB;
    private LimitedMotorX linear;
    private ServoX
            outtake,
            tip,
            odoL,
            odoR,
            odoB;
//    DistanceSensor detectRed;
    DistanceSensor detectBlue;

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

        linear = new LimitedMotorX(hardwareMap.dcMotor.get("linear"), 1607, 34.76625);

        outtake = new ServoX(hardwareMap.servo.get("outtake"));
        //servos to raise and lower the odometry pods
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));

//        detectRed = hardwareMap.get(DistanceSensor.class, "detectRed");
        detectBlue = hardwareMap.get(DistanceSensor.class, "detectBlue");

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("odoRear"), ticksPerRev, (circumference));
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference));
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, -(circumference));

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, 0, 0, 0);

        // sets up drivetrain
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        //sets initial position for the drivetrain
        double[] initialPos = {17 * side, -201.5, 0}; //x, y, phi
        positionTracker.x = initialPos[0];
        positionTracker.y = -initialPos[1];
        positionTracker.phi = initialPos[2];

        //sets up threading for odometry
        Thread positionTracking = new Thread(positionTracker);
        positionTracking.start();

        telemetry.addData("Done initializing", "");
        telemetry.update();

        /* ----------- waiting for start ----------- */
        waitForStart();

        /* ------------ setup movement ------------ */
        //movement parameters
        double exponent = 4; //4 //exponent that the rate curve is raised to
        double[] speed = {0.4, 0.3, 0.35}; //x, y, phi //.35    //first argument(number) is for straight line movement, second is for turning
        double[] speedSpin = {0.4, 0.3, 0.3};
        double[] detectSpeed = {0.35, 0.2, 0.35};
        double[] stopTolerance = {4, (Math.PI / 45)}; //4 //acceptable tolerance (cm for linear, radians for turning) for the robot to be in a position

        //just needs to be here
        double[] drivePower;

        //positions: in the format x, y, phi. (in cm for x and y and radians for phi) this can be declared at the top of the program
        double[] startOffset = {0, -47.25 * 2.54, 0};

        double[] carousel = {32.5 * side, -23, 0};
        double[] asuPark = {91.75 * side, -25, 0}; //89, -25, 0
        double[] detect2 = {53 * side, -91 + startOffset[1], 0}; //68.5 too far //location for detecting the top placement
//        double[] detect1 = {53 * side, -68.5 + startOffset[1], 0}; //location for detecting the middle location
        double[] detect1 = {53 * side, -188.515, 0}; //location for detecting the middle location

//        double[] ashSpin = {ashStage[0], ashStage[1], -Math.PI};
//        double[] ashSpin = {/*detect1[0]*/ 53 * side, /*detect1[1]*/-188.515, Math.PI/2};
        double[] ashSpin = {/*detect1[0]*/ 53 * side, /*detect1[1]*/-188.515, Math.PI/4};
        double[] ashSpin2 = {/*detect1[0]*/ 53 * side, /*detect1[1]*/-188.515, Math.PI/2};

        //after reset odometry
        double[] ashAfterSpin = {ashSpin[1], ashSpin[0], 0};
        double[] ashNormal = {118.5 * side, -195, Math.PI}; //-102 for y
        double[] ashAdjusted = {(-65.5-9) * side, 15, 0};  //6.485
        double[] ashStage = {0 * side, ashAdjusted[1]+5, 0};
        double[] warehouseStage = {0 * side, 10, 0};
        double[] approachWall = {29 * side, 0, 0}; //33 //17
        double[] warehouse = {33 * side, 86.485, 0};

//        double[] ashAdjusted = {-(ashNormal[1]-detect1[1]), -(ashNormal[0]-detect1[0]), 0};
//        double[] approachWall = {-(-20-detect1[1]), -ashAfterSpin[1], 0}; //17
//        double[] warehouse = {approachWall[0], (-275-detect1[0]), 0};


        //outtake (linear) positions
        double[] dumpLevel = {3.175, 16.51, 34.625}; // equivalent in inches: {1.25, 6.5, 13.6875}; //low (3), med(8), high(13.6875)
        double minLinearPos = 0.9525; //0.375in //the btm position of the outake (how far down it will go)
        double bottomLinearPos = minLinearPos; //0.9
        double linearMaxSpeed = 0.7;

        //outtake (servo) positions
        double outtakeTravelPos = 137.5; //servo position for travel
        double outtakeDumpPos = 85; //servo position for dump
        double outtakeCollectPos = 180;

        /* --------------- move robot --------------- */
        //tilt bucket up and drop odometry pods
//        outtake.goToAngle(outtakeTravelPos, 500);
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

        //detect freight TODO: write more code and make this actually work
        int levelTarget;
        double distance = detectBlue.getDistance(DistanceUnit.CM);
//        double distance = detectRed.getDistance(DistanceUnit.CM);
        double detectZone[];

        if (distance < 20) {
            levelTarget = 2;
            detectZone = detect2;

            do {
                drivePower = fakePid_DrivingEdition(detect2, detect1, positionTracker, detectSpeed, exponent, stopTolerance);
                drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
            } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));

        } else {
            do {
                drivePower = fakePid_DrivingEdition(detect2, detect1, positionTracker, detectSpeed, exponent, stopTolerance);
                drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
            } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
            sleep(750);

            distance = detectBlue.getDistance(DistanceUnit.CM);
//            distance = detectRed.getDistance(DistanceUnit.CM);
            if (distance < 20) {
                levelTarget = 1;
                detectZone = detect2;
            } else {
                levelTarget = 0;
                detectZone = detect1;
            }
        }

        if (levelTarget == 0) {
            telemetry.addData("level", "low");
        } else if (levelTarget == 1) {
            telemetry.addData("level", "middle");
        } else {
            telemetry.addData("level", "top");
        }
//        telemetry.addData("level", levelTarget);
        telemetry.update();

        sleep(1000);

        //spin 180
        long startAshSpin = System.currentTimeMillis();
        long timeOutAshSpin = 2000;
        do {
            drivePower = fakePid_DrivingEdition(detect1, ashSpin, positionTracker, speedSpin, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);

            telemetry.addData("x", drivePower[0]);
            telemetry.addData("y", drivePower[1]);
            telemetry.addData("phi", drivePower[2]);
            telemetry.update();
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startAshSpin) < timeOutAshSpin));
        sleep(1000);

//        drivetrain.stop();
//        long startAshSpin = System.currentTimeMillis();
//        long timeOutAshSpin = 100000;
//        do {
//            telemetry.addData("x current", positionTracker.x);
//            telemetry.addData("y current", -positionTracker.y);
//            telemetry.addData("phi current (deg)", positionTracker.phi * 180 / Math.PI);
//            telemetry.update();
//        } while (!isStopRequested() && ((System.currentTimeMillis() - startAshSpin) < timeOutAshSpin));
//        sleep(250);

        long startAshSpin2 = System.currentTimeMillis();
        long timeOutAshSpin2 = 2000;
        do {
            drivePower = fakePid_DrivingEdition(ashSpin, ashSpin2, positionTracker, speedSpin, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);

            telemetry.addData("x", drivePower[0]);
            telemetry.addData("y", drivePower[1]);
            telemetry.addData("phi", drivePower[2]);
            telemetry.update();
//            sleep(50);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startAshSpin2) < timeOutAshSpin2));
        sleep(500);

        long startAshSpin3 = System.currentTimeMillis();
        long timeOutAshSpin3 = 100000;
        do {
            telemetry.addData("x current", positionTracker.x);
            telemetry.addData("y current", -positionTracker.y);
            telemetry.addData("phi current (deg)", positionTracker.phi * 180 / Math.PI);
            telemetry.update();
        } while (!isStopRequested() && ((System.currentTimeMillis() - startAshSpin3) < timeOutAshSpin3));
        sleep(250);
        //
//
//        //reset odometry
//        positionTracker.x = 0;
//        positionTracker.y = 0;
//        positionTracker.phi = 0;
//        sleep(50);
//
//        //ash stage
//        long startAshStage = System.currentTimeMillis();
//        long timeOutAshStage = 1000;
//        do {
//            drivePower = fakePid_DrivingEdition(ashAfterSpin, ashStage, positionTracker, speed, 8, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startAshStage) < timeOutAshStage));
//        sleep(250);
//
//        //approach ash
//        long startASH = System.currentTimeMillis();
//        long timeOutASH = 2500;
//        do {
//            drivePower = fakePid_DrivingEdition(ashStage, ashAdjusted, positionTracker, speed, 8, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startASH) < timeOutASH));
//        sleep(500);
//
//        //raise and dump
//        do {
//            if (levelTarget == 2) {
//                linear.setPower(0.5);
//            } else {
//                linear.setVelocity(fakePid(linear, linear.getPosition(), dumpLevel[levelTarget], linearMaxSpeed, 1.5)); //change the 3rd arg to adjust slow down speed, should be >1
//            }
//        } while (linear.getPosition() < (dumpLevel[levelTarget]) && !isStopRequested());
//        sleep(250);
//        outtake.goToAngle(outtakeDumpPos, 1500);



        /* ---------------- todo: one way - over barriers ---------------- */
//        //go back to spin location
//        long startGoBack = System.currentTimeMillis();
//        long timeOutGoBack = 2500;
//        do {
//            drivePower = fakePid_DrivingEdition(ashAdjusted, warehouseStage, positionTracker, speed, 6, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startGoBack) < timeOutGoBack));
//        sleep(1500);
//
//        //raise odometry pods
//        odoL.setAngle(180);
//        odoR.setAngle(178);
//        odoB.goToAngle(155, 2000);
//
//
//        drivetrain.stop();
//
//        //drive over barrier to park
//        double overBarrierPower = 0.8;
//        long startBarrier = System.currentTimeMillis();
//        long timeOutBarrier = 100;
//        do {
////            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
////            drivetrain.driveWithGamepad(1,0.8,0,0);
//            drivetrain.mLF.setPower(overBarrierPower);
//            drivetrain.mLB.setPower(overBarrierPower);
//            drivetrain.mRF.setPower(overBarrierPower);
//            drivetrain.mRB.setPower(overBarrierPower);
//
//        } while (((System.currentTimeMillis() - startBarrier) < timeOutBarrier));
//
//
        sleep(1000);



        /* ---------------- todo: other way - through wall edge ---------------- */
//        //go to wall
//        long startToWall = System.currentTimeMillis();
//        long timeOutToWall = 3000;
//        do {
//            drivePower = fakePid_DrivingEdition(ashAdjusted, approachWall, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startToWall) < timeOutToWall) && (detectRed.getDistance(DistanceUnit.CM) > 1));
//        sleep(500);
//
//        //go along wall into warehouse
//        long startWarehouse = System.currentTimeMillis();
//        long timeOutWarehouse = 1000;
//        do {
//            drivePower = fakePid_DrivingEdition(approachWall, warehouse, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startWarehouse) < timeOutWarehouse));
//        sleep(500);



        //lower slide to be ready to start teleop
        outtake.goToAngle(outtakeTravelPos, 750);
        do {
            linear.setPower(-0.5);
        } while (linear.getPosition() > (bottomLinearPos) && !isStopRequested());
        sleep(500);

        /* ---------------- shut down ---------------- */
        drivetrain.setBrake(true);
        drivetrain.stop();
        positionTracker.stop();
    }//end of runOpMode


    /* ----------- backend of drive code: fake pid, but curved on both ends ----------- */
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

//        //read out positions
        telemetry.addData("x current", odo.x);
        telemetry.addData("y current", -odo.y);
        telemetry.addData("phi current (deg)", odo.phi * 180 / Math.PI);
        telemetry.addData("", "");
        telemetry.addData("x target", targetPos[0]);
        telemetry.addData("y target", targetPos[1]);
        telemetry.addData("phi target (deg)", targetPos[2]);
        telemetry.update();

        return returnPowers;
    }

    /* ---------- used to slow a motor down when approching target pos ---------- */
    /* ------------- returns (distance left to travel)^(1/adjuster) ------------- */
    private double fakePid(DcMotorX motor, double startPos, double targetPos, double speed, /*double exponent, /*double adjuster,*/ double stopTolerance) {
        double currentPos = motor.getPosition();
        double totalMoveDist = (targetPos - startPos);
        double distanceToMove = Math.abs(targetPos - currentPos);

//        double linearExponent = 6;
//        if (totalMoveDist < 5) linearExponent = 2;
//        else if (totalMoveDist < 15) linearExponent = 4;
//        else if (totalMoveDist < 25) linearExponent = 6;
//        else linearExponent = 6;

        double negAdjust;
        if ((totalMoveDist > 0) && ((currentPos - startPos) < 2)) negAdjust = (totalMoveDist - 1);
        else negAdjust = 0;

        double distanceRemaining = Math.abs(targetPos - currentPos);
        if (distanceRemaining > stopTolerance) {
//            return (-(Math.pow((((currentPos) - (totalMoveDist/2) + negAdjust) / (totalMoveDist / 2)), (linearExponent))) + 1) * speed;
            return Math.pow(distanceToMove, speed / 50) * (currentPos < targetPos ? 1 : -1);
        } else {
            return 0.0;
        }
    }


    /* ----------------- spins a motor ----------------- */
    private void spin(DcMotorX name, double power, int waitTime) {
        name.setPower(power);
        sleep(waitTime);
        name.setPower(0);
    }


}//end of linear op mode