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
public class FreightAutonBlueCloseLimit extends LinearOpMode {

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
            wheelB,
            linear;
    private ServoX
            outtake,
            tip,
            odoL,
            odoR,
            odoB;
    DistanceSensor detectBlue;
    //    DistanceSensor detectRed;
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

        detectBlue = hardwareMap.get(DistanceSensor.class, "detectBlue");
//        detectRed = hardwareMap.get(DistanceSensor.class, "detectRed");
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

        telemetry.addData("Done initializing", "");
        telemetry.update();
//        linear.reset();

        /* ----------- waiting for start ----------- */
//        linear.resetEncoder();
        waitForStart();

        /* ------------ setup movement ------------ */
        //movement parameters
        double exponent = 4; //4 //exponent that the rate curve is raised to
//        double[] speed = {0.4, 0.3, 0.35}; //x, y, phi //.35    //first argument(number) is for straight line movement, second is for turning
        double[] speed = {0.45, 0.4, 0.35};//todo: fix this??
//        double[] speed = {0.55, 0.5, 0.35};
//        double[] detectSpeed = {0.35, 0.2, 0.35};
        double[] detectSpeed = {0.35, 0.25, 0.35};
        double[] stopTolerance = {4, (Math.PI / 45)}; //4 //acceptable tolerance (cm for linear, radians for turning) for the robot to be in a position

        //just needs to be here
        double[] drivePower;

        //positions: in the format x, y, phi. (in cm for x and y and radians for phi) this can be declared at the top of the program
        double[] ash = {-118.5 - 9, -105 + 1.5, 0}; //-105+1.5 for y
        double[] ashLow = {-118.5 - 9, -105 + 2.5, 0}; //-105+1.5 for y
        double[] detect2 = {-53, -91 - 1.5, 0}; //68.5 too far //location for detecting the top placement
        double[] detect1 = {-53, -69.5, 0}; //todo -68.5 //location for detecting the middle location
        double[] carouselStage = {-60, -60, 0};
        double[] carouselStageSpin = {carouselStage[0], carouselStage[1], Math.PI / 2};

        //positions after reset
        double[] carouselStageAfterSpin = {0, 0, 0};
//        double[] carouselNormal = {-45, -25, Math.PI/2}; //32.5, -23, 0
//        double[] carouselNormal = {-40, -19, Math.PI/2}; //32.5, -23, 0
//        double[] carouselAdjusted = {-29, 17, 0}; //-35 //-30, 18
        double[] carouselAdjusted = {-32, 21, 0}; //-35 //-30, 18 //todo: fine tune this
//        double[] asuParkNormal = {-91.75, -25, Math.PI/2}; //89, -25, 0
        double[] asuParkAdjusted = {-31, -21, 0}; //todo -30, -30 og, move towards carousel

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
            levelTarget = 0;
            detectZone = detect2;
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
                levelTarget = 2;
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
        telemetry.update();

        sleep(500);

        double maxLinearPower = 0.2;

        long startDump = System.currentTimeMillis();
        long timeOutDump = 2000;

        if (levelTarget == 0)
            do linear.setPower(maxLinearPower);
            while (!low.isPressed() && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));
        else if (levelTarget == 1)
            do linear.setPower(maxLinearPower);
            while (!middle.isPressed() && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));
        else
            do linear.setPower(maxLinearPower);
            while (!top.isPressed() && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));

        sleep(2000);

//
//        //go to ash
//        long startASH = System.currentTimeMillis();
//        long timeOutASH = 2500;
//        do {
//            drivePower = fakePid_DrivingEdition(detectZone, ash, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startASH) < timeOutASH));
//        sleep(500);
//
//
//        //reset linear slide encoder
////        linear.resetEncoder();
//
//
//        //raise and dump
//        long startDump = System.currentTimeMillis();
//        long timeOutDump = 2500;
//        do {
//            if (levelTarget == 2) {
//                linear.setPower(0.5);
//            } else {
//                linear.setVelocity(fakePid(linear, linear.getPosition(), dumpLevel[levelTarget], linearMaxSpeed, 1.5)); //change the 3rd arg to adjust slow down speed, should be >1
//            }
//        } while ((linear.getPosition() < (dumpLevel[levelTarget])) && !isStopRequested() && ((System.currentTimeMillis() - startDump) < timeOutDump));
//        sleep(250);
//        outtake.goToAngle(outtakeDumpPos, 1500);
//
//        //carousel staging location
//        long startCarouselStage = System.currentTimeMillis();
//        long timeOutCarouselStage = 3000;
//        sleep(250);
//        do {
//            drivePower = fakePid_DrivingEdition(ash, carouselStage, positionTracker, speed, 6, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startCarouselStage) < timeOutCarouselStage));
//
//        //carousel staging location - spin robot to get ready
//        long startCarouselStageSpin = System.currentTimeMillis();
//        long timeOutCarouselStageSpin = 3000;
//        do {
//            drivePower = fakePid_DrivingEdition(carouselStage, carouselStageSpin, positionTracker, speed, 6, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startCarouselStageSpin) < timeOutCarouselStageSpin));
//        sleep(500);
//
//        //reset odometry system heading
//        positionTracker.x = carouselStageAfterSpin[0];
//        positionTracker.y = carouselStageAfterSpin[1];
//        positionTracker.phi = carouselStageAfterSpin[2];
//        sleep(50);
//
//        //go to carousel
//        long startCarousel = System.currentTimeMillis();
//        long timeOutCarousel = 2000;
//        do {
//            drivePower = fakePid_DrivingEdition(carouselStageAfterSpin, carouselAdjusted, positionTracker, speed, 6, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startCarousel) < timeOutCarousel));
//        sleep(250);
//
//        //spin carousel
//        spin(spinner, -0.5 * side, 6000); //turns on carousel spinner at power 0.5 for 500ms (or whatever you set them to)
//
//        //park in asu
//        long startPark = System.currentTimeMillis();
//        long timeOutPark = 2750;
//        do {
//            drivePower = fakePid_DrivingEdition(carouselAdjusted, asuParkAdjusted, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startPark) < timeOutPark));
//        sleep(250);

        //lower slide to be ready to start teleop
        outtake.goToAngle(outtakeTravelPos, 750);
        do {
            linear.setPower(-0.1);
        } while (!bottom.isPressed() && !isStopRequested());
        sleep(500);

        /* ---------------- shut down ---------------- */
        drivetrain.setBrake(true);
        drivetrain.stop();
        positionTracker.stop();
//        drivetrain.stopController();
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
//        telemetry.addData("x current", odo.x);
//        telemetry.addData("y current", -odo.y);
//        telemetry.addData("phi current (deg)", odo.phi * 180 / Math.PI);
//        telemetry.addData("", "");
//        telemetry.addData("x target", targetPos[0]);
//        telemetry.addData("y target", targetPos[1]);
//        telemetry.addData("phi target (deg)", targetPos[2]);
//        telemetry.update();

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