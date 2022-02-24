package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;

import java.util.Arrays;

@Autonomous
public class FreightAutonDetect extends LinearOpMode {

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
    DistanceSensor detect;

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
        linear = new LimitedMotorX(hardwareMap.dcMotor.get("linear"), 1607, 13.6875);
        outtake = new ServoX(hardwareMap.servo.get("outtake"));
        detect = hardwareMap.get(DistanceSensor.class, "detect");

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("odoR"), ticksPerRev, (circumference));
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference));
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, -(circumference));

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, 0, 0, 0);

        // sets up drivetrain
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        //sets initial position for the drivetrain
        double[] initialPos = {17, -95.7, 0}; //x, y, phi
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

        //TODO: drop odometry pods
//        odoL.setAngle(0);
//        odoR.setAngle(0);
//        odoB.setAngle(0); //odoB.goToAngle(0, 500); //gives them time to drop


        /* ------------ setup movement ------------ */
        //movement parameters
        double exponent = 4; //4 //exponent that the rate curve is raised to
        double[] speed = {0.4, 0.3, 0.35}; //x, y, phi     //first argument(number) is for straight line movement, second is for turning
        double[] stopTolerance = {4, (Math.PI / 45)}; //acceptable tolerance (cm for linear, radians for turning) for the robot to be in a position

        //just needs to be here
        double[] drivePower;

        //positions: in the format x, y, phi. (in cm for x and y and radians for phi) this can be declared at the top of the program
        double[] carousel = {26, -33, 0}; //27.5, -33, 0
        double[] ash = {118.5, -105, 0}; //-102 for y
        double[] asuPark = {89, -25, 0};
        double[] clearWall = {initialPos[0] + 15, initialPos[1], initialPos[2]};
        double[] detect2 = {53, -91, 0}; //68.5 too far
        double[] detect1 = {53, -70, 0};
        double[] detect0 = {53, -48.5, 0};

        //outtake positions
        double[] dumpLevel = {3, 8, 13.6875}; //low, med, high
        double outtakeTravelPos = 137.5; //servo position for travel
        double outtakeDumpPos = 85; //servo position for dump

        /* --------------- move robot --------------- */
        //move bucket up
        outtake.goToAngle(outtakeTravelPos, 500);

        //strafe clear of the wall
        do {
            drivePower = fakePid_DrivingEdition(initialPos, clearWall, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
        sleep(500);

        //go to detect location
        do {
            drivePower = fakePid_DrivingEdition(clearWall, detect2, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
        sleep(500);

        //detect freight TODO: write more code and make this actually work
        int levelTarget;
        double distance = detect.getDistance(DistanceUnit.CM);
        double detectZone[];

        if (distance < 20) {
            levelTarget = 2;
            detectZone = detect2;
        } else {
            do {
                drivePower = fakePid_DrivingEdition(detect2, detect1, positionTracker, speed, exponent, stopTolerance);
                drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
            } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));
            sleep(500);

            distance = detect.getDistance(DistanceUnit.CM);
            if (distance < 20) {
                levelTarget = 1;
                detectZone = detect2;
            } else {
                levelTarget = 0;
                detectZone = detect1;
            }
        }

        if (levelTarget == 0){
            telemetry.addData("level", "low");
        } else if (levelTarget == 1){
            telemetry.addData("level", "middle");
        } else {
            telemetry.addData("level", "top");
        }
//        telemetry.addData("level", levelTarget);
        telemetry.update();

        wait(10000);

//        //go to ash
//        long startASH = System.currentTimeMillis();
//        long timeOutASH = 6000;
//        do {
//            drivePower = fakePid_DrivingEdition(detectZone, ash, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startASH) < timeOutASH));
//        sleep(500);
//
//        //raise and dump
//        do {
//            linear.setPower(0.8);
//        } while (linear.getPosition() < dumpLevel[levelTarget]);
//        outtake.goToAngle(outtakeDumpPos, 1500);
////        outtake.goToAngle(outtakeTravelPos, 500);
//
//        //go to carousel
//        long startCarousel = System.currentTimeMillis();
//        long timeOutCarousel = 4500;
//        do {
//            drivePower = fakePid_DrivingEdition(ash, carousel, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startCarousel) < timeOutCarousel));
//
//        //spin carousel
//        spin(spinner, -0.5, 6000); //turns on carousel spinner at power 0.5 for 500ms (or whatever you set them to)
//
//        //park in asu
//        long startPark = System.currentTimeMillis();
//        long timeOutPark = 4500;
//        do {
//            drivePower = fakePid_DrivingEdition(carousel, asuPark, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startPark) < timeOutPark));


        /* ---------------- shut down ---------------- */
        drivetrain.setBrake(true);
        drivetrain.stop();
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
            double scaleToOne;
            if (powerFractions[0] > powerFractions[1]) {
                scaleToOne = Math.abs(powerFractions[0]);
            } else {
                scaleToOne = Math.abs(powerFractions[1]);
            }
            double fakePidAdjustment = (-(Math.pow((((totalDistanceRemaining) - (totDistBetween)) / (totDistBetween)), (exponent))) + 1); //curved as it starts and ends - experimental
            returnPowers[0] = powerFractions[0] / scaleToOne * fakePidAdjustment * speed[0];
            returnPowers[1] = powerFractions[1] / scaleToOne * fakePidAdjustment * speed[1];
        }
        double totalTurnDistance = Math.abs(distanceToMoveRemaining[2]);
        if (totalTurnDistance > stopTolerance[1]) {
            returnPowers[2] = speed[2] * (distanceToMoveRemaining[2] >= 0 ? 1 : -1);
        }

        //read out positions
        telemetry.addData("x current", odo.x);
        telemetry.addData("y current", -odo.y);
        telemetry.addData("phi current (deg)", odo.phi * 180 / Math.PI);
        telemetry.addData("x target", targetPos[0]);
        telemetry.addData("y target", targetPos[1]);
        telemetry.addData("phi target (deg)", targetPos[2]);
        telemetry.update();

        return returnPowers;
    }

    /* ---------- used to slow a motor down when approching target pos ---------- */
    /* ------------- returns (distance left to travel)^(1/adjuster) ------------- */
    private double fakePid(DcMotorX motor, double targetPos, double speed, double adjuster, double stopTolerance) {
        double currentPos = motor.getPosition();
        double distanceToMove = Math.abs(targetPos - currentPos);
        if (distanceToMove > stopTolerance) {
            return Math.pow(distanceToMove, speed / adjuster) * (currentPos < targetPos ? 1 : -1);
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