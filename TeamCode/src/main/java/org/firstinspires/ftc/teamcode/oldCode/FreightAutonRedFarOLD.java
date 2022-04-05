package org.firstinspires.ftc.teamcode.oldCode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
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

//@Autonomous
public class FreightAutonRedFarOLD extends LinearOpMode {

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
    DistanceSensor detectR;

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
        //servos to raise and lower the odometry pods
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));

        detectR = hardwareMap.get(DistanceSensor.class, "detectR");

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("odoR"), ticksPerRev, (circumference));
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference));
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, -(circumference));

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, 0, 0, 0);

        // sets up drivetrain
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        //sets initial position for the drivetrain
        double[] initialPos = {17, -201.5, 0}; //x, y, phi
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
        double[] detectSpeed = {0.35, 0.2, 0.35};
        double[] stopTolerance = {4, (Math.PI / 45)}; //4 //acceptable tolerance (cm for linear, radians for turning) for the robot to be in a position

        //just needs to be here
        double[] drivePower;

        //positions: in the format x, y, phi. (in cm for x and y and radians for phi) this can be declared at the top of the program
        double[] startOffset = {0, -47.25*2.54, 0};

        double[] carousel = {32.5, -23, 0};
        double[] ashStage = {118.5, -201, 0};
//        double[] ashSpin = {ashStage[0], ashStage[1], -Math.PI};
        double[] ash = {118.5, -195, -Math.PI};
        double[] asuPark = {91.75, -25, 0}; //89, -25, 0
        double[] detect2 = {53, -91+startOffset[1], 0}; //68.5 too far
        double[] detect1 = {53, -68.5+startOffset[1], 0};
        double[] ashSpin = {detect1[0], detect1[1], -Math.PI};
        double[] detect0 = {53, -48.5+startOffset[1], 0};

        //outtake positions
        double[] dumpLevel = {1.25, 6.5, 13.6875}; //low (3), med(8), high(13.6875)
        double outtakeTravelPos = 137.5; //servo position for travel
        double outtakeDumpPos = 85; //servo position for dump

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
        double distance = detectR.getDistance(DistanceUnit.CM);
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

            distance = detectR.getDistance(DistanceUnit.CM);
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

        sleep(500);

//        //go to ash staging position
//        long startAshStage = System.currentTimeMillis();
//        long timeOutAshStage = 2000;
//        do {
//            drivePower = fakePid_DrivingEdition(detectZone, ashStage, positionTracker, speed, exponent, stopTolerance);
//            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
//        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startAshStage) < timeOutAshStage));
//        sleep(750);

        //spin 180
        long startAshSpin = System.currentTimeMillis();
        long timeOutAshSpin = 2000;
        do {
            drivePower = fakePid_DrivingEdition(detect1, ashSpin, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startAshSpin) < timeOutAshSpin));
        sleep(1000);

        //approach ash
        long startASH = System.currentTimeMillis();
        long timeOutASH = 2000;
        do {
            drivePower = fakePid_DrivingEdition(ashSpin, ash, positionTracker, speed, exponent, stopTolerance);
            drivetrain.driveWithGamepad(1, drivePower[1], drivePower[2], drivePower[0]);
        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}) && ((System.currentTimeMillis() - startASH) < timeOutASH));
        sleep(2000);

        //raise and dump
        do {
            if (levelTarget == 2) {
                linear.setPower(0.8);
            } else {
                linear.setVelocity(fakePid(linear, 0, dumpLevel[levelTarget], 0.8, exponent, 0.625)); //change the 3rd arg to adjust slow down speed, should be >1
            }
        } while (linear.getPosition() < (dumpLevel[levelTarget]) && !isStopRequested());
        sleep(250);
        outtake.goToAngle(outtakeDumpPos, 1500);


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
            double fakePidAdjustment = (-(Math.pow((((totalDistanceRemaining) - (totDistBetween/2)) / (totDistBetween/2)), (exponent))) + 1); //curved as it starts and ends - experimental
            returnPowers[0] = powerFractions[0] * fakePidAdjustment * speed[0];
            returnPowers[1] = powerFractions[1] * fakePidAdjustment * speed[1];
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
    private double fakePid(DcMotorX motor, double startPos, double targetPos, double speed, double exponent, /*double adjuster,*/ double stopTolerance) {
        double currentPos = motor.getPosition();
        double totalMoveDist = (targetPos - startPos);
        double distanceRemaining = Math.abs(targetPos - currentPos);
        if (distanceRemaining > stopTolerance) {
            return (-(Math.pow((((currentPos) - (totalMoveDist/2)) / (totalMoveDist/2)), (exponent))) + 1) * speed;
//            return Math.pow(distanceToMove, speed / adjuster) * (currentPos < targetPos ? 1 : -1);
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