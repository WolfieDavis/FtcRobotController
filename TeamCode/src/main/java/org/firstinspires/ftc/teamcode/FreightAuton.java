package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;

import java.util.Arrays;

@Autonomous
public class FreightAuton extends LinearOpMode {

    // Odometry parameters
    private int ticksPerRev = 8192; //left same as last year
    private double circumference = 15.71; //left same as last year
    private double width = 26.9; //distance between centers of odometry wheels
    private double backDistancePerRadian = 1.43 / (2 * Math.PI); //TODO: test to see what this is - rotate bot 360 - take the x value and put it over 2pi - it compensates fo the wheel being in the back of the bot

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
            odoL,
            odoR,
            odoB;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get all of the drivetrain motors
        mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF"));
        mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF"));
        mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB"));
        mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("odoR"), ticksPerRev, (-circumference));
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference));
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, circumference);

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, 0, 0, 0);

        // sets up drivetrain
        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);
        drivetrain.reverse();
        // drivetrain.telemetry = telemetry;

        Thread positionTracking = new Thread(positionTracker);
        positionTracking.start();

        //TODO: add linear slide code in here if we are using it

        telemetry.addData("Done initializing", "");
        telemetry.update();


        /* ----------- waiting for start ----------- */
        waitForStart();

        //drop odometry pods
//        odoL.setAngle(0);
//        odoR.setAngle(0);
//        odoB.setAngle(0); //odoB.goToAngle(0, 500); //gives them time to drop //TODO: move odometry and drivetrain init code down here to make sure it initializes when servos are down and reading

        //reads out where we are in the code
        telemetry.addData("started??", "");
        telemetry.update();


        /* --------------- move robot --------------- */
        //movement parameters
        double[] speed = {0.5, 0.5}; //first arg is for straight line movement, second is for turning
        double[] adjuster = {20, 20}; //how "curved" the rate is, needs to be > 1
        double[] stopTolerance = {10, 10}; //acceptable tolerance (cm) for the robot to be in a position

        //just needs to be here
        double[] drivePower;

        //positions
       // double[] position1 = {0, 50, 0}; //x, y, phi. (in cm for x and y and radians for phi) this can be declared at the top of the program
        double[] position1 = {0, 50, 0};
        double[] position2 = {50, 50, 0};
        double[] position3 = {0, 0, 0};

//        do{
//            //read out positions
//            telemetry.addData("x current", positionTracker.x);
//            telemetry.addData("y current", positionTracker.y);
//            telemetry.addData("phi current (deg)", positionTracker.phi * 180 / Math.PI);
//            telemetry.addData("x target", position1[0]);
//            telemetry.addData("y target", position1[1]);
//            telemetry.addData("phi target (deg)", position1[2]);
//            telemetry.update();
//        }while (!isStopRequested());




        //for each movement copy this while loop, change position1
        do {
            drivePower = fakePid_DrivingEdition(position1, positionTracker, speed, adjuster, stopTolerance);
            drivetrain.driveWithGamepad(0.2, -drivePower[1], drivePower[2], drivePower[0]);

        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));

        sleep(1000);

        //strafe
        do {
            drivePower = fakePid_DrivingEdition(position2, positionTracker, speed, adjuster, stopTolerance);
            drivetrain.driveWithGamepad(0.5, -drivePower[1], drivePower[2], -drivePower[0]);

        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));

        sleep(1000);

        do {
            drivePower = fakePid_DrivingEdition(position3, positionTracker, speed, adjuster, stopTolerance);
            drivetrain.driveWithGamepad(0.5, drivePower[1], drivePower[2], drivePower[0]);

        } while (!isStopRequested() && !Arrays.equals(drivePower, new double[]{0, 0, 0}));


        /* ------------------ other ------------------ */

        //    spinDucks(0.5, 500); //turns on carousel spinner at power 0.5 for 500ms (or whatever you set them to)


        /* ---------------- shut down ---------------- */
        drivetrain.setBrake(true);
        drivetrain.stop();
    }//end of runOpMode


    /* ----------- backend of drive code: fake pid ----------- */
    private double[] fakePid_DrivingEdition(double[] targetPos, Odometry odo, double[] speed, double[] adjuster, double[] stopTolerance) {
        double[] distanceToMove = {targetPos[0] - odo.x, targetPos[1] - odo.y, targetPos[2] - odo.phi};
        double totalDistance = Math.sqrt(Math.pow(distanceToMove[0], 2) + Math.pow(distanceToMove[1], 2));

        double[] returnPowers = {0, 0, 0};
        if (totalDistance > stopTolerance[0]) {
            double[] powerFractions = {distanceToMove[0] / totalDistance, distanceToMove[1] / totalDistance};
            if (powerFractions[0] > powerFractions[1]) {
                powerFractions[1] = powerFractions[1] / powerFractions[0];
                powerFractions[0] = 1;
            } else {
                powerFractions[0] = powerFractions[0] / powerFractions[1];
                powerFractions[1] = 1;
            }
            double fakePidAdjustment = Math.pow(totalDistance, speed[0] / adjuster[0]);
            returnPowers[0] = powerFractions[0] * fakePidAdjustment;
            returnPowers[1] = powerFractions[1] * fakePidAdjustment;
        }
        double totalTurnDistance = Math.abs(distanceToMove[2]);
        if (totalTurnDistance > stopTolerance[1]) {
            returnPowers[2] = Math.pow(totalTurnDistance, speed[1] / adjuster[1]) * (distanceToMove[2] >= 0 ? 1 : -1);
        }

//        //read out positions
        telemetry.addData("x current", odo.x);
        telemetry.addData("y current", odo.y);
        telemetry.addData("phi current (deg)", odo.phi * 180 / Math.PI);
        telemetry.addData("x target", targetPos[0]);
        telemetry.addData("y target", targetPos[1]);
        telemetry.addData("phi target (deg)", targetPos[2]);
        telemetry.update();

        return returnPowers;
    }

    /* ----------------- spins the carousel ----------------- */
    private void spinDucks(double power, int waitTime) {
        //will spin duck wheel- intake for testing for now
        intake.setPower(power);
        sleep(waitTime);
        intake.setPower(0);
    }


}//end of linear op mode