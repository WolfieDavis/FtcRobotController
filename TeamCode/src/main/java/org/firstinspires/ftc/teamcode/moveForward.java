package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;

import java.util.Arrays;

@Autonomous
public class moveForward extends LinearOpMode {

    int side = 1; //modifier for side: set to 1 for red, or -1 for blue

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
            linear,
            tapeExtend;
    private ServoX
            outtake,
            outtake2,
            tip,
            odoL,
            odoR,
            odoB,
            tapePan,
            tapeTilt;
    DistanceSensor detectBlue;
    DistanceSensor detectRed;
    private TouchSensor
            bottom,
            low,
            middle,
            top;
    private double
            tapePanValue = 90,
            tapeTiltValue = 90,
            panMin = 90 - 50, //left
            panMax = 90 + 50, //right
            tiltMin = 90 - 50, //down
            tiltMax = 90 + 40, //up
            tapeExtendPower = 1, //.85
            tapePanMultiplier = 0.10, //.15
            tapeTiltMultiplier = 0.10, //.15

    //outtake (servo) positions
    outtake2Offset = 15 + 5, //-15 bc slide is 15 deg, and 5 for other adjustment
            outtake2CollectPos = -3.5 + outtake2Offset, //starting with 0 as bottom
            outtake2TravelPos = 42.5 + outtake2Offset,
            outtake2DumpPos = 95 - 10 + outtake2Offset,
            outtake2DumpPos2 = 95 - 15 + outtake2Offset;


    @Override
    public void runOpMode() throws InterruptedException {
        /* ----------- waiting for start ----------- */
        waitForStart();

        // Get all of the drivetrain motors
        mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF"));
        mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF"));
        mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB"));
        mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));
        tapeExtend = new DcMotorX(hardwareMap.dcMotor.get("odoRear"));//encoder for rear odometry pod, and motor for capping mechanism


        //set up other motors
        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));
        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));
        linear = new DcMotorX(hardwareMap.dcMotor.get("linear"));
        outtake = new ServoX(hardwareMap.servo.get("outtake"));
        outtake2 = new ServoX(hardwareMap.servo.get("outtake2"));

        //servos to raise and lower the odometry pods
        odoL = new ServoX(hardwareMap.servo.get("odoL"));
        odoR = new ServoX(hardwareMap.servo.get("odoR"));
        odoB = new ServoX(hardwareMap.servo.get("odoB"));
        tapePan = new ServoX(hardwareMap.servo.get("tapePan"), 180, panMax, panMin);
        tapeTilt = new ServoX(hardwareMap.servo.get("tapeTilt"), 180, tiltMax, tiltMin);

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


        //sets up threading for odometry
        Thread positionTracking = new Thread(positionTracker);
        positionTracking.start();

        //displays on the phone that everything has initialized correctly
        telemetry.addData("Done initializing", "");
        telemetry.update();


        //spin carousel
        spin(tapeExtend, 0.5, 1500);


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