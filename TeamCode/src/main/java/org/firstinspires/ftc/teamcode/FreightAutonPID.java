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
public class FreightAutonPID extends LinearOpMode {

    // Odometry parameters
    private int ticksPerRev = 8192; //left same as last year
    private double circumference = 15.71; //left same as last year
    private double width = 26.9; //distance between centers of odometry wheels
    private double backDistancePerRadian = 170.556/ (2*Math.PI); //TODO: test to see what this is - rotate bot 360 - take the x value and put it over 2pi - it compensates fo the wheel being in the back of the bot

    //starting position
    private final double x0 = 0;
    private final double y0 = 0;
    private final double phi0 = 0;

//    private final double TILE_SIZE = 60.96; //NO we're not measuring in fractional tiles this year, SAE is enough as it is

    private ControlledDrivetrain drivetrain;

    private final double[] duckCheckPos = {5, 30, 0}; //x, y, phi
    private final double[] spinPos = {5, 50, 0}; //x, y, phi

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
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, x0, y0, phi0);

        // sets up drivetrain
        double[] kp = {22, 20, 20}; //kpx, kpy, kpphi
        double[] ki = {22, 20, 20}; //kix, kiy, kiphi
        double[] kd = {22, 20, 20}; //kdx, kdy, kdphi
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker, .01, .01, .01, kp, ki, kd, 50);
        //drivetrain.reverse();
        drivetrain.telemetry = telemetry;

        Thread drivetrainThread = new Thread(drivetrain);
        drivetrainThread.start();


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

        drivetrain.setPosition(duckCheckPos[0], duckCheckPos[1], duckCheckPos[2]);

        /* ------------------ other ------------------ */

//        spinDucks(0.5, 500); //turns on carousel spinner at power 0.5 for 500ms (or whatever you set them to)


        /* ---------------- shut down ---------------- */
        drivetrain.setBrake(true);
        drivetrain.stop();
        drivetrain.setActive(false);
        drivetrain.stopController();
    }//end of runOpMode

    /* ----------------- spins the carousel ----------------- */
    private void spinDucks(double power, int waitTime) {
        //will spin duck wheel- intake for testing for now
        intake.setPower(power);
        sleep(waitTime);
        intake.setPower(0);
    }


}//end of linear op mode