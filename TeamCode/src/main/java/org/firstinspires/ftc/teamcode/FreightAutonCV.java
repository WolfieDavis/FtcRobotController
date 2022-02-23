package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.api.TensorFlowX;

import java.util.Arrays;

//@Autonomous
public class FreightAutonCV extends LinearOpMode {

    // Odometry parameters
    private int ticksPerRev = 8192; //left same as last year
    private double circumference = 15.71; //left same as last year
    private double width = 26.9; //distance between centers of odometry wheels
    private double backDistancePerRadian = 170.556/ (2*Math.PI); //TODO: test to see what this is - rotate bot 360 - take the x value and put it over 2pi - it compensates fo the wheel being in the back of the bot


    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "FreightFrenzy.tflite";
    private final String DUCK_LABEL = "Duck";
//    private final String SINGLE_LABEL = "Single";

    private TensorFlowX tfod;

//    private final double TILE_SIZE = 60.96; //NO we're not measuring in fractional tiles this year, SAE is enough as it is

    private ControlledDrivetrain drivetrain;

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

        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));
        linear = new LimitedMotorX(hardwareMap.dcMotor.get("linear"));
        outtake = new ServoX(hardwareMap.servo.get("outtake"));

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("odoR"), ticksPerRev, (-circumference));
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, (-circumference));
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mLB"), ticksPerRev, circumference);

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, 0, 0, 0);

        // sets up drivetrain
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        drivetrain.reverse();
        // drivetrain.telemetry = telemetry;

        Thread positionTracking = new Thread(positionTracker);
        positionTracking.start();

        //TODO: add linear slide code in here if we are using it

        telemetry.addData("Done initializing", "");
        telemetry.update();

        try {
            tfod = new TensorFlowX(
                    TFOD_MODEL_ASSET,
                    VUFORIA_KEY,
                    VuforiaLocalizer.CameraDirection.BACK,
                    new String[]{DUCK_LABEL},
                    hardwareMap
            );
        }catch(Exception e){
            telemetry.addData("Error initializing TensorFlow", e);
        }


        /* ----------- waiting for start ----------- */
        waitForStart();


        //reads out where we are in the code
        telemetry.addData("started??", "");
        telemetry.update();


        /* --------------- move robot --------------- */
        //movement parameters


        long start = System.currentTimeMillis();

        long minTime = 1000;

        // Continue checking for rings until the time runs out or stacked rings are detected
        while((System.currentTimeMillis() - start) < minTime && !isStopRequested()){

            // Get updated object recognition data from TensorFlow
            try {
                Recognition duck = tfod.recognize(DUCK_LABEL);

                // Determine number of rings based on stack type detected
                if(duck != null){
                    telemetry.addLine("suck is present");
                    telemetry.update();
                }
            }catch(Exception e){
                telemetry.addData("Exception", e);
            }
        }



        /* ------------------ other ------------------ */


        //spinDucks(0.5, 500); //turns on carousel spinner at power 0.5 for 500ms (or whatever you set them to)


        /* ---------------- shut down ---------------- */
        tfod.shutdown();
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