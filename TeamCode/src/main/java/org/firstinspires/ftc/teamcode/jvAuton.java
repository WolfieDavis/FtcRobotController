package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.ServoX;

//@Autonomous
public class jvAuton extends LinearOpMode {

    private Drivetrain drivetrain;

    private DcMotorX
            spinner;

    private ServoX
            plow;

    public void runOpMode(){
        DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));
        plow = new ServoX(hardwareMap.servo.get("plow"));

        waitForStart();


        //Start pos of bot
        final int startPosX = 0;
        final int startPosY = 0;


//        Automation
        long startDriveForward = System.currentTimeMillis();
        while(System.currentTimeMillis() - startDriveForward < 500 && !isStopRequested()){

            /* ---- Code in simple language drivetrain.drive(0.5, Drivetrain.Direction.BACKWARD); moves robot for 1500 mili seconds ----*/

            drivetrain.drive(0.5, Drivetrain.Direction.FORWARD);

        }
        //Sets down the box to push stuff
//        plow.setAngle(90);




        drivetrain.stop();
    }

}
