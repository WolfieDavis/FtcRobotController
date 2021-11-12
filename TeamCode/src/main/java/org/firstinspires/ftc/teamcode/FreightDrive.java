    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.TouchSensor;

    import org.firstinspires.ftc.teamcode.api.DcMotorX;
    import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
    import org.firstinspires.ftc.teamcode.api.Drivetrain;
    import org.firstinspires.ftc.teamcode.api.State;
    import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
    import org.firstinspires.ftc.teamcode.api.DcMotorX;
    import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
    import org.firstinspires.ftc.teamcode.api.Odometry;
    import org.firstinspires.ftc.teamcode.api.ServoX;
    import org.firstinspires.ftc.teamcode.api.State;

    @TeleOp
    public class FreightDrive extends OpMode {


        private Drivetrain drivetrain;
        private DcMotorX
            spinner,
            linear,
            intake;

        private double power = 1;
        private TouchSensor spinLimit;
        //limit switch is named spinLimit

        // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
        private State.Buttons lastButtons1 = new State.Buttons();
        private State.Dpad lastDpads1 = new State.Dpad();
        private State.Bumpers lastBumpers1 = new State.Bumpers();


        public void init(){
            DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                    mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                    mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                    mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

            linear = new DcMotorX(hardwareMap.dcMotor.get("linear"));//motor for linear rail
            intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));//motor for intake spinner
            spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));//motor for carousel spinner

            drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        }// end of init

        public void loop(){
            double leftX = gamepad1.left_stick_x;
            double rightX = -gamepad1.right_stick_x;
            double rightY = -gamepad1.right_stick_y; // Reads negative from the controller
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean bumperLeft = gamepad1.left_bumper;
            boolean bumperRight = gamepad2.right_bumper;
            boolean xHit = x && !lastButtons1.x;
            boolean yHit = y && !lastButtons1.y;
            boolean aHit = a && !lastButtons1.a;
            boolean bHit = b && !lastButtons1.b;
            boolean bumperLeftHit = bumperLeft && !lastBumpers1.left_bumper;
            boolean bumperRightHit = bumperRight && !lastBumpers1.right_bumper;

            //random toggle / value change buttons
            int spinDirection = 1;
            int intakeToggle = 1;

            spinDirection = (bumperLeftHit)? spinDirection *= -1: spinDirection; //reverse the direction if left bumper  is pressed


            //code for the linear rail uses the values read by the trigger. curve it later.
            if (gamepad1.right_trigger >  0.01){
                linear.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.01){
                linear.setPower(-gamepad1.left_trigger);
            } else {
                linear.setPower(0.0);
            }

            //intake spinner is toggled if b is pressed
            if (bHit){
                intakeToggle *= -1;
                switch (intakeToggle){
                    case -1 :
                        intake.setPower(0.5);
                        break;
                    case 1 :
                        intake.setPower(0.0);
                        break;
                }//end of switch case
            }

            //carousel spinner
            if(gamepad1.a || gamepad2.a){
                spinner.setPower(0.8 * spinDirection);
            } else {
                spinner.setPower(0);
            }


            // Drive the robot with joysticks if they are moved (with rates)
            if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
                drivetrain.driveWithGamepad(1, rateCurve(rightY, 1.7),rateCurve(leftX, 1.7)/* 0.5*leftX */, rateCurve(rightX,1.7));      //curved stick rates
            }else{
                // If the joysticks are not pressed, do not move the bot
                drivetrain.stop();
            }

            // Save button states
            lastButtons1.update(a, b, x, y);
            //lastDpads1.update(dpadUp, dpadDown, dpadRight, dpadLeft);
            lastBumpers1.update(bumperRight, bumperLeft);

        }//end of loop

        private double rateCurve(double input, double rate){
            return Math.pow(Math.abs(input),rate)*((input>0)?1:-1);
        }


    }
