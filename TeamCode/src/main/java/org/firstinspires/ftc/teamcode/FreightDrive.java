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
            spinner;
        private double power = 1;
        private TouchSensor spinLimit;
        //limit switch is named spinLimit

        //yes?? // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
        private State.Buttons lastButtons1 = new State.Buttons();
        private State.Dpad lastDpads1 = new State.Dpad();
        private State.Bumpers lastBumpers1 = new State.Bumpers();


        public void init(){
            DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                    mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                    mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                    mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

            drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

            spinner = new DcMotorX(hardwareMap.dcMotor.get("spinner"));
            //spinner.setBrake(true);
        }

        public void loop(){
            double leftX = gamepad1.left_stick_x;
            double rightX = -gamepad1.right_stick_x;
            double rightY = -gamepad1.right_stick_y; // Reads negative from the controller
            boolean a = gamepad1.a;
            int spinDirection = 1;


            if (gamepad1.left_bumper || gamepad2.left_bumper){
                spinDirection *= -1;
            }

            //spinner code
            if(gamepad1.a || gamepad2.a){
                    spinner.setPower(0.8 * spinDirection);
            }
            else{
                spinner.setPower(0);
            }
            

            // Drive the robot with joysticks if they are moved (with rates)
            if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
                drivetrain.driveWithGamepad(1, rateCurve(rightY, 1.7),rateCurve(leftX, 1.7)/* 0.5*leftX */, rateCurve(rightX,1.7));      //curved stick rates
            }else{
                // If the joysticks are not pressed, do not move the bot
                drivetrain.stop();
            }
        }

        private double rateCurve(double input, double rate){
            return Math.pow(Math.abs(input),rate)*((input>0)?1:-1);
        }


    }
