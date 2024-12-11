import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

@TeleOp(name = "Basic: Linear OpMode", group = "Linear OpMode")
@Disabled
class BasicOpModeLinear : LinearOpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private lateinit var leftDrive: DcMotor
    private lateinit var rightDrive: DcMotor

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor::class.java, "left_drive")
        rightDrive = hardwareMap.get(DcMotor::class.java, "right_drive")

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        // Wait for the game to start (driver presses START)
        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            var leftPower: Double
            var rightPower: Double

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            val drive = -gamepad1.left_stick_y.toDouble()
            val turn = gamepad1.right_stick_x.toDouble()
            leftPower = Range.clip(drive + turn, -1.0, 1.0)
            rightPower = Range.clip(drive - turn, -1.0, 1.0)

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.power = leftPower
            rightDrive.power = rightPower

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
            telemetry.update()
        }
    }
}
