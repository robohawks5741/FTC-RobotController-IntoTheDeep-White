import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import ConceptVisionColorLocator
import android.util.Size
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion
import java.util.Locale
import kotlin.math.abs

@TeleOp(name = "Basic: Linear OpMode", group = "Linear OpMode")
@Disabled
class ClawTest : LinearOpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private lateinit var leftDrive: DcMotor
    private lateinit var rightDrive: DcMotor
    private lateinit var claw: Servo

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        var visionLocator = ConceptVisionColorLocator()
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        claw = hardwareMap.get(Servo::class.java, "claw")

        var clawAngle = claw.position

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        val colorLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE) // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // exclude blobs inside blobs
            .setRoi(
                ImageRegion.asUnityCenterCoordinates(
                    -0.5,
                    0.5,
                    0.5,
                    -0.5
                )
            ) // search central 1/4 of camera view
            .setDrawContours(true) // Show contours on the Stream Preview
            .setBlurSize(5) // Smooth the transitions between different colors in image
            .build()

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        val portal = VisionPortal.Builder()
            .addProcessor(colorLocator)
            .setCameraResolution(Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()

        telemetry.msTransmissionInterval = 50 // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
        // Wait for the game to start (driver presses START)
        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            clawAngle = claw.position
            var leftPower: Double
            var rightPower: Double

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            telemetry.addData("preview on/off", "... Camera Stream\n")

            // Read the current list
            val blobs = colorLocator.blobs

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */
            ColorBlobLocatorProcessor.Util.filterByArea(
                50.0,
                20000.0,
                blobs
            ) // filter out very small blobs.

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            val boxfit = blobs[0].boxFit
            if (abs(boxfit.angle.toInt() - clawAngle) < 10){
                claw.position = clawAngle + boxfit.angle.toInt()
            }
            telemetry.addLine(" Area Density Aspect  Center")

            // Display the size (area) and center location for each Blob.
            for (b in blobs) {
                val boxFit = b.boxFit
                telemetry.addLine(
                    String.format(
                        Locale.US,
                        "%5d  %4.2f   %5.2f  (%3d,%3d) %4.2f",
                        b.contourArea,
                        b.density,
                        b.aspectRatio,
                        boxFit.center.x.toInt(),
                        boxFit.center.y.toInt(),
                        boxFit.angle.toInt()

                    )
                )
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: $runtime")

            telemetry.update()
        }

    }
}
