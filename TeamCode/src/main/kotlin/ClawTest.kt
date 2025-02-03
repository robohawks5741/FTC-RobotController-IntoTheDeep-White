import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import ConceptVisionColorLocator
import android.annotation.SuppressLint
import android.util.Size
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.Point
import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan

@TeleOp(name = "ClawTest", group = "Linear OpMode")

class ClawTest : LinearOpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private lateinit var leftDrive: DcMotor
    private lateinit var rightDrive: DcMotor
    private lateinit var claw: Servo

    @SuppressLint("DefaultLocale")
    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        var visionLocator = ConceptVisionColorLocator()
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        claw = hardwareMap.get(Servo::class.java, "claw")
        claw.scaleRange(0.0625, .75)
        var clawAngle: Double

        val colorLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE) // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // exclude blobs inside blobs
            .setRoi(
                ImageRegion.asUnityCenterCoordinates(
                    -0.9,
                    0.9,
                    0.9,
                    -0.9
                )
            ) // search central 1/4 of camera view
            .setDrawContours(true) // Show contours on the Stream Preview
            .setBlurSize(5) // Smooth the transitions between different colors in image
            .build()
        val portal = VisionPortal.Builder()
            .addProcessor(colorLocator)
            .setCameraResolution(Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()
        telemetry.msTransmissionInterval = 50 // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
        // Wait for the game to start (driver presses START)
        var offset: Boolean
        offset = false
        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            clawAngle = claw.position
            telemetry.addData("preview on/off", "... Camera Stream\n")

            // Read the current list
            val blobs = colorLocator.blobs

            ColorBlobLocatorProcessor.Util.filterByArea(
                50.0,
                40000.0,
                blobs
            ) // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
            if (blobs.isNotEmpty()) {
                val boxfit = blobs[0].boxFit
                var targetPos = claw.position
                var lastAngle: Double

               /* if (abs(boxfit.angle.toInt() - clawAngle) < 10) {
                    claw.position = clawAngle + boxfit.angle.toInt()
                }*/
                telemetry.addLine(" Area Density Aspect  Center")

                // Display the size (area) and center location for each Blob.

               // if (boxfit.angle >= 7.0 && 90.0 - boxfit.angle >= 7.0) {
                    telemetry.addLine(String.format("%6.1f FormerClawAngle", clawAngle))
                    clawAngle = clawAngle + boxfit.angle/180
                    telemetry.addLine(String.format("%6.1f ClawAngle", clawAngle))
                    telemetry.addLine(String.format("%6.1f BoxAngle", boxfit.angle/180))
                    lastAngle = boxfit.angle
                    val myBoxCorners = arrayOfNulls<Point>(4)
                    boxfit.points(myBoxCorners)
                    var boxAngle: Double
                    /* TODO: Take the points 0 & 3, subtract the x of 0 from the x of 3. if its negative use trig to find angle and use that.
                    TODO: If positive, do trig and subtract from 180.
                    */

                    if (myBoxCorners.isNotEmpty()) {
                        if (boxfit.boundingRect().height > boxfit.boundingRect().width) {
                            if (myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!! < 0) {

                                val trigRatio = myBoxCorners[0]?.y?.minus(myBoxCorners[3]?.y!!)
                                    ?.div(myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!!)
                                boxAngle = atan(trigRatio!!)
                                telemetry.addLine("<0")
                            } else if (myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!! > 0) {
                                val trigRatio = myBoxCorners[0]?.y?.minus(myBoxCorners[3]?.y!!)
                                    ?.div(myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!!)
                                val interAngle = (atan(trigRatio!!)) * 120
                                boxAngle = 180 + interAngle
                                telemetry.addLine(">0")
                                telemetry.addLine(
                                    String.format(
                                        "%6.1f Intermediate Angle",
                                        interAngle
                                    )
                                )
                            } else {
                                boxAngle = 0.0
                            }
                        } else {
                            if (myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!! < 0) {

                                val trigRatio = myBoxCorners[0]?.y?.minus(myBoxCorners[3]?.y!!)
                                    ?.div(myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!!)
                                boxAngle = atan(trigRatio!!)
                                telemetry.addLine("<0")
                            } else if (myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!! > 0) {
                                val trigRatio = myBoxCorners[0]?.y?.minus(myBoxCorners[3]?.y!!)
                                    ?.div(myBoxCorners[3]?.x?.minus(myBoxCorners[0]?.x!!)!!)
                                val interAngle = (atan(trigRatio!!)) * 120
                                boxAngle = 180 + interAngle
                                telemetry.addLine(">0")
                                telemetry.addLine(
                                    String.format(
                                        "%6.1f Intermediate Angle",
                                        interAngle
                                    )
                                )
                            } else {
                                boxAngle = 0.0
                            }
                        }
                        boxAngle /= 2
                        telemetry.addLine(String.format("%6.1f Calculated Box Angle", boxAngle))
                            /*if (abs(90 - boxAngle) <=7.0) {
                                targetPos = claw.position
                            } else if (boxAngle + (claw.position*180) >180) {
                                targetPos =  (boxAngle + (claw.position*180) - 180
                                        )/180
                            } else if ((claw.position*180 - boxAngle) >180) {
                                targetPos = (180 -  ((claw.position*180) - boxAngle))/180
                            } else {
                                targetPos = (boxAngle + claw.position*180)/180
                            }*/

                        targetPos = boxAngle/180
                        if (gamepad1.x) {
                            offset = true
                        } else if (gamepad1.y) {
                            offset = false
                        }
                        if (offset) {
                            if (targetPos - 0.5 <0){
                                targetPos += 0.5
                            }else if (targetPos + 0.5 > 1) {
                                targetPos -= 0.5
                            }
                        }


                    }
                    /*claw.position += 0.1
                    sleep(200)
                    if (boxfit.angle > lastAngle) {
                        claw.position += boxfit.angle / 90
                    } /*else if (boxfit.angle < lastAngle) {
                        targetPos = claw.position - (boxfit.angle/90)
                    }*/

                   // claw.position = clawAngle/2.00
                    if (gamepad1.dpad_up) {
                        claw.position = 0.0
                    } else if (gamepad1.dpad_down) {
                        claw.position = 1.0
                    }
                    /*if (boxfit.angle >= 45) {

                        targetPos+= ((90 - boxfit.angle) / 180)
                        if (targetPos > 1.0) {
                            targetPos -= 1.0
                        }
                    }else if (boxfit.angle < 45) {
                        targetPos -= ((boxfit.angle) / 180)
                        if (targetPos < 0.0) {
                            targetPos = 1.0 - targetPos
                        }
                    } */ */
                    claw.position = targetPos
                    telemetry.addLine(String.format("%6.1f TargetClawPosition", targetPos))
                    telemetry.addLine(String.format("%6.1f ClawPosition", claw.position))

                for (b in blobs) {
                    val boxFit = b.boxFit
                    telemetry.addLine(
                        String.format(
                            Locale.US,
                            "%6.1f,  %6.1f,   %6.1f,  (%3d,%3d), %6.1f",
                            b.contourArea.toDouble(),
                            b.density,
                            b.aspectRatio,
                            boxFit.center.x.toInt(),
                            boxFit.center.y.toInt(),
                            boxFit.angle

                        )
                    )
                }
                //sleep(20)
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: $runtime")

            telemetry.update()
        }

    }
}
