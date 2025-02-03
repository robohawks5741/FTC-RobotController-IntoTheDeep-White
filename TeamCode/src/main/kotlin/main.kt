import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.Point
import java.util.Locale
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.pow

@TeleOp(name = "#Main")
class main : LinearOpMode() {
    private val lift: DcMotorEx? = null
    private val leftRotate: DcMotorEx? = null
    private val rightRotate: DcMotorEx? = null
    private val rotate: Servo? = null
    private lateinit var clawDrive: CRServo
    private val right: Servo? = null
    private lateinit var distanceFront: Rev2mDistanceSensor

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private var aprilTag: AprilTagProcessor? = null
    private var colorLocator: ColorBlobLocatorProcessor? = null
    /**
     * The variable to store our instance of the vision portal.
     */
    var yAxisPower = 0.0
    var xAxisPower = 0.0
    var yawPower = 0.0
    private lateinit var claw: Servo
    private var visionPortal: VisionPortal? = null
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val twoDeadWheelLocalizer = TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870.0)


        initAprilTag()
        var yAxisPower = 0.0
        var xAxisPower = 0.0
        var yawPower = 0.0
        var tagBearing = 0.0
        var tagRange = 0.0
        val tagYaw = 0.0
        val tagY = 0.0
        val tagX = 0.0
        val lastHeading = 0.0
        var bPressed = false
        var bIsPressed = false
        clawDrive = hardwareMap.get(CRServo::class.java, "clawDrive")
        distanceFront = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceFront")
        waitForStart()
        if (opModeIsActive()) {
            var visionLocator = ConceptVisionColorLocator()
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            claw = hardwareMap.get(Servo::class.java, "claw")
            claw.scaleRange(0.0625, .75)
            var clawAngle: Double
            /*
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
                .build() */
           /* val portal = VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
                .build() */
            telemetry.msTransmissionInterval = 50 // Speed up telemetry updates, Just use for debugging.
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
            // Wait for the game to start (driver presses START)
            var offset: Boolean
            offset = false
            var alignClaw = false
            while (opModeIsActive()) { //Main loop
                /*drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));*/
                telemetryAprilTag()
                yAxisPower = gamepad1.left_stick_y.toDouble()
                xAxisPower = gamepad1.left_stick_x.toDouble()
                yawPower = gamepad1.right_stick_x.toDouble()
                var frontDist = distanceFront.getDistance(DistanceUnit.CM)
                var yaw = (twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw)%360
                if (frontDist < 30) {
                    telemetry.addLine(String.format("Close"))
                    if (yAxisPower < 0.00) {
                        yAxisPower = 0.00
                    }
                }else if (frontDist < 55) {
                    yAxisPower *= (frontDist/100)
                }
                else {
                    telemetry.addLine(String.format("Not Close"))
                }
                /*if (frontDist < 16) {

                    yAxisPower = 0.00
                    if (yaw < 45) {
                        driveToPosition(0.00, drive, twoDeadWheelLocalizer)
                    } else if (yaw < 135) {
                        driveToPosition(90.00, drive, twoDeadWheelLocalizer)
                    } else if (yaw < 225) {
                        driveToPosition(180.00, drive, twoDeadWheelLocalizer)
                    } else if (yaw < 315) {
                        driveToPosition(270.00, drive, twoDeadWheelLocalizer)
                    } else if (yaw <= 315) {
                        driveToPosition(0.00, drive, twoDeadWheelLocalizer)
                    }
                } */
                //Intake
                clawDrive.power = 1.00
                //lift
                val currentDetections: List<AprilTagDetection> = aprilTag!!.detections
                if (!gamepad1.b) {
                    for (detection in currentDetections) {
                        if (detection.id == 15 || detection.id == 11) {
                            tagBearing = detection.ftcPose.bearing
                            tagRange = detection.ftcPose.range
                        }
                    }
                }
                if (currentDetections.isEmpty()) {
                    tagBearing = 0.0
                    tagRange = 0.0
                }
                val absoluteY = (cos((drive.pose.heading.toDouble() - 90) + tagBearing) * tagRange)

                if (gamepad1.b) {
                    drive.setDrivePowers(
                        PoseVelocity2d(
                            Vector2d(
                                yAxisPower,
                                xAxisPower
                            ), tagBearing,
                        )
                    )
                }
                else {
                    drive.setDrivePowers(
                        PoseVelocity2d(
                            Vector2d(
                                yAxisPower,
                                xAxisPower
                            ), -yawPower,
                        )
                    )
                }
                drive.updatePoseEstimate()
                clawAngle = claw.position
                telemetry.addData("preview on/off", "... Camera Stream\n")

                // Read the current list
                val blobs = colorLocator?.blobs

                ColorBlobLocatorProcessor.Util.filterByArea(
                    50.0,
                    40000.0,
                    blobs
                ) // filter out very small blobs.

                ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
                if (blobs?.isNotEmpty() == true) {
                    val boxfit = blobs.get(0)?.boxFit
                    var targetPos = claw.position
                    var lastAngle: Double

                    /* if (abs(boxfit.angle.toInt() - clawAngle) < 10) {
                         claw.position = clawAngle + boxfit.angle.toInt()
                     }*/
                    telemetry.addLine(" Area Density Aspect  Center")

                    // Display the size (area) and center location for each Blob.

                    // if (boxfit.angle >= 7.0 && 90.0 - boxfit.angle >= 7.0) {
                    telemetry.addLine(String.format("%6.1f FormerClawAngle", clawAngle))

                    telemetry.addLine(String.format("%6.1f ClawAngle", clawAngle))
                    if (boxfit != null) {
                        telemetry.addLine(String.format("%6.1f BoxAngle", boxfit.angle/180))
                    }
                    if (boxfit != null) {
                        lastAngle = boxfit.angle
                    }
                    val myBoxCorners = arrayOfNulls<Point>(4)
                    if (boxfit != null) {
                        boxfit.points(myBoxCorners)
                    }
                    var boxAngle: Double
                    /* TODO: Take the points 0 & 3, subtract the x of 0 from the x of 3. if its negative use trig to find angle and use that.
                    TODO: If positive, do trig and subtract from 180.
                    */

                    if (gamepad1.b && bPressed == false) {
                        bPressed = true
                        bIsPressed = true
                    }
                    if (!gamepad1.b) {
                        bIsPressed = false
                    }
                    if (bPressed == true && bIsPressed == false) {

                    }
                    if (gamepad1.b && bPressed == true && bIsPressed == false) {
                         bPressed = false
                    }
                    if (myBoxCorners.isNotEmpty() && gamepad1.right_bumper) {
                        if (boxfit?.boundingRect()?.height!! > boxfit.boundingRect()!!.width) {
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

                telemetry.addLine(String.format("x", drive.pose.position.x))
                telemetry.addLine(String.format("y", drive.pose.position.y))
                telemetry.addLine(String.format("%6.1f Distance Front (CM)", frontDist))
                telemetry.addLine(
                    String.format(
                        "%6.1f heading (deg)",
                        Math.toDegrees(drive.pose.heading.toDouble())
                    )
                )
                telemetry.update()

                val packet = TelemetryPacket()
                packet.fieldOverlay().setStroke("#3F51B5")
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose)
                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }
        }
    }

    private fun driveToPosition(
        targYaw: Double,
        drive: MecanumDrive,
        twoDeadWheelLocalizer: TwoDeadWheelLocalizer
    ) {
        //var positionX = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble()
        //var positionY = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
        var yaw = twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw


        while (yaw != targYaw) {
            //positionX = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble()
            //positionY = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
            //yaw = twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw

            //val powerX: Double = 0.53 * ((abX - positionX) * -1).pow(1.0 / 11)
            //val powerY: Double = 0.53 * ((abY - positionY) * -1).pow(1.0 / 11)
            val powerYaw: Double = 0.53 * ((targYaw - yaw) * -1).pow(1.0/11)



            //xAxisPower = powerX
            //linearVelY = powerY
            yawPower = powerYaw

        }
    }

    private fun initAprilTag() {
        // Create the AprilTag processor.

        aprilTag =
            AprilTagProcessor.Builder() // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag?.setDecimation(3f)


        // Create the vision portal by using a builder.
        val builder = VisionPortal.Builder()

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK)
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(Size(640, 480))

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true)

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        colorLocator = ColorBlobLocatorProcessor.Builder()
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
        builder.addProcessor(aprilTag)
        builder.addProcessor(colorLocator)

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build()

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal?.setProcessorEnabled(aprilTag, true)
    } // end method initAprilTag()

    private fun telemetryAprilTag() {
        val currentDetections: List<AprilTagDetection> = aprilTag!!.detections

        telemetry.addData("# AprilTags Detected", currentDetections.size)

        // Step through the list of detections and display info for each one.
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(
                    String.format(
                        "\n==== (ID %d) %s",
                        detection.id,
                        detection.metadata.name
                    )
                )
                telemetry.addLine(
                    String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                    )
                )
                telemetry.addLine(
                    String.format(
                        "PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw
                    )
                )
                telemetry.addLine(
                    String.format(
                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                    )
                )
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                telemetry.addLine(
                    String.format(
                        "Center %6.0f %6.0f   (pixels)",
                        detection.center.x,
                        detection.center.y
                    )
                )
            }
        } // end for() loop


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)")
        telemetry.addLine("RBE = Range, Bearing & Elevation")
    } // end method telemetryAprilTag()

    companion object {
        private const val USE_WEBCAM = true // true for webcam, false for phone camera
    }
}
