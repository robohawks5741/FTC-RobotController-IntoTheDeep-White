import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.pow

@Autonomous(name = "#Auto Main Kt")
class Auto2 : LinearOpMode() {
    private val lift: DcMotorEx? = null
    private val leftRotate: DcMotorEx? = null
    private val rightRotate: DcMotorEx? = null
    private val rotate: Servo? = null
    private val left: Servo? = null
    private val right: Servo? = null
    private val distanceFront: Rev2mDistanceSensor? = null

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private var aprilTag: AprilTagProcessor? = null

    /**
     * The variable to store our instance of the vision portal.
     */
    private var visionPortal: VisionPortal? = null

    var linearVelX: Double = 0.0
    var linearVelY: Double = 0.0
    var angularVel: Double = 0.0

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val twoDeadWheelLocalizer = TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870.0)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        initAprilTag()

        var tagBearing = 0.0
        var tagRange = 0.0
        val tagYaw = 0.0
        val tagY = 0.0
        val tagX = 0.0
        val lastHeading = 0.0
        val startX = drive.pose.position.x
        val startY = drive.pose.position.y
        var targetX: Double
        var targetY: Double
        var frontDist = distanceFront?.getDistance(DistanceUnit.CM)
        waitForStart()
        resetRuntime()
        if (opModeIsActive()) {
            while (opModeIsActive()) { //Main loop

                telemetryAprilTag()

                driveToPosition(100.0, 0.0, drive, twoDeadWheelLocalizer)

                driveToPosition(0.0, 100.0, drive, twoDeadWheelLocalizer)

                //Lift
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



                //double absoluteY = (Math.cos((drive.pose.heading.toDouble() - 90) + tagBearing)*tagRange);
                if (runtime <= 3.00) {
                    linearVelX = 1.0
                }

                //Final Drive Inputs
                drive.setDrivePowers(
                    PoseVelocity2d(
                        Vector2d(linearVelX, linearVelY),
                        angularVel
                    )
                ) //End of function call
            }
        }
    }

    private fun driveToPosition(
        abX: Double,
        abY: Double,
        drive: MecanumDrive,
        twoDeadWheelLocalizer: TwoDeadWheelLocalizer
    ) {
        var positionX = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble()
        var positionY = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()

        while (positionX != abX || positionY != abY) {
            positionX = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble()
            positionY = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()

            val powerX: Double = 0.53 * ((abX - positionX) * -1).pow(1.0 / 11)
            val powerY: Double = 0.53 * ((abY - positionY) * -1).pow(1.0 / 11)

            linearVelX = powerX
            linearVelY = powerY
            runTelemetry(drive)


        }
    }

    private fun runTelemetry(drive: MecanumDrive) {
        telemetry.addLine(String.format("%6.1f time", runtime))
        telemetry.addLine(String.format("%6.1f Diffx", ((100 - drive.pose.position.x) * -1) / 1000))
        //telemetry.addLine(String.format("%6.1f Pose X", drive.pose.position.x));
        telemetry.addLine(String.format("%6.1f Pose X", drive.pose.position.x))
        telemetry.addLine(String.format("%6.1f Pose Y", drive.pose.position.y))
        telemetry.addLine(
            String.format(
                "%6.1f heading (deg)",
                Math.toDegrees(drive.pose.heading.toDouble())
            )
        )
        telemetryAprilTag()
        telemetry.update()
        val packet = TelemetryPacket()
        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
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
        builder.addProcessor(aprilTag)

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
