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
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
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
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.pow

@Autonomous(name = "#Auto Blue Left")
class Auto2BL : LinearOpMode() {

    private val leftRotate: DcMotorEx? = null
    private val rightRotate: DcMotorEx? = null
    private val rotate: Servo? = null
    private val left: Servo? = null
    private val right: Servo? = null
    private lateinit var lift: DcMotorEx
    private lateinit var clawDrive: CRServo
    /*
     * The variable to store our instance of the AprilTag processor.
     */


    private var aprilTag: AprilTagProcessor? = null
    private var colorLocator: ColorBlobLocatorProcessor? = null
    private var colorLocator2: ColorBlobLocatorProcessor? = null

    /**
     * The variable to store our instance of the vision portal.
     */
    private lateinit var claw: Servo
    private var visionPortal: VisionPortal? = null
    private var visionPortal2: VisionPortal? = null
    var linearVelX: Double = 0.0
    var linearVelY: Double = 0.0
    var angularVel: Double = 0.0
    private var startX: Double = 0.0
    private var startY: Double = 0.0
    private var startYaw: Double = 0.0
    var positionX: Double = 0.0
    var positionY: Double = 0.0



    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val twoDeadWheelLocalizer = TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870.0)
        positionX = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX
        positionY = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startY
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 50
        lift = hardwareMap.get(DcMotorEx::class.java, "lift")
        clawDrive = hardwareMap.get(CRServo::class.java, "clawDrive")
        clawDrive.direction = DcMotorSimple.Direction.FORWARD
        lift.targetPosition = lift.currentPosition
        lift.mode = DcMotor.RunMode.RUN_TO_POSITION
        lift.direction = DcMotorSimple.Direction.FORWARD
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.power = 1.0

        val liftStart = lift.currentPosition


        var liftExt = lift.currentPosition - liftStart

        var clawAngle: Double
        initAprilTag()

        var tagBearing = 0.0
        var tagRange = 0.0
        val tagYaw = 0.0
        val tagY = 0.0
        val tagX = 0.0
        val lastHeading = 0.0
        startX = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
        startY = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble()
        startYaw = twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw
        var targetX: Double
        var targetY: Double

        var offset = false
        var contingency = true
        waitForStart()
        resetRuntime()
        if (opModeIsActive()) {
            while (opModeIsActive()) { //Main loop

                //   telemetryAprilTag()


                // driveToPosition(0.0, 10000.0, 0.00, drive, twoDeadWheelLocalizer)

                //Lift

                telemetry.addLine(
                    String.format(
                        "%6.1f lift EXT prev",
                        lift.currentPosition.toDouble()
                    )
                )

                telemetry.addLine(String.format("%6.1f lift EXT", liftExt.toDouble()))

                if (contingency) {
                    driveToPosition(0.0, 2000.0, drive, twoDeadWheelLocalizer)
                    driveToPosition(8000.0, 2000.0, drive, twoDeadWheelLocalizer)
                    driveToPosition(8000.0, 50.0, drive, twoDeadWheelLocalizer)
                } else {

                    clawDrive.power = 1.0
                    sleep(1000)
                    // driveToPosition(-6000.00, 0.00, drive, twoDeadWheelLocalizer)
                    driveToAngle(45.00, drive, twoDeadWheelLocalizer)
                    lift(-8400, liftStart)
                }



                //Makes sure the lift does not exceed the maximum extension and start making expensive noises



            }
        }
    }
    private fun lift(ext: Int, start: Int) {
        if (ext < -8400 || ext > 0) {
            lift.targetPosition = ext + start
        }
        telemetry.addLine(String.format("%6.1f lift EXT", lift.targetPosition.toDouble()))

    }
    private fun alignClaw () {
        val clawAngle: Double = claw.position
        val blobs = arrayListOf<ColorBlobLocatorProcessor.Blob>()
        blobs.addAll(colorLocator?.blobs!!)
        blobs.addAll(colorLocator2?.blobs!!)
        ColorBlobLocatorProcessor.Util.filterByArea(
            50.0,
            40000.0,
            blobs
        ) // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
        if (blobs.isNotEmpty()) {
            val boxfit = blobs.get(0)?.boxFit
            var targetPos = claw.position
            telemetry.addLine(" Area Density Aspect  Center")

            // Display the size (area) and center location for each Blob.

            // if (boxfit.angle >= 7.0 && 90.0 - boxfit.angle >= 7.0) {
            telemetry.addLine(String.format("%6.1f FormerClawAngle", clawAngle))

            telemetry.addLine(String.format("%6.1f ClawAngle", clawAngle))
            if (boxfit != null) {
                telemetry.addLine(String.format("%6.1f BoxAngle", boxfit.angle / 180))
            }
            val myBoxCorners = arrayOfNulls<Point>(4)
            if (boxfit != null) {
                boxfit.points(myBoxCorners)
            }
            var boxAngle: Double

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
                targetPos = boxAngle / 180



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

            //driveToPosition(-6000.00, 500.00, drive, twoDeadWheelLocalizer)

            // drive.setDrivePowers(PoseVelocity2d(Vector2d(-linearVelY,-linearVelX), -0.25*(angularVel)))
        }
    }
    fun driveToPosition(
        abX: Double,
        abY: Double,
        drive: MecanumDrive,
        twoDeadWheelLocalizer: TwoDeadWheelLocalizer
    ) {


        var powerX: Double
        var powerY: Double
        //var powerYaw: Double

        while (abs(positionX-abX) < 100|| abs(positionY-abY) < 100) {
            positionX =
                (twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX)/10
            positionY =
                (twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startY)/10




                //Tolerance of 20
            if (abs(positionX-abX) >= 20) {
                //Stops the math from breaking
                if ((positionX - abX) <= 0) {
                    powerX = -0.03 * (abs(positionX - abX)).pow(1.0 / 3)
                } else {
                    powerX = 0.03 * (abs(positionX - abX)).pow(1.0 / 3)
                }
            } else {
                powerX = 0.0
            }

            //Tolerance of 20
            if (abs(positionY-abY) >=20) {
                //Stops the math from breaking
                if ((positionY - abY) <= 0) {
                    powerY = -0.025 * (abs(positionY - abY)).pow(1.0 / 3)
                } else {
                    powerY = 0.025 * (abs(positionY - abY)).pow(1.0 / 3)
                }
            }else {
                powerY = 0.0
            }



            //powerYaw = 0.0 //temp for testing
            //powerY = 0.0
            //Sends the calculated power values to the inputs for the setDrivePowers
            linearVelX = (-1) * powerX
            linearVelY = (-1) * powerY

            telemetry.addLine(String.format("%6.1f Power X", powerX))
            telemetry.addLine(String.format("%6.1f Power Y", powerY))
            telemetry.addLine(String.format("%6.1f MotorPow LF", (drive.leftFront.getCurrent(CurrentUnit.MILLIAMPS))))
            telemetry.addLine(String.format("%6.1f MotorPow LB", drive.leftBack.getCurrent(CurrentUnit.MILLIAMPS)))
            telemetry.addLine(String.format("%6.1f MotorPow RF", drive.rightFront.getCurrent(CurrentUnit.MILLIAMPS)))
            telemetry.addLine(String.format("%6.1f MotorPow RB", drive.rightBack.getCurrent(CurrentUnit.MILLIAMPS)))
            telemetry.addLine(String.format("%6.1f Pos X", positionX))
            telemetry.addLine(String.format("%6.1f Pos Y", positionY))
            telemetry.addLine(String.format("%6.1f Targ X", abX))
            telemetry.addLine(String.format("%6.1f Targ Y", abY))


            runTelemetry(drive, twoDeadWheelLocalizer)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(linearVelY, linearVelX), 0.00))
            if (abs(abX-positionX) < 50  && abs(abY-positionY) < 50){
                break
            }



        }
    }
    private fun driveToAngle(
        targYaw: Double,
        drive: MecanumDrive,
        twoDeadWheelLocalizer: TwoDeadWheelLocalizer,
    ) {
        //Current Yaw
        var yaw = twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw //- startYaw
        var powerYaw: Double

        while (abs(targYaw-yaw) >=10) {
            yaw = twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw// - startYaw

            //Power curve
            powerYaw = if (abs(targYaw-yaw) >=10) {
                if ((targYaw - yaw) <= 0) {
                    -0.05 * (abs(targYaw - yaw)).pow(1.0 / 3)
                } else {
                    0.05 * (abs(targYaw - yaw)).pow(1.0 / 3)
                }
            }else {
                0.0
            }
            angularVel = powerYaw*5
            telemetry.addLine(String.format("%6.1f Pos Yaw", yaw))
            telemetry.addLine(String.format("%6.1f Targ Yaw", targYaw))
            telemetry.addLine(String.format("%6.1f Power Yaw", powerYaw))
            telemetry.addLine(String.format("%6.1f Angular Vel", angularVel))
            runTelemetry(drive, twoDeadWheelLocalizer)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.00,0.00), angularVel))
        }
        startX = (twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX)/10
        startY = (twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startX)/10
    }
    private fun runTelemetry(drive: MecanumDrive, twoDeadWheelLocalizer: TwoDeadWheelLocalizer) {
        telemetry.addLine(String.format("%6.1f time", runtime))
        telemetry.addLine(String.format("%6.1f Diffx", ((100 - drive.pose.position.x) * -1) / 1000))
        //telemetry.addLine(String.format("%6.1f Pose X", drive.pose.position.x));
        telemetry.addLine(String.format("%6.1f Pose X", twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX))
        telemetry.addLine(String.format("%6.1f Pose Y", twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startY))
        telemetry.addLine(String.format("%6.1f Linear Vel X", linearVelX))
        telemetry.addLine(String.format("%6.1f Linear Vel Y", linearVelY))
        telemetry.addLine(String.format("%6.1f Yaw", (Math.toDegrees((twoDeadWheelLocalizer.imu.robotYawPitchRollAngles.yaw).toDouble()))%360))
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
    private fun initAprilTag() {
        // Create the AprilTag processor.



        //2nd builder for claw
        val builder2 = VisionPortal.Builder()

        // Set the camera (webcam vs. built-in RC phone camera).

        builder2.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))

        builder2.setCamera(BuiltinCameraDirection.BACK)



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
        colorLocator2 = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.YELLOW) // use a predefined color match
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

        builder2.addProcessor(colorLocator)
        builder2.addProcessor(colorLocator2)

        // Build the Vision Portal, using the above settings.

        visionPortal2 = builder2.build()

        // Disable or re-enable the aprilTag processor at any time.

        visionPortal2?.setProcessorEnabled(colorLocator, true)
        visionPortal2?.setProcessorEnabled(colorLocator2, true)
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
