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
import org.opencv.core.Point
import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.pow

@Autonomous(name = "#Auto Red Right")
class Auto2RL : LinearOpMode() {

    private val leftRotate: DcMotorEx? = null
    private val rightRotate: DcMotorEx? = null
    private val rotate: Servo? = null
    private val left: Servo? = null
    private val right: Servo? = null
    private lateinit var distanceFront: Rev2mDistanceSensor
    private lateinit var lift: DcMotorEx
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


    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val twoDeadWheelLocalizer = TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870.0)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 50
        distanceFront = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceFront")
        lift = hardwareMap.get(DcMotorEx::class.java, "lift")
        lift.targetPosition = lift.currentPosition
        lift.mode = DcMotor.RunMode.RUN_TO_POSITION
        lift.direction = DcMotorSimple.Direction.FORWARD
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.power = 1.0
        val liftStart = lift.currentPosition


        var liftExt = lift.currentPosition - liftStart


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
        var frontDist = distanceFront.getDistance(DistanceUnit.CM)

        waitForStart()
        resetRuntime()
        if (opModeIsActive()) {
            while (opModeIsActive()) { //Main loop

             //   telemetryAprilTag()



               // driveToPosition(0.0, 10000.0, 0.00, drive, twoDeadWheelLocalizer)

                //Lift

                telemetry.addLine(String.format("%6.1f lift EXT prev", lift.currentPosition.toDouble()))

                telemetry.addLine(String.format("%6.1f lift EXT", liftExt.toDouble()))

                val currentDetections: List<AprilTagDetection> = aprilTag!!.detections

                for (detection in currentDetections) {
                    if (detection.id == 15 || detection.id == 11) {
                        tagBearing = detection.ftcPose.bearing
                        tagRange = detection.ftcPose.range
                    }
                }

                if (currentDetections.isEmpty()) {
                    tagBearing = 0.0
                    tagRange = 0.0
                }

               // driveToPosition(-6000.00, 0.00, drive, twoDeadWheelLocalizer)
                driveToAngle(45.00, drive,twoDeadWheelLocalizer)
                liftExt = -8400

                //Makes sure the lift does not exceed the maximum extension and start making expensive noises
                if (liftExt < -8400 || liftExt > 0) {
                    lift.targetPosition = liftExt + liftStart
                }
                telemetry.addLine(String.format("%6.1f lift EXT", lift.targetPosition.toDouble()))

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
                        telemetry.addLine(String.format("%6.1f BoxAngle", boxfit.angle/180))
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

                //driveToPosition(-6000.00, 500.00, drive, twoDeadWheelLocalizer)

               // drive.setDrivePowers(PoseVelocity2d(Vector2d(-linearVelY,-linearVelX), -0.25*(angularVel)))
            }
        }
    }

    fun driveToPosition(
        abX: Double,
        abY: Double,
        drive: MecanumDrive,
        twoDeadWheelLocalizer: TwoDeadWheelLocalizer
    ) {
        var positionX = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX
        var positionY = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startY

        var powerX: Double
        var powerY: Double
        //var powerYaw: Double
        var frontDist = distanceFront.getDistance(DistanceUnit.CM)

        while (abs(positionX-abX) < 100|| abs(positionY-abY) < 100) {
            positionX =
                (twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX)/10
            positionY =
                (twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startY)/10

            telemetry.addLine(String.format("%6.1f Front Distance Sensor", frontDist))
            //val powerX: Double = (0.53 * (((abX - positionX) ).pow(1.0 / 11)))
            //val powerY: Double = (0.53 * (((abY - positionY) ).pow(1.0 / 11)))
            frontDist = 30.0 //for testing purposes TODO:Remove this once distance sensor works
            //Sets the power to 0 if the front distance is below 20CM
            if (frontDist <= 20) {
                powerX = 0.00
                powerY = 0.00

            } else {
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
            if (abs(abX-positionX) <50  && abs(abY-positionY) < 50){
                break
            }



        }
    }
    fun driveToAngle(
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

    }
    fun runTelemetry(drive: MecanumDrive, twoDeadWheelLocalizer: TwoDeadWheelLocalizer) {
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

    fun initAprilTag() {
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

        val builder2 = VisionPortal.Builder()

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder2.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        } else {
            builder2.setCamera(BuiltinCameraDirection.BACK)
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(Size(320, 240))

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
