import android.annotation.SuppressLint
import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.Encoder
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
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
import kotlin.math.cos
import kotlin.math.pow

@TeleOp(name = "#Main")
class Config : LinearOpMode() {

    private val leftRotate: DcMotorEx? = null
    private val rightRotate: DcMotorEx? = null
    private val rotate: Servo? = null
    private val right: Servo? = null
    private lateinit var distanceFrontLeft: Rev2mDistanceSensor
    private lateinit var distanceFrontRight: Rev2mDistanceSensor
    private lateinit var distanceBackLeft: Rev2mDistanceSensor
    private lateinit var distanceBackRight: Rev2mDistanceSensor

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private var aprilTag: AprilTagProcessor? = null
    private var colorLocator: ColorBlobLocatorProcessor? = null
    private var colorLocator2: ColorBlobLocatorProcessor? = null
    /**
     * The variable to store our instance of the vision portal.
     */
    var yAxisPower = 0.0
    var xAxisPower = 0.0
    var yawPower = 0.0
    private lateinit var claw: Servo
    private lateinit var clawDrive: CRServo
    private lateinit var linkArmL: Servo
    private lateinit var linkArmR: Servo
    private lateinit var lift: DcMotorEx
    private var visionPortal1: VisionPortal? = null
    private var visionPortal2: VisionPortal? = null
    private var offset: Boolean = false


    @SuppressLint("DefaultLocale")
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
        clawDrive.direction = DcMotorSimple.Direction.FORWARD
        linkArmL = hardwareMap.get(Servo::class.java, "linkArmL")
        linkArmR = hardwareMap.get(Servo::class.java, "linkArmR")
        distanceFrontLeft = hardwareMap.get(Rev2mDistanceSensor::class.java, "distFrontLeft")
        distanceFrontRight = hardwareMap.get(Rev2mDistanceSensor::class.java, "distFrontRight")
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor::class.java, "distBackLeft")
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor::class.java, "distBackRight")
        claw = hardwareMap.get(Servo::class.java, "claw")

        waitForStart()
        if (opModeIsActive()) {
            var visionLocator = ConceptVisionColorLocator()
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).


            claw.scaleRange(0.0625, .75)
            lift = hardwareMap.get(DcMotorEx::class.java, "lift")
            lift.targetPosition = lift.currentPosition
            lift.mode = DcMotor.RunMode.RUN_TO_POSITION
            lift.direction = DcMotorSimple.Direction.FORWARD
            lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            lift.power = 1.0
            val liftStart = lift.currentPosition
            lift.targetPosition = -10
            linkArmR.direction = Servo.Direction.FORWARD
            linkArmL.direction = Servo.Direction.REVERSE
            linkArmR.scaleRange(0.0, 0.75)
            linkArmL.scaleRange(0.0, 0.75)
            var liftExt = lift.currentPosition - liftStart


            var clawAngle: Double
            claw.position = 0.5
            telemetry.msTransmissionInterval = 50 // Speed up telemetry updates, Just use for debugging.
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
            // Wait for the game to start (driver presses START)

            var alignClaw = false
            while (opModeIsActive()) { //Main loop
                val FLDist = distanceFrontLeft.getDistance(DistanceUnit.CM)
                val FRDist = distanceFrontRight.getDistance(DistanceUnit.CM)
                val BLDist = distanceBackLeft.getDistance(DistanceUnit.CM)
                val BRDist = distanceBackRight.getDistance(DistanceUnit.CM)
                telemetry.addLine(
                    String.format(
                    "FL:%6.1f, FR:%6.1f, BL:%6.1f, BR:%6.1f",
                        FLDist,
                        FRDist,
                        BLDist,
                        BRDist
                ))

                telemetry.addLine(String.format("%6.1f lift EXT prev", liftExt.toDouble()))

                    if (gamepad2.dpad_up) {
                        liftExt = -10000
                    } else if (gamepad2.dpad_down) {
                        liftExt = -10
                    }


                //automatic intake macro
                //set lift to low position and intake drive wheels to full power


                telemetry.addLine(String.format("%6.1f lift EXT", liftExt.toDouble()))
                if (liftExt >= -10000 && liftExt <= -10) {
                    lift.targetPosition = liftExt + liftStart
                }
                telemetry.addLine(String.format("%6.1f lift EXT", lift.targetPosition.toDouble()))
                if (gamepad2.dpad_left && liftExt > -3000) {
                    linkArmL.position = 0.0
                    linkArmR.position = linkArmL.position
                } else if (gamepad2.dpad_right && liftExt > -3000) {
                    linkArmL.position = 1.0
                    linkArmR.position = linkArmL.position
                }
                if (liftExt < -3000) {
                    linkArmL.position = 0.0
                    linkArmR.position = linkArmL.position
                }
               // telemetryAprilTag()
                yAxisPower = gamepad1.left_stick_y.toDouble()
                xAxisPower = gamepad1.left_stick_x.toDouble()
                yawPower = gamepad1.right_stick_x.toDouble()


                //Intake
                if (gamepad2.right_trigger > 0.5) {
                    clawDrive.power = gamepad2.right_trigger.toDouble()
                } else if (gamepad2.left_trigger > 0.5) {
                    clawDrive.power = -gamepad2.left_trigger.toDouble()
                } else {
                    clawDrive.power = 0.0
                }
                //distance control
                if (!gamepad1.right_bumper) {
                    if (FLDist <= 30 && FRDist <= 30) {
                        telemetry.addLine("FL&FR")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                    }else if (FLDist <= 30 && FRDist >= 30) {
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                        yawPower = -0.5
                    }else if (FLDist >= 30 && FRDist <= 30) {
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                        yawPower = 0.5
                    }
                } else if (gamepad1.right_bumper) {
                    //Prox Limit
                    if (FLDist <= 30 && FRDist <= 30) {
                        telemetry.addLine("FL&FR")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                    }else if (FLDist <= 30 && FRDist >= 30) { //Front left prox limit
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                        xAxisPower = -0.5
                    }else if (FLDist >= 30 && FRDist <= 30) { //Front right prox limit
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                        xAxisPower = 0.5
                    }
                }

                //send final drive powers to the setDrivePowers function
                drive.setDrivePowers(
                    PoseVelocity2d(
                        Vector2d(
                            yAxisPower,
                            xAxisPower
                        ), -yawPower,
                    )
                )
                telemetry.addLine(String.format("%6.1f", lift.getCurrent(CurrentUnit.AMPS)))

                drive.updatePoseEstimate()
                clawAngle = claw.position
                telemetry.addData("preview on/off", "... Camera Stream\n")

                // Read the current list
                runBlobs(clawAngle)

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: $runtime")

                telemetry.update()

                telemetry.addLine(String.format("x", drive.pose.position.x))
                telemetry.addLine(String.format("y", drive.pose.position.y))
                //telemetry.addLine(String.format("%6.1f Distance Front (CM)", frontLeftDist))
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
    private fun runBlobs(clawAngle:Double) {
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

            if (myBoxCorners.isNotEmpty() && gamepad2.right_bumper) {
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
                if (gamepad2.x) {
                    offset = true
                } else if (gamepad2.y) {
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

        //2nd builder for claw
        val builder2 = VisionPortal.Builder()
        val builderApril = VisionPortal.Builder()

       //Select cameras for the builders (Webcam 1 = claw, Webcam 2 = forward/backwards facing

        builder2.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        builderApril.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 2"))
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
            .setBlurSize(5) // Smooth the transitions between different colors in image]
            .build()
        aprilTag =
            AprilTagProcessor.Builder() // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
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

        builderApril.setCameraResolution(Size(1280, 720))

        builderApril.addProcessor(aprilTag)
        builder2.addProcessor(colorLocator)
        builder2.addProcessor(colorLocator2)


        // Build the Vision Portal, using the above settings.

        visionPortal2 = builder2.build()
        visionPortal1 = builderApril.build()

        // Disable or re-enable the aprilTag processor at any time.

        visionPortal2?.setProcessorEnabled(colorLocator, true)
        visionPortal2?.setProcessorEnabled(colorLocator2, true)
        visionPortal1?.setProcessorEnabled(aprilTag, true)
    } // end method initAprilTag()


}
