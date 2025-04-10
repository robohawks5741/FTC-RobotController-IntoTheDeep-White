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
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
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
class main : LinearOpMode() {

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
       // val twoDeadWheelLocalizer = TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870.0)
        val threeDeadWheelLocalizer = ThreeDeadWheelLocalizer(hardwareMap, 0.00144943115234375)

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
        distanceFrontLeft = hardwareMap.get(Rev2mDistanceSensor::class.java, "distFL")
        distanceFrontRight = hardwareMap.get(Rev2mDistanceSensor::class.java, "distFR")
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor::class.java, "distBL")
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor::class.java, "distBR")
        claw = hardwareMap.get(Servo::class.java, "claw")
        claw.scaleRange(0.08, .375)

        var par0Pos: Double = 0.0
        var par1Pos: Double = 0.0
        var perpPos: Double = 0.0
        claw.position = 1.0
        waitForStart()
        if (opModeIsActive()) {
            var visionLocator = ConceptVisionColorLocator()
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).


            lift = hardwareMap.get(DcMotorEx::class.java, "lift")
            lift.targetPosition = lift.currentPosition
            lift.mode = DcMotor.RunMode.RUN_TO_POSITION
            lift.direction = DcMotorSimple.Direction.FORWARD
            lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            lift.power = 0.75
            val liftStart = lift.currentPosition
            lift.targetPosition = -10 //- liftStart
            linkArmR.direction = Servo.Direction.FORWARD
            linkArmL.direction = Servo.Direction.REVERSE
            linkArmR.scaleRange(0.1875, .9)
            linkArmL.scaleRange(0.1875, .9)
            var liftExt = 0//- liftStart


            var clawAngle: Double
            claw.position = 1.0
            telemetry.msTransmissionInterval = 5 // Speed up telemetry updates, Just use for debugging.
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE)
            // Wait for the game to start (driver presses START)

            var alignClaw = false
            while (opModeIsActive()) { //Main loop
                val FLDist = distanceFrontLeft.getDistance(DistanceUnit.CM)
                val FRDist = distanceFrontRight.getDistance(DistanceUnit.CM)
                val BLDist = distanceBackLeft.getDistance(DistanceUnit.CM)
                val BRDist = distanceBackRight.getDistance(DistanceUnit.CM)
                par0Pos = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
                par1Pos = threeDeadWheelLocalizer.par0.getPositionAndVelocity().position.toDouble()
                perpPos = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
                telemetry.addLine(String.format("claw position %6.1f", claw.position))
                telemetry.addLine(
                    String.format(
                        "FL:%6.1f, FR:%6.1f, BL:%6.1f, BR:%6.1f",
                        FLDist,
                        FRDist,
                        BLDist,
                        BRDist
                    )
                )

                telemetry.addLine(String.format("%6.1f lift EXT prev", liftExt.toDouble()))
                telemetry.addLine(
                    String.format(
                        "%6.1f par0, %6.1f par1, %6.1f perp",
                        par0Pos,
                        par1Pos,
                        perpPos
                    )
                )
                if (gamepad2.x) {
                    if (gamepad2.dpad_up && liftExt - 100 >= -9800) {
                        //liftExt = -2000
                        liftExt -= 100
                    } else if (gamepad2.dpad_down && liftExt <= 100) {
                        //liftExt = -100
                        liftExt += 100
                    }
                } else if (gamepad2.y) {
                    if (gamepad2.dpad_up) {
                        liftExt = -700
                    } else if (gamepad2.dpad_down) {
                        liftExt = -20
                    }
                }else if (gamepad2.right_stick_button) {
                    if (gamepad2.dpad_up) {
                        liftExt = -2100
                    }
                }else {
                    if (gamepad2.dpad_up && !gamepad2.right_bumper) {
                        liftExt = -9800
                    } else if (gamepad2.dpad_up && gamepad2.right_bumper){
                        liftExt = -4800
                    }else if (gamepad2.dpad_down) {
                        liftExt = -20
                    }
                }
                if (gamepad2.left_bumper && gamepad2.dpad_down) {
                    liftExt = lift.currentPosition
                }

                //automatic intake macro
                //set lift to low position and intake drive wheels to full power
               /* if (gamepad2.left_bumper) {
                    if ((liftExt - 100) >= -10000 && gamepad2.dpad_up) {
                        liftExt -= 100
                        //liftExt = -10000
                    } else if ((liftExt + 100) <= -10 && gamepad2.dpad_down) {
                        liftExt += 100
                        //liftExt = -10
                    }
                } else {
                    if (gamepad2.dpad_down) {
                        var DsequenceComplete = false
                        while (DsequenceComplete != false) {
                            clawDrive.power = 1.0
                            liftExt = -10
                            if (abs((lift.currentPosition - liftStart) - 100) < 5 || gamepad2.dpad_up) {
                                DsequenceComplete = true
                            }
                        }
                    } else if (gamepad2.dpad_up) {
                        var UsequenceComplete = false
                        while (UsequenceComplete != false) {
                            clawDrive.power = 0.0
                            liftExt = -10000
                            if (lift.currentPosition - liftStart == -10000 || gamepad2.dpad_down) {
                                UsequenceComplete = true
                            }
                        }
                    }
                }*/

                telemetry.addLine(String.format("%6.1f lift EXT", liftExt.toDouble()))
                telemetry.addLine(String.format("%6.1f lift EXTAug", liftExt.toDouble() - liftStart))

                if (liftExt >= -9800 && liftExt <= 100) {
                    lift.targetPosition = liftExt - liftStart
                }
                /*if (lift.getCurrent(CurrentUnit.AMPS) > 5) {
                    lift.power = 0.0
                    liftExt = lift.currentPosition - liftStart
                } else {
                    lift.power = 0.75
                }*/
              //  telemetry.addLine(String.format("%6.1f Lift Amps", lift.getCurrent(CurrentUnit.MILLIAMPS)))
                telemetry.addLine(String.format("%6.1f lift AMP", lift.getCurrent(CurrentUnit.MILLIAMPS)))
                telemetry.addLine(String.format("%6.1f lift CurrEXT", lift.currentPosition.toDouble()))
                telemetry.addLine(String.format("%6.1f lift CurrEXTAug", lift.currentPosition.toDouble() - liftStart))
                telemetry.addLine(String.format("%6.1f lift TargEXT", lift.targetPosition.toDouble()))
                //LEFT AND RIGHT ARMS ARE SWITCHED
                if (gamepad2.x) {
                    if (gamepad2.dpad_left && liftExt > -3000 && linkArmL.position + 0.1 <= 1.1) {
                        linkArmL.position -= 0.1
                        linkArmR.position += 0.1
                    } else if (gamepad2.dpad_right && liftExt > -3000 && linkArmL.position - 0.1 >= 0) {
                        linkArmL.position += 0.1
                        linkArmR.position -= 0.1
                    }
                } else {
                    if (gamepad2.dpad_left && liftExt > -3000) {
                        linkArmL.position = 0.0
                        linkArmR.position = 1.0
                    } else if (gamepad2.dpad_right && liftExt > -3000) {
                        linkArmL.position = 1.0
                        linkArmR.position = 0.0
                    }
                }
                if (liftExt < -3000) {
                    linkArmL.position = 0.75
                    linkArmR.position = 0.75
                }
               // telemetryAprilTag()
                yAxisPower = gamepad1.left_stick_y.toDouble()
                xAxisPower = gamepad1.left_stick_x.toDouble()
                yawPower = gamepad1.right_stick_x.toDouble()

                if (lift.currentPosition < -4810) {
                    yAxisPower *= 0.25
                    xAxisPower *= 0.25
                    yawPower *= 0.25
                }
                //Intake
                if (gamepad2.right_trigger > 0.5) {
                    clawDrive.power = gamepad2.right_trigger.toDouble()
                } else if (gamepad2.left_trigger > 0.5) {
                    clawDrive.power = -gamepad2.left_trigger.toDouble()
                } else {
                    clawDrive.power = 0.0
                }
                var yaw = (par0Pos-par1Pos)/10
                telemetry.addLine(String.format("%6.1f par0, %6.1f par1, %6.1f perp", par0Pos, par1Pos, perpPos))
                val proxLimit = 30 //CM
                //distance control
                if (!gamepad1.right_bumper && gamepad1.left_bumper) {
                    if (FLDist <= proxLimit && FRDist <= proxLimit) {
                        telemetry.addLine("FL&FR")
                        /*if (yAxisPower < 0.0) {
                            yAxisPower = 0.0
                        }*/
                        xAxisPower *= 0.5
                    }else if (FLDist <= (proxLimit) && FRDist >= (proxLimit)) {
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {

                        }
                        yawPower = -0.5
                    }else if (FLDist >= (proxLimit) && FRDist <= (proxLimit)) {
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {

                        }
                        yawPower = 0.5
                    } else if (FLDist <= proxLimit && FRDist <= proxLimit) {
                        if (yAxisPower > 0.0) {
                         //   yAxisPower *= 0.5
                        }
                    }
                } else if (gamepad1.right_bumper) {

                    //Prox Limit
                    if (FLDist <= proxLimit && FRDist <= proxLimit) {
                        telemetry.addLine("FL&FR")
                        if (yAxisPower < 0.0) {
                            yAxisPower = 0.0
                        }
                    }else if (FLDist <= (proxLimit) && FRDist >= (proxLimit)) { //Front left prox limit
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                        xAxisPower = -0.5
                    }else if (FLDist >= (proxLimit) && FRDist <= (proxLimit)) { //Front right prox limit
                        telemetry.addLine("FL")
                        if (yAxisPower > 0.0) {
                            yAxisPower = 0.0
                        }
                        xAxisPower = 0.5
                    }else if (FLDist <= proxLimit|| FRDist <= proxLimit) {
                        if (yAxisPower > 0.0) {
                            yAxisPower *= 0.5
                        }
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
                //runBlobs(clawAngle)

                claw.position = -gamepad2.right_stick_y.toDouble()
                if (gamepad2.a) {
                    claw.position = 0.35
                }

                // Show the elapsed game time and wheel p ower.
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
        builder2.enableLiveView(true)

       // builderApril.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 2"))
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

        //builderApril.setCameraResolution(Size(1280, 720))

       // builderApril.addProcessor(aprilTag)
        builder2.addProcessor(colorLocator)
        builder2.addProcessor(colorLocator2)


        // Build the Vision Portal, using the above settings.

        visionPortal2 = builder2.build()

       // visionPortal1 = builderApril.build()

        // Disable or re-enable the aprilTag processor at any time.

        visionPortal2?.setProcessorEnabled(colorLocator, true)
        visionPortal2?.setProcessorEnabled(colorLocator2, true)
       // visionPortal1?.setProcessorEnabled(aprilTag, true)
    } // end method initAprilTag()


}
