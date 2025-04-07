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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
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
import kotlin.math.sin
import kotlin.math.tan

@Autonomous(name = "#Auto Red")
class Auto2R : LinearOpMode() {

    private val leftRotate: DcMotorEx? = null
    private val rightRotate: DcMotorEx? = null
    private val rotate: Servo? = null
    private val left: Servo? = null
    private val right: Servo? = null


    /*
     * The variable to store our instance of the AprilTag processor.
     */


    private var aprilTag: AprilTagProcessor? = null
    private var colorLocator: ColorBlobLocatorProcessor? = null
    private var colorLocator2: ColorBlobLocatorProcessor? = null

    /**
     * The variable to store our instance of the vision portal.
     */

    private var visionPortal: VisionPortal? = null
    private var visionPortal2: VisionPortal? = null
    private lateinit var claw: Servo
    private lateinit var clawDrive: CRServo
    private lateinit var linkArmL: Servo
    private lateinit var linkArmR: Servo
    private lateinit var lift: DcMotorEx
    var linearVelX: Double = 0.0
    var linearVelY: Double = 0.0
    var angularVel: Double = 0.0
    private var startX: Double = 0.0
    private var startY: Double = 0.0
    private var startYaw: Double = 0.0
    var positionX: Double = 0.0
    var positionY: Double = 0.0
    var par0Pos: Double = 0.0
    var par1Pos: Double = 0.0
    var perpPos: Double = 0.0

    private lateinit var distanceFrontLeft: Rev2mDistanceSensor
    private lateinit var distanceFrontRight: Rev2mDistanceSensor
    private lateinit var distanceBackLeft: Rev2mDistanceSensor
    private lateinit var distanceBackRight: Rev2mDistanceSensor

    //Tag Alignment
    var tagBearing = 0.0
    var tagRange = 0.0
    var tagYaw = 0.0
    var tagY = 0.0
    var tagX = 0.0
    var botX = 0.0
    var botY = 0.0


    @Throws(InterruptedException::class)
    override fun runOpMode(){
        clawDrive = hardwareMap.get(CRServo::class.java, "clawDrive")
        clawDrive.direction = DcMotorSimple.Direction.FORWARD
        linkArmL = hardwareMap.get(Servo::class.java, "linkArmL")
        linkArmR = hardwareMap.get(Servo::class.java, "linkArmR")
        claw = hardwareMap.get(Servo::class.java, "claw")
        claw.scaleRange(0.02, .375)
        lift = hardwareMap.get(DcMotorEx::class.java, "lift")
        lift.targetPosition = lift.currentPosition
        lift.mode = DcMotor.RunMode.RUN_TO_POSITION
        lift.direction = DcMotorSimple.Direction.FORWARD
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.power = 0.75
        val liftStart = lift.currentPosition
        lift.targetPosition = -10
        linkArmR.direction = Servo.Direction.FORWARD
        linkArmL.direction = Servo.Direction.REVERSE
        linkArmR.scaleRange(0.0, 0.75)
        linkArmL.scaleRange(0.0, 0.75)
        var liftExt = 0

        distanceFrontLeft = hardwareMap.get(Rev2mDistanceSensor::class.java, "distFL")
        distanceFrontRight = hardwareMap.get(Rev2mDistanceSensor::class.java, "distFR")
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor::class.java, "distBL")
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor::class.java, "distBR")

        var clawAngle: Double
        claw.position = 1.0
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

       // val twoDeadWheelLocalizer = TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870.0)
        /*positionX = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX
        positionY = twoDeadWheelLocalizer.par.getPositionAndVelocity().position.toDouble() - startY
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)*/
        val threeDeadWheelLocalizer = ThreeDeadWheelLocalizer(hardwareMap, 0.00144943115234375)

        par0Pos = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
        par1Pos = threeDeadWheelLocalizer.par0.getPositionAndVelocity().position.toDouble()
        perpPos = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()

        startX = perpPos
        startY = par0Pos
        startYaw = (par0Pos - par1Pos)/10

        positionX = perpPos - startX
        positionY = par0Pos - startY
        initAprilTag()

        var tagBearing = 0.0
        var tagRange = 0.0
        val tagYaw = 0.0
        val tagY = 0.0
        val tagX = 0.0
        val lastHeading = 0.0
        startX = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
        startY = (threeDeadWheelLocalizer.par0.getPositionAndVelocity().position.toDouble()+threeDeadWheelLocalizer.par1.getPositionAndVelocity().position.toDouble())/2
        startYaw = (par0Pos - par1Pos)/10

        var targetX: Double
        var targetY: Double

        var offset = false
        val contingency = false
        waitForStart()
        resetRuntime()
        if (opModeIsActive()) {
            while (opModeIsActive()) { //Main loop
                val FLDist = distanceFrontLeft.getDistance(DistanceUnit.CM)
                val FRDist = distanceFrontRight.getDistance(DistanceUnit.CM)
                val BLDist = distanceBackLeft.getDistance(DistanceUnit.CM)
                val BRDist = distanceBackRight.getDistance(DistanceUnit.CM)
                par0Pos = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
                par1Pos = threeDeadWheelLocalizer.par0.getPositionAndVelocity().position.toDouble()
                perpPos = threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble()
                telemetry.addLine(
                    String.format(
                        "FL:%6.1f, FR:%6.1f, BL:%6.1f, BR:%6.1f",
                        FLDist,
                        FRDist,
                        BLDist,
                        BRDist
                    ))
                //   telemetryAprilTag()


                //driveToPosition(0.0, 10000.0, 0.00, drive, twoDeadWheelLocalizer)

                driveToPosition(1000.0, 0.0, drive, threeDeadWheelLocalizer)
                driveToPosition(0.0, 500.0, drive, threeDeadWheelLocalizer)
                clawDrive.power = 1.0
                claw.position = ((1.toDouble())/2) + 0.5
                driveToPosition(100.0, 0.0, drive, threeDeadWheelLocalizer)
                clawDrive.power = 0.0
                claw.position = ((0.toDouble())/2) + 0.5
                driveToAngle(-90.0, drive, threeDeadWheelLocalizer)
                alignTag(16)
                driveToPosition(0.0, 2000.0, drive, threeDeadWheelLocalizer)
                lift(-9900, liftStart)
                clawDrive.power = -1.0
                sleep(500)
                lift(10, liftStart)
                driveToPosition(0.0, 1000.0, drive, threeDeadWheelLocalizer)
                driveToPosition(-500.0, 0.0, drive, threeDeadWheelLocalizer)
                //Lift

                telemetry.addLine(
                    String.format(
                        "%6.1f lift EXT prev",
                        lift.currentPosition.toDouble()
                    )
                )

                telemetry.addLine(String.format("%6.1f lift EXT", liftExt.toDouble()))




            }
        }

    }
    private fun lift(ext: Int, start: Int) {
        if (ext >= -9900 && ext <= 100) {
            lift.targetPosition = ext - start
        }
        telemetry.addLine(String.format("%6.1f lift EXT", lift.targetPosition.toDouble()))

    }

    //claw alignment function
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
            //blob telemetry
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
    private fun driveToPosition(
        abX: Double,
        abY: Double,
        drive: MecanumDrive,
        //twoDeadWheelLocalizer: TwoDeadWheelLocalizer
        threeDeadWheelLocalizer: ThreeDeadWheelLocalizer
    ) {
        positionY = (perpPos - startX)
        positionX= ((par0Pos + par1Pos)/2 - startY)

        var powerX: Double
        var powerY: Double
        //var powerYaw: Double


        while (abs(positionX-abX) > 100|| abs(positionY-abY) > 100) {
            //Current position
           /* positionX =
                (threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX)/10
            */
            positionY = (perpPos - startX)
            positionX= ((par0Pos + par1Pos)/2 - startY)


            //val powerX: Double = (0.53 * (((abX - positionX) ).pow(1.0 / 11)))
            //val powerY: Double = (0.53 * (((abY - positionY) ).pow(1.0 / 11)))



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


            runTelemetry(drive, threeDeadWheelLocalizer)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(linearVelY, linearVelX), 0.00))
            if (abs(abX-positionX) <100  && abs(abY-positionY) < 100){
                startX = positionX
                startY = positionY
                break
            }
        }
    }
    private fun driveToAngle(
        targYaw: Double,
        drive: MecanumDrive,
        threeDeadWheelLocalizer: ThreeDeadWheelLocalizer,
    ) {
        //Current Yaw
        //yaw = par0-par1/trackwidth
        var yaw = (par0Pos - par1Pos)/10 //- startYaw
        var powerYaw: Double

        while (abs(targYaw-yaw) >=10) {
            yaw = (par0Pos - par1Pos)/10// - startYaw

            //Power curve
            powerYaw = if (abs(targYaw-yaw) >=10) {
                if ((targYaw - yaw) <= 0) {
                    -0.5 * (abs(targYaw - yaw)).pow(1.0 / 3)
                } else {
                    0.5 * (abs(targYaw - yaw)).pow(1.0 / 3)
                }
            }else {
                0.0
            }
            angularVel = powerYaw*5
            telemetry.addLine(String.format("%6.1f Pos Yaw", yaw))
            telemetry.addLine(String.format("%6.1f Targ Yaw", targYaw))
            telemetry.addLine(String.format("%6.1f Power Yaw", powerYaw))
            telemetry.addLine(String.format("%6.1f Angular Vel", angularVel))
            runTelemetry(drive, threeDeadWheelLocalizer)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.00,0.00), angularVel))
        }
        startX = perpPos
        startY = (par0Pos + par1Pos) / 2
    }
    private fun alignTag (id: Int, ) {
        val currentDetections: List<AprilTagDetection> = aprilTag!!.detections

        for (detection in currentDetections) {
            if (detection.id == id) {
                //collect values for localization
                tagBearing = detection.ftcPose.elevation
                tagRange = detection.ftcPose.range
                tagX = currentDetections[0].ftcPose.z
                tagY = currentDetections[0].ftcPose.y
                tagYaw = currentDetections[0].ftcPose.pitch
            }

        }
        if (currentDetections.isEmpty()) {
            tagBearing = 0.0
            tagRange = 0.0
        }

        botY = sin(180 - (90+tagYaw)) *((tan(tagYaw) *tagX) + tagY)
        botX = botY/ tan((180-(90+tagBearing))+tagYaw)
        telemetry.addLine(String.format("%6.1f Bot X, %6.1f Bot Y", botX, botY))
        var abY = -100.0
        while (tagYaw != 0.0) {
            for (detection in currentDetections) {
                if (detection.id == id) {
                    //collect values for localization
                    tagBearing = detection.ftcPose.elevation
                    tagRange = detection.ftcPose.range
                    tagX = currentDetections[0].ftcPose.z
                    tagY = currentDetections[0].ftcPose.y
                    tagYaw = currentDetections[0].ftcPose.pitch
                }
            }
            if ((tagYaw - 0.0) <= 0) {
                angularVel = -0.03 * (abs(tagYaw - 0.0)).pow(1.0 / 3)
            } else {
                angularVel = 0.03 * (abs(tagYaw - 0.0)).pow(1.0 / 3)
            }

        }
        while (botX != -100.0) {
            if (abs(positionY-abY) >=20) {
                //Stops the math from breaking
                if ((positionY - abY) <= 0) {
                    linearVelY = -0.025 * (abs(positionY - abY)).pow(1.0 / 3)
                } else {
                    linearVelY = 0.025 * (abs(positionY - abY)).pow(1.0 / 3)
                }
            }else {
                linearVelY = 0.0
                break
            }
        }
    }
    private fun runTelemetry(drive: MecanumDrive, threeDeadWheelLocalizer: ThreeDeadWheelLocalizer) {
       /* telemetry.addLine(String.format("%6.1f time", runtime))
        telemetry.addLine(String.format("%6.1f Diffx", ((100 - drive.pose.position.x) * -1) / 1000))
        //telemetry.addLine(String.format("%6.1f Pose X", drive.pose.position.x));
        telemetry.addLine(String.format("%6.1f Pose X", threeDeadWheelLocalizer.perp.getPositionAndVelocity().position.toDouble() - startX))
        telemetry.addLine(String.format("%6.1f Pose Y", threeDeadWheelLocalizer.par0.getPositionAndVelocity().position.toDouble() - startY))
        telemetry.addLine(String.format("%6.1f Linear Vel X", linearVelX))
        telemetry.addLine(String.format("%6.1f Linear Vel Y", linearVelY))
        telemetry.addLine(String.format("%6.1f Yaw", (Math.toDegrees((threeDeadWheelLocalizer.perp.robotYawPitchRollAngles.yaw).toDouble()))%360))
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
        FtcDashboard.getInstance().sendTelemetryPacket(packet) */
    }
    private fun initAprilTag() {
        // Create the AprilTag processor.


        val builderApril = VisionPortal.Builder()
        //2nd builder for claw
        val builder2 = VisionPortal.Builder()

        // Set the camera (webcam vs. built-in RC phone camera).

        builder2.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 2"))
        builderApril.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        builderApril.setLiveViewContainerId(0)
        //builder2.setCamera(BuiltinCameraDirection.BACK)



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
        builder2.addProcessor(colorLocator)
        builder2.addProcessor(colorLocator2)
        builderApril.addProcessor(aprilTag)

        // Build the Vision Portal, using the above settings.

        visionPortal = builderApril.build()
     //   visionPortal2 = builder2.build()

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal?.setProcessorEnabled(aprilTag, true)

       // visionPortal2?.setProcessorEnabled(colorLocator, true)
        //visionPortal2?.setProcessorEnabled(colorLocator2, true)
    } // end method initAprilTag()
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
            var boxY = boxfit?.center?.y
            


        }
    }

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
