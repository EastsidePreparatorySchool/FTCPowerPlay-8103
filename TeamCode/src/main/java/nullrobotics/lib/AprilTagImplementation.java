
        /*
         * Copyright (c) 2021 OpenFTC Team
         *
         * Permission is hereby granted, free of charge, to any person obtaining a copy
         * of this software and associated documentation files (the "Software"), to deal
         * in the Software without restriction, including without limitation the rights
         * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
         * copies of the Software, and to permit persons to whom the Software is
         * furnished to do so, subject to the following conditions:
         *
         * The above copyright notice and this permission notice shall be included in all
         * copies or substantial portions of the Software.
         * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
         * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
         * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
         * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
         * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
         * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
         * SOFTWARE.
         */

        package nullrobotics.lib;

        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

        import java.util.ArrayList;

        public class AprilTagImplementation {
            OpenCvCamera camera;
            AprilTagDetectionPipeline aprilTagDetectionPipeline;

            // Lens intrinsics
            // UNITS ARE PIXELS
            // NOTE: this calibration is for the C920 webcam at 800x448.
            // You will need to do your own calibration for other configurations!
            double fx = 578.272;
            double fy = 578.272;
            double cx = 402.145;
            double cy = 221.506;

            // UNITS ARE METERS
            double tagsize = 0.05;

            int numFramesWithoutDetection = 0;

            final float DECIMATION_HIGH = 3;
            final float DECIMATION_LOW = 2;
            final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
            final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

            boolean detectionHasOccured;

            HardwareMap hardwareMap;
            Telemetry telemetry;

            public void init(HardwareMap hwMap, Telemetry tel, OpenCvCamera camerai) {
                hardwareMap = hwMap;
                telemetry = tel;

                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                this.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "CameraFront"), cameraMonitorViewId);
                this.camera = camerai;

                aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

                this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                        camera.setPipeline(aprilTagDetectionPipeline);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                });

                telemetry.setMsTransmissionInterval(50);

            }

            public ArrayList<AprilTagDetection> scan(int timeoutMillis) {
                //We're going to scan until we see something.

                telemetry.addData("Tag Status", "Searching for tags...");
                telemetry.update();

                detectionHasOccured = false;

                long beginTime = System.currentTimeMillis();

                while( System.currentTimeMillis() - beginTime <= timeoutMillis ){

                    // Calling getDetectionsUpdate() will only return an object if there was a new frame
                    // processed since the last time we called it. Otherwise, it will return null. This
                    // enables us to only run logic when there has been a new frame, as opposed to the
                    // getLatestDetections() method which will always return an object.
                    ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

                    // If there's been a new frame...
                    if (detections != null) {
                        telemetry.addData("FPS", camera.getFps());
                        telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                        telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                        // If we don't see any tags
                        if (detections.size() == 0) {
                            numFramesWithoutDetection++;

                            // If we haven't seen a tag for a few frames, lower the decimation
                            // so we can hopefully pick one up if we're e.g. far back
                            if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                                aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                            }

                            //Continue the loop

                        }
                        // We do see tags!
                        else {
                            numFramesWithoutDetection = 0;

                            // If the target is within 1 meter, turn on high decimation to
                            // increase the frame rate
                            if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                                aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                            }

    //                        for (AprilTagDetection detection : detections) {
    //                            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    //                            telemetry.addLine(String.format("Translation X: %.2f m", detection.pose.x ));
    //                            telemetry.addLine(String.format("Translation Y: %.2f m", detection.pose.y ));
    //                            telemetry.addLine(String.format("Translation Z: %.2f m", detection.pose.z ));
    //                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
    //                            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
    //                            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    //                        }

                            return detections;

                        }

                    }

                }

                return new ArrayList<AprilTagDetection>();

            }

            //If we are SURE only one tag will be in frame.
//            public AprilTagDetection scanSingle() {
//                return this.scan().get(0);
//            }

            public void addDetectionToTelemetry(AprilTagDetection detection) {
                telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                telemetry.addLine(String.format("Translation X: %.2f m", detection.pose.x ));
                telemetry.addLine(String.format("Translation Y: %.2f m", detection.pose.y ));
                telemetry.addLine(String.format("Translation Z: %.2f m", detection.pose.z ));
                telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
            }

        }