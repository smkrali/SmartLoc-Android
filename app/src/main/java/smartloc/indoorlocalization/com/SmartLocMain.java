package smartloc.indoorlocalization.com;


import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.SensorManager;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import com.estimote.sdk.Beacon;
import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.EstimoteSDK;
import com.estimote.sdk.MacAddress;
import com.estimote.sdk.Region;
import com.estimote.sdk.cloud.CloudCallback;
import com.estimote.sdk.cloud.EstimoteCloud;
import com.estimote.sdk.cloud.model.BeaconInfo;
import com.estimote.sdk.exception.EstimoteServerException;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.AbstractQueue;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import la.matrix.DenseMatrix;
import la.matrix.Matrix;
import la.vector.DenseVector;
import la.vector.Vector;
import ml.utils.Matlab;

import static ml.utils.Matlab.eye;
import static ml.utils.Matlab.hilb;
import static ml.utils.Matlab.innerProduct;
import static ml.utils.Matlab.inv;
import static ml.utils.Matlab.norm;
import static ml.utils.Matlab.ones;
import static ml.utils.Matlab.sign;
import static ml.utils.Matlab.zeros;
import static ml.utils.Printer.disp;

public class SmartLocMain extends AppCompatActivity {

    // Estimote cloud.
    String EST_APP_ID = "kamran-luminite-gmail-com--ac5";
    String EST_APP_TOKEN = "93f63d5ffa7ef28d7cb3603e3d8bcbab";

    int ForegroundBeaconScanPeriod = 50, BackgroundBeaconScanPeriod = 50;
    int DataNormalizationConstant = 1;
    // Give node co-ordinates, in 2-D, here. TODO: Can modify program to allow 3-D localization, but that needs more Anchors nodes for accuracy.
    int[] AnchorPositionsSetOrNot;
    // These positions are automacitcally set by reading the positions set in the
    Matrix AnchorNodePositions;
    Matrix DistancesSquared;
    int TransmitPowerdBm = -12; // Really doesn't matter in our algorithm. TODO: Please check.

    int PVAL = 1; // how many instances before should a value be chosen to estimate slope of trend in path loss.
    double ADD = 0.002; // TODO: should depend on accelerometer and gyro based later on.
    Matrix SlopeTrendRSSVector;
    double ExponentialAveragePLE1 = 0.8, ExponentialAveragePLE2 = 0.2;
    double AllowedChangeInPLE = 2; // TODO: should depend on accelerometer and gyro based later on.
    Matrix PreviousRSSVectorPLE; // Required by PLE estimator.
    Matrix LatestRSSVectorPLE; // Required by PLE estimator.
    Matrix LatestPLEEstimatesVector;
    Matrix FirstPosEstimate, LastPosEstimate;
    Trilaterate FinalPositionEstimation;

    int MinAnchorNodesRequired = 3; // 3 for 2-D, 4 for 3-D localization. You can chose to be arbitrary.
    int DimensionOfCoordinates = 2; //2D or 3D points
    boolean CanHearFromMinAnchorNodes = false; // TODO: We should be able to hear from MinAnchorNodesRequired nodes, frequently.
    int MaxAllowedTimeBetweenUpdates = 50; // N no. of readings. TODO: We should be able to hear from MinAnchorNodesRequired nodes, frequently.
    double ExponentialAverageL1 = 0.99, ExponentialAverageL2 = 0.92; // TODO: Try this later.
    double ExponentialAverageAlpha = 0.9; // TODO: Should depend on accelerometer movement/linear motion. Should decrease when motion detected, to give latest readings more weight.
    int MovingAverageWindow = 10; // Good to mitigate multipath effects.
    int MovingMedianWindow = 10; // Good to mitigate multipath effects.
    Vector WindowCountBeforeProcessingBegins;
    double MahalanobisDistanceSquaredNewReading; // t^2, save the computations due to square root.
    int MahalanobisDistanceThresh = 10; // TODO: Should change according to the motion sensor readings.
    Matrix IdentityMatrix;
    Matrix LatestDiffRSSVector; // Used for outlier detection, difference between latest RSS vectors/readings.
    Matrix LatestMeanSubtractedRSSVector; // Used for outlier detection.
    Matrix LatestRSSVector; // Used for outlier detection.
    Vector LatestUpdateVector;
    Vector DataInstanceVector;
    Matrix RSSRunningMeanVector;
    Matrix RSSRunningMeanSquaredVector;
    int ShiftToExponentialMeanAfterInstances = MinAnchorNodesRequired * 3 + 1; // For updating covariance matrix.
    Matrix ExponentialMeanVector; // EMA - Exponential Moving Average
    Matrix ExponentialL1L2MeanVector; // EMA - Exponential Moving Average, with ExponentialAverageL1 and ExponentialAverageL2
    Matrix CovarianceMatrixInverse;
    MovingAverage[] RSSQueuesMovingAverage;
    MovingMedian[] RSSQueuesMovingMedian;

    public static final String TAG = "Basic Network Demo";
    public static final int MY_PERMISSIONS_REQUEST_READ_COARSE_LOCATION = 1;
    private UUID FOBO_PROXIMITY_UUID = UUID.fromString("B9407F30-F5F8-466E-AFF9-25556B57FE6D");
    private Region FOBOBEACONS;

    private BeaconManager beaconManager;
    HashMap<String, Integer> beaconIDMap;
    int beaconIDMapSize, beaconIDMapSizeWhenLoaded; // beaconIDMapSizeWhenLoaded is created to make sure only currently known nodes are used for localization. Can change later.

    TextView BasicTVDetectedNodeInformation;

    private SensorManager sensorManager;
    private SensorsMonitor monitorTask;
    private double stepChangeRate;
    private double stepPrevChangeRate;
    private double stepPrevChange;
    private static final double AverageHumanStrideMeters = 0.762; // meters

    public void startBeaconDetectionService(){
        beaconManager = new BeaconManager(SmartLocMain.this);
        beaconManager.setForegroundScanPeriod(ForegroundBeaconScanPeriod, 0);
        beaconManager.setBackgroundScanPeriod(BackgroundBeaconScanPeriod, 0);
        beaconManager.setRangingListener(new BeaconManager.RangingListener() {
            @Override
            public void onBeaconsDiscovered(Region beaconRegion, List<Beacon> list) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Vector LatestUpdate = new DenseVector(new double[beaconIDMapSizeWhenLoaded]);
                        Matrix ExponentialMeanVectorOld = ExponentialMeanVector;
                        Matrix LatestRSSVectorOld = LatestRSSVector;
                        if (list.size() > 0) {
                            //Check how many beacons were scanned:
                            Log.d("Beacons Scanned: ", Integer.toString(list.size()));

                            for (int i = 0; i < list.size(); i++) {
                                //String beacMajor = Integer.toString(list.get(i).getMajor());
                                //String beacMinor = Integer.toString(list.get(i).getMinor());
                                String beacMAC = list.get(i).getMacAddress().toString();
                                if (!beaconIDMap.containsKey(beacMAC)) {
                                    beaconIDMap.put(beacMAC, beaconIDMapSize + 1);
                                    beaconIDMapSize = beaconIDMapSize + 1;
                                }

                                if (!beaconIDMap.isEmpty() && beaconIDMap.get(beacMAC) <= beaconIDMapSizeWhenLoaded) {
                                    //String beacRSS = Integer.toString(list.get(i).getRssi());
                                    //Log.d("Beacons ID: ", beacMAC);

                                    // Set Anchor position if not set already.
                                    if (AnchorPositionsSetOrNot[beaconIDMap.get(beacMAC) - 1] == 0){
                                        setBeaconName(list.get(i).getMacAddress());
                                    }

                                    LatestUpdate.set(beaconIDMap.get(beacMAC) - 1, 1);
                                    double MMFiltered = RSSQueuesMovingMedian[beaconIDMap.get(beacMAC) - 1].next(list.get(i).getRssi() / DataNormalizationConstant);
                                    double MAFiltered = RSSQueuesMovingAverage[beaconIDMap.get(beacMAC) - 1].next(list.get(i).getRssi() / DataNormalizationConstant);

                                    double m = VectorMean(WindowCountBeforeProcessingBegins);
                                    if (beaconIDMapSizeWhenLoaded > 0 && m > MovingAverageWindow) { // There should be some nodes at least, before we compute complex things.
                                        double k = VectorMean(DataInstanceVector);
                                        LatestRSSVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, ((MMFiltered + MAFiltered) / 2) / DataNormalizationConstant);
                                        if (k < beaconIDMapSizeWhenLoaded) { // i.e there are some beacons which were never received.
                                            RSSRunningMeanVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, ((MMFiltered + MAFiltered) / 2) / DataNormalizationConstant); // Initial value of running mean
                                            ExponentialMeanVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, ((MMFiltered + MAFiltered) / 2) / DataNormalizationConstant); // Initial value of EMA mean
                                            //RSSRunningMeanSquaredVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, ((MMFiltered+MAFiltered)/2*((MMFiltered+MAFiltered)/2;
                                            RSSRunningMeanSquaredVector = LatestRSSVector.mtimes(LatestRSSVector.transpose());
                                        } else {
                                            // double k = DataInstanceVector.get(beaconIDMap.get(beacMAC) - 1); //TODO: Get exact value of k or Should be max value? We are keeping k = mean of all k's right now.
                                            RSSRunningMeanVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, (k * RSSRunningMeanVector.getEntry(beaconIDMap.get(beacMAC) - 1, 0) + ((MMFiltered + MAFiltered) / 2) / DataNormalizationConstant) / (k + 1)); // Initial value of running mean
                                            //ExponentialMeanVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, ExponentialAverageL2 * ExponentialMeanVector.getEntry(beaconIDMap.get(beacMAC) - 1, 0) + (1 - ExponentialAverageL1) * ((MMFiltered+MAFiltered)/2);
                                            ExponentialMeanVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, ExponentialAverageAlpha * ExponentialMeanVector.getEntry(beaconIDMap.get(beacMAC) - 1, 0) + (1 - ExponentialAverageAlpha) * ((MMFiltered + MAFiltered) / 2) / DataNormalizationConstant); // Initial value of EMA mean
                                            //RSSRunningMeanSquaredVector.setEntry(beaconIDMap.get(beacMAC) - 1, 0, (k * RSSRunningMeanSquaredVector.getEntry(beaconIDMap.get(beacMAC) - 1, 0) + ((MMFiltered+MAFiltered)/2)*((MMFiltered+MAFiltered)/2)) / (k + 1));
                                            RSSRunningMeanSquaredVector = ((RSSRunningMeanSquaredVector.times(k)).plus(LatestRSSVector.mtimes(LatestRSSVector.transpose()))).times(1 / (k + 1));
                                        }
                                        // Increment data instances, i.e. "k", for both regular instances (DataInstanceVector).
                                        DataInstanceVector.set(beaconIDMap.get(beacMAC) - 1, DataInstanceVector.get(beaconIDMap.get(beacMAC) - 1) + 1);
                                    }
                                    // as well as moving window filters (WindowCountBeforeProcessingBegins)
                                    WindowCountBeforeProcessingBegins.set(beaconIDMap.get(beacMAC) - 1, WindowCountBeforeProcessingBegins.get(beaconIDMap.get(beacMAC) - 1) + 1);

                                    // Display on Toast. Notification blob on screen.
                                    //Toast.makeText(SmartLocMain.this, "I found beacon with ID: " + Integer.toString(beaconIDMap.get(beacMAC)) + "," + beacRSS, Toast.LENGTH_SHORT).show();
                                }
                            }

                        }

                        // Necessary condition, otherwise functions will be called on null objects.
                        if (beaconIDMapSizeWhenLoaded > 0) {

                            // Check if all information is recent. TODO make sure it works for arbitrary nodes.
                            double sumRecentInfoCheck = 0; // If it remains zero, it means everything recent.

                            for (int i = 0; i < LatestUpdateVector.getDim(); i++) {
                                if (LatestUpdate.get(i) == 1)
                                    LatestUpdateVector.set(i, 0); // if it was just updated, set to 0.
                                else
                                    LatestUpdateVector.set(i, LatestUpdateVector.get(i) + 1); // increment by one if you don't hear from that node in this scan period.

                                sumRecentInfoCheck += LatestUpdateVector.get(i);
                            }

                            // Check if Anchor Node positions are all loaded and good to go. TODO make sure it works for arbitrary nodes.
                            long sumAnchorNodesPosCheck = 0;
                            for(int i : AnchorPositionsSetOrNot) {
                                sumAnchorNodesPosCheck += i;
                            }

                            String ToastText = "";
                            double m = VectorMean(WindowCountBeforeProcessingBegins);
                            double k = VectorMean(DataInstanceVector);
                            // Do all the following Mahalanobis distance/Pathloss estimation/Localization related and other calculations, if minimum requirements are met.
                            if (sumRecentInfoCheck == 0 && sumAnchorNodesPosCheck >= MinAnchorNodesRequired && beaconIDMapSizeWhenLoaded >= MinAnchorNodesRequired && m >= MovingAverageWindow && beaconIDMapSizeWhenLoaded > 0 && k >= MinAnchorNodesRequired * 3) { // Enough readings in buffer.

                                // Calculating Mahalanobis distance of new RSS vector.
                                MahalanobisDistanceSquaredNewReading = java.lang.Math.sqrt(java.lang.Math.abs(innerProduct(LatestRSSVector.minus(ExponentialMeanVectorOld), CovarianceMatrixInverse.mtimes(LatestRSSVector.minus(ExponentialMeanVectorOld)))));
                                //Calculating Covariance Matrix.
                                if (k < ShiftToExponentialMeanAfterInstances) { // i.e. Enough readings.
                                    // Use regular covariance update.
                                    CovarianceMatrixInverse = inv(RSSRunningMeanSquaredVector.minus(RSSRunningMeanVector.mtimes(RSSRunningMeanVector.transpose())));
                                    //Log.d("Beacons Scanned: ", "Covariance-Done-1");
                                } else if (k >= ShiftToExponentialMeanAfterInstances) { // Use efficient, EMA based, Covariance Inverse update to be used directly in Mahalanobis Distance calculation.
                                    //Taking absolute of Mahalanobis Distance just to be on safe side. Initial miss-estimated covariance matrix might lead to negative output of following multiplication.
                                    Matrix CMI_A = ((LatestRSSVector.minus(ExponentialMeanVectorOld)).mtimes(LatestRSSVector.minus(ExponentialMeanVectorOld).transpose())).mtimes(CovarianceMatrixInverse);
                                    //Log.d("Beacons Scanned: ", "Covariance-Done-2");
                                    double CMI_B = ((k - 1) / ExponentialAverageAlpha) + innerProduct(LatestRSSVector.minus(ExponentialMeanVectorOld), CovarianceMatrixInverse.mtimes(LatestRSSVector.minus(ExponentialMeanVectorOld)));
                                    Matrix CMI_C = IdentityMatrix.minus(CMI_A.times(1 / CMI_B));
                                    CovarianceMatrixInverse = (CovarianceMatrixInverse.times(k / (ExponentialAverageAlpha * (k - 1)))).mtimes(CMI_C);
                                    //Log.d("Beacons Scanned: ", Integer.toString(CovarianceMatrixInverse.getRowDimension()) + " " + Integer.toString(CovarianceMatrixInverse.getColumnDimension()));
                                    //Log.d("Beacons k: ", Double.toString(k) + "," + Double.toString(CMI_B));
                                }

                                // Path loss estimation: PLE
                                if (java.lang.Math.max(m, k) >= java.lang.Math.max(MovingAverageWindow, beaconIDMapSizeWhenLoaded * 3)) {
                                    PreviousRSSVectorPLE = LatestRSSVectorOld;
                                    LatestRSSVectorPLE = LatestRSSVector;
                                    LatestDiffRSSVector = LatestRSSVector.minus(LatestRSSVectorOld);
                                    SlopeTrendRSSVector = sign(LatestDiffRSSVector);
                                    if (MahalanobisDistanceSquaredNewReading > MahalanobisDistanceThresh) { // I.e. Outlier.
                                        LatestRSSVectorPLE = PreviousRSSVectorPLE.plus((Matlab.abs(LatestDiffRSSVector.times(ADD / AllowedChangeInPLE))).times(SlopeTrendRSSVector));
                                        FirstPosEstimate = ApproxPositionEstimate(AnchorNodePositions, LatestRSSVectorPLE);
                                        for (int i = 0; i < LatestPLEEstimatesVector.getRowDimension(); i++) {
                                            LatestPLEEstimatesVector.setEntry(i, 0, ExponentialAveragePLE1 * LatestPLEEstimatesVector.getEntry(i, 0) + ExponentialAveragePLE2 * (LatestRSSVectorPLE.getEntry(i, 0) - TransmitPowerdBm) / (10 * java.lang.Math.log10(1 / norm((AnchorNodePositions.getRowMatrix(i)).minus(FirstPosEstimate.transpose())))));
                                            DistancesSquared.setEntry(i, 0, java.lang.Math.pow(1 / (java.lang.Math.pow(10, (LatestRSSVectorPLE.getEntry(i, 0) - TransmitPowerdBm) / (10 * LatestPLEEstimatesVector.getEntry(i, 0)))), 2));
                                            //DistancesSquared.setEntry(i, 0, java.lang.Math.pow(norm((AnchorNodePositions.getRowMatrix(i)).minus(FirstPosEstimate.transpose())), 2));
                                        }
                                    } else {
                                        FirstPosEstimate = ApproxPositionEstimate(AnchorNodePositions, LatestRSSVectorPLE);
                                        for (int i = 0; i < LatestPLEEstimatesVector.getRowDimension(); i++) {
                                            LatestPLEEstimatesVector.setEntry(i, 0, (LatestRSSVectorPLE.getEntry(i, 0) - TransmitPowerdBm) / (10 * java.lang.Math.log10(1 / norm((AnchorNodePositions.getRowMatrix(i)).minus(FirstPosEstimate.transpose())))));
                                            DistancesSquared.setEntry(i, 0, java.lang.Math.pow(1 / (java.lang.Math.pow(10, (LatestRSSVectorPLE.getEntry(i, 0) - TransmitPowerdBm) / (10 * LatestPLEEstimatesVector.getEntry(i, 0)))), 2));
                                            //DistancesSquared.setEntry(i, 0, java.lang.Math.pow(norm((AnchorNodePositions.getRowMatrix(i)).minus(FirstPosEstimate.transpose())), 2));
                                        }
                                    }
                                    PreviousRSSVectorPLE = LatestRSSVectorPLE;

                                    // Check if node positions correctly set, before messing up Trilateration.
                                    /*
                                    Log.d("Beacons", Integer.toString(AnchorPositionsSetOrNot.length));
                                    Log.d("Beacons", Integer.toString(AnchorNodePositions.getRowDimension()) + " " + Integer.toString(AnchorNodePositions.getColumnDimension()));
                                    ToastText += "\nAnchorNodePositions \n";
                                    for (int i = 0; i < AnchorNodePositions.getRowDimension(); i++) {
                                        for (int j = 0; j < AnchorNodePositions.getColumnDimension(); j++) {
                                            ToastText += Double.toString(AnchorNodePositions.getEntry(i, j)) + " ";
                                        }
                                        ToastText += "\n";
                                    }
                                    Log.d("Beacons", ToastText);
                                    */

                                    // Set to the latest known Anchor Node positions.
                                    FinalPositionEstimation.setAnchorPositions(AnchorNodePositions, AnchorNodePositions.getColumnDimension());
                                    // Trilateration. Final Localization. Adding reference node's coordinates or origin (usually 0,0) in position matrix, as all other positions are relative to reference node position in global co-ordinate system.
                                    LastPosEstimate = (FinalPositionEstimation.getTargetPosition(DistancesSquared)); //.plus((AnchorNodePositions.getRowMatrix(0)).transpose());
                                }
                                // Displaying stuff. Print.
                                ToastText = "ID/LastScanned/Latest/RunMean/EMAMean/MD: " + Double.toString(RoundUp(MahalanobisDistanceSquaredNewReading, 2)) + "\n";
                                for (int i = 0; i < LatestRSSVector.getRowDimension(); i++) {
                                    ToastText += Integer.toString(i) + "/" + Double.toString(LatestUpdateVector.get(i)) + "/" + Double.toString(RoundUp(LatestRSSVector.getEntry(i, 0), 3)) + "/" + Double.toString(RoundUp(RSSRunningMeanVector.getEntry(i, 0), 3)) + "/" +
                                            Double.toString(RoundUp(ExponentialMeanVector.getEntry(i, 0), 3)); // + "/" + Double.toString(RoundUp(CovarianceMatrixInverse.getRowDimension() * CovarianceMatrixInverse.getColumnDimension(),3));
                                    if (i < LatestRSSVector.getRowDimension() - 1)
                                        ToastText += "\n";
                                }
                                ToastText += "\nCovarianceMatrixInverse \n";
                                for (int i = 0; i < CovarianceMatrixInverse.getRowDimension(); i++) {
                                    for (int j = 0; j < CovarianceMatrixInverse.getColumnDimension(); j++) {
                                        ToastText += Double.toString(RoundUp(CovarianceMatrixInverse.getEntry(i, j), 3)) + " ";
                                    }
                                    ToastText += "\n";
                                }

                                ToastText += "\n Beacons MAC Addresses \n";
                                for (String key : beaconIDMap.keySet()) {
                                    ToastText += Integer.toString(beaconIDMap.get(key)-1) + "," + key + "\n";
                                }

                                ToastText += "\n Basic Target Position Estimate\n";
                                for (int i = 0; i < FirstPosEstimate.getRowDimension(); i++) {
                                    if (i == 0) ToastText += "X = " + Double.toString(RoundUp(FirstPosEstimate.getEntry(i, 0),3)) + " Meters\n";
                                    else if (i == 1) ToastText += "Y = " + Double.toString(RoundUp(FirstPosEstimate.getEntry(i, 0),3)) + " Meters\n";
                                    else ToastText += "Z = " + Double.toString(RoundUp(FirstPosEstimate.getEntry(i, 0),3)) + " Meters\n";
                                }

                                ToastText += "\n Target Position \n";
                                for (int i = 0; i < LastPosEstimate.getRowDimension(); i++) {
                                    if (i == 0) ToastText += "X = " + Double.toString(RoundUp(LastPosEstimate.getEntry(i, 0),3)) + " Meters\n";
                                    else if (i == 1) ToastText += "Y = " + Double.toString(RoundUp(LastPosEstimate.getEntry(i, 0),3)) + " Meters\n";
                                    else ToastText += "Z = " + Double.toString(RoundUp(LastPosEstimate.getEntry(i, 0),3)) + " Meters\n";
                                }
                            } else {
                                ToastText = "Not enough data yet!\nOR\nNodes out of range!\nOR\nMinimum Anchors not detected!\n";
                            }

                            // StepChangeRate Detection.
                            stepChangeRate = (stepPrevChangeRate + (1000*AverageHumanStrideMeters*(monitorTask.getStepsTaken() - stepPrevChange))/(8.0*ForegroundBeaconScanPeriod))/2;
                            stepPrevChange = monitorTask.getStepsTaken(); stepPrevChangeRate = stepChangeRate;
                            BasicTVDetectedNodeInformation.setText(ToastText + "\nSteps Change Rate (m/s): " + Double.toString(RoundUp(stepChangeRate,3)));

                            if (stepChangeRate > 0.1)
                                ExponentialAverageAlpha = 0.1;
                            else
                                ExponentialAverageAlpha = 0.9;

                            //Toast.makeText(SmartLocMain.this, ToastText,Toast.LENGTH_SHORT).show();
                        }
                    }
                });
            }
        });
    }

    protected void onStart() {
        super.onStart();
        monitorTask.start();
        // Should be invoked in #onStart.
        beaconManager.connect(new BeaconManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                beaconManager.startRanging(FOBOBEACONS);
                Log.d("Beacons Scanned: ", "StartedRanging");
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        monitorTask.start();
        // TODO: As app is running, we should update the discovered beacons (MAP saved by onDestroy and onPause). And reset all the matrices and variables related to localization. This will be very useful in real life.
        beaconManager.connect(new BeaconManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                beaconManager.startRanging(FOBOBEACONS);
                Log.d("Beacons Scanned: ", "StartedRanging");
            }
        });

    }

    @Override
    protected void onPause() {
        beaconManager.stopRanging(FOBOBEACONS);
        monitorTask.stop();

        File f = new File(getFilesDir(),"beaconIDMap.mp");
        try {
            FileOutputStream fileOutputStream  = new FileOutputStream(f,false);
            ObjectOutputStream objectOutputStream= new ObjectOutputStream(fileOutputStream);
            try {
                objectOutputStream.writeObject(beaconIDMap);
                Log.d("IOException: ", "HashMap file written.");
            } finally {
                objectOutputStream.close();
                fileOutputStream.close();
            }
        } catch (IOException ex) {
            Log.d("IOException: ", "HashMap file not written.");
            ex.printStackTrace();
        }

        super.onPause();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        beaconManager.disconnect();
        monitorTask.stop();

        File f = new File(getFilesDir(),"beaconIDMap.mp");
        try {
            FileOutputStream fileOutputStream  = new FileOutputStream(f,false);
            ObjectOutputStream objectOutputStream= new ObjectOutputStream(fileOutputStream);
            try {
                objectOutputStream.writeObject(beaconIDMap);
                Log.d("IOWrite: ", "HashMap file written.");
            } finally {
                objectOutputStream.close();
                fileOutputStream.close();
            }
        } catch (IOException ex) {
            Log.d("IOException: ", "HashMap file not written.");
            ex.printStackTrace();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_smart_loc_main);

        BasicTVDetectedNodeInformation = (TextView) findViewById(R.id.sample_text);

        if (ContextCompat.checkSelfPermission(SmartLocMain.this,Manifest.permission.ACCESS_COARSE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            // Should we show an explanation?
            if (ActivityCompat.shouldShowRequestPermissionRationale(SmartLocMain.this,
                    Manifest.permission.ACCESS_COARSE_LOCATION)) {

            } else {

                ActivityCompat.requestPermissions(SmartLocMain.this,
                        new String[]{Manifest.permission.ACCESS_COARSE_LOCATION},
                        MY_PERMISSIONS_REQUEST_READ_COARSE_LOCATION); // 1 represents my permission to read contacts.
            }
        }

        //SystemRequirementsChecker.checkWithDefaultDialogs(this);
        File f = new File(getFilesDir(),"beaconIDMap.mp");
        if (f.isFile() && f.canRead()) {
            try {
                FileInputStream fileInputStream  = new FileInputStream(f);
                ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
                try {
                    beaconIDMap = (HashMap) objectInputStream.readObject();
                    beaconIDMapSize = beaconIDMap.size();
                    Log.d("IORead:beaconIDMapSize=", Integer.toString(beaconIDMapSize));
                } catch (ClassNotFoundException e) {
                    e.printStackTrace();
                } finally {
                    objectInputStream.close();
                    fileInputStream.close();
                }
            } catch (IOException ex) {
                Log.d("IOException: ", "HashMap file not opened.");
                ex.printStackTrace();
            }

            if (beaconIDMapSize > 0) {
                // Java language spec initialized all of these to zero automatically.
                AnchorNodePositions = new DenseMatrix(new double[beaconIDMapSize][DimensionOfCoordinates]);
                FinalPositionEstimation = new Trilaterate(); // new Trilaterate(AnchorNodePositions, AnchorNodePositions.getColumnDimension());
                AnchorPositionsSetOrNot = new int[beaconIDMapSize];
                FirstPosEstimate = zeros(DimensionOfCoordinates, 1);
                LastPosEstimate = zeros(DimensionOfCoordinates, 1);
                LatestPLEEstimatesVector = ones(beaconIDMapSize, 1).times(2.0); // Regular free space path loss initially.
                PreviousRSSVectorPLE = zeros(beaconIDMapSize, 1);
                LatestRSSVectorPLE = zeros(beaconIDMapSize, 1);
                DistancesSquared = zeros(beaconIDMapSize, 1);
                SlopeTrendRSSVector = zeros(beaconIDMapSize, 1);
                DataInstanceVector = new DenseVector(new double[beaconIDMapSize]);
                CovarianceMatrixInverse = zeros(beaconIDMapSize);
                IdentityMatrix = eye(beaconIDMapSize);
                LatestMeanSubtractedRSSVector = zeros(beaconIDMapSize, 1);
                LatestDiffRSSVector = zeros(beaconIDMapSize, 1);
                RSSRunningMeanVector = zeros(beaconIDMapSize, 1);
                RSSRunningMeanSquaredVector = zeros(beaconIDMapSize, 1);
                ExponentialMeanVector = zeros(beaconIDMapSize, 1);
                LatestRSSVector = zeros(beaconIDMapSize, 1); // Initializing a vector for latest RSS readings from already known nodes.
                RSSQueuesMovingAverage = new MovingAverage[beaconIDMapSize];
                RSSQueuesMovingMedian = new MovingMedian[beaconIDMapSize];
                WindowCountBeforeProcessingBegins = new DenseVector(new double[beaconIDMapSize]);
                for (int i = 0; i < beaconIDMapSize; i++) {
                    RSSQueuesMovingAverage[i] = new MovingAverage(MovingAverageWindow);
                    RSSQueuesMovingMedian[i] = new MovingMedian(MovingMedianWindow);
                }
                LatestUpdateVector = new DenseVector(new double[beaconIDMapSize]); // This vector will keep track how many "beacons" passed since we last heard from this node.
            }
        } else {
            beaconIDMap = new HashMap<String, Integer>(); // Otherwise create a new hash.
            beaconIDMapSize = 0;
        }

        beaconIDMapSizeWhenLoaded = beaconIDMapSize;

        FOBOBEACONS = new Region("SmartLocRegion",FOBO_PROXIMITY_UUID, null, null);
        EstimoteSDK.initialize(SmartLocMain.this, EST_APP_ID, EST_APP_TOKEN);
        sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
        monitorTask = new SensorsMonitor(50, sensorManager, this); // Check every 50 milliseconds how many steps taken.
        monitorTask.start();
        startBeaconDetectionService();
        // Example of a call to a native method
        // TextView tv = (TextView) findViewById(R.id.sample_text);
        // tv.setText(stringFromJNI());
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_READ_COARSE_LOCATION: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // permission was granted, yay! Do the location task now.

                } else { // permission denied, boo! Disable the functionality that depends on this permission.
                }
                return;
            }
            // other 'case' lines to check for other permissions this app might request
        }
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    public static double RoundUp(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    public static double VectorMean(Vector Data) {

        int sum = 0;
        for (int d = 0; d < Data.getDim(); d++) sum += Data.get(d);
        return (double)sum/Data.getDim();
    }

    public static Matrix ApproxPositionEstimate (Matrix AnchorPositions, Matrix RSSVector) {
        Matrix NumeratorApproxPosEstimate = zeros(AnchorPositions.getColumnDimension(), 1);
        double DenominatorApproxPosEstimate = 0; // While estimating positions by RSS based weighted averaging.
        Matrix PostionEstimate;

        for (int i = 0; i < AnchorPositions.getRowDimension(); i++){
            NumeratorApproxPosEstimate = NumeratorApproxPosEstimate.plus(((AnchorPositions.getRowMatrix(i)).transpose()).times(java.lang.Math.pow(10,(RSSVector.getEntry(i, 0))/10)));
            DenominatorApproxPosEstimate = DenominatorApproxPosEstimate + java.lang.Math.pow(10,(RSSVector.getEntry(i, 0))/10);
        }

        PostionEstimate = NumeratorApproxPosEstimate.times(1/DenominatorApproxPosEstimate);

        /*
        for (int i = 0; i < PostionEstimate.getRowDimension(); i++) {
            if (i == 0) Log.d("Beacons: ", "X = " + Double.toString(RoundUp(PostionEstimate.getEntry(i, 0),3)) + " Meters\n");
            else if (i == 1) Log.d("Beacons: ", "Y = " + Double.toString(RoundUp(PostionEstimate.getEntry(i, 0),3)) + " Meters\n");
            else Log.d("Beacons: ", "Z = " + Double.toString(RoundUp(PostionEstimate.getEntry(i, 0),3)) + " Meters\n");
        }
        */

        return PostionEstimate;
    }

    public void setBeaconName(MacAddress beacMACaddress){
        EstimoteCloud.getInstance().fetchBeaconDetails(beacMACaddress, new CloudCallback<BeaconInfo>() {
            @Override
            public void success(BeaconInfo beaconInfo) {
                String CurrentBeaconName = String.valueOf(beaconInfo.name);
                String[] parts = CurrentBeaconName.split(",");
                //BasicTVDetectedNodeInformation.setText(CurrentBeaconName);
                if (parts.length == 3) {
                    //Log.d("AnchorPos is set to :", parts[1] + "," + parts[2]);
                    AnchorNodePositions.setEntry(beaconIDMap.get(beacMACaddress.toString()) - 1, 0, Double.parseDouble(parts[1]));
                    AnchorNodePositions.setEntry(beaconIDMap.get(beacMACaddress.toString()) - 1, 1, Double.parseDouble(parts[2]));
                    AnchorPositionsSetOrNot[beaconIDMap.get(beacMACaddress.toString()) - 1] = 1;
                }
                Log.d("AnchorPos===========", CurrentBeaconName);
            }

            @Override
            public void failure(EstimoteServerException e) {
                Log.e("AnchorPos===========", "BEACON INFO ERROR: " + e);
            }
        });
    }

}

