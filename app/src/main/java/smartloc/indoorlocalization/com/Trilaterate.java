package smartloc.indoorlocalization.com;

import android.util.Log;

import la.matrix.Matrix;
import ml.utils.Matlab;

/**
 * Created by spider on 7/10/17.
 */

public class Trilaterate {
    Matrix AnchorNodePositions;
    Matrix TargetPosition;
    Matrix DistancesFromAnchorsSquared;
    int Dimensions;
    // AnchorNodePositions = no.of nodes x dimensions vector, each row contains x,y,z coordinate of i-th node.
    // DistancesFromAnchorsSquared = no.of nodes x 1 vector containing distance squared information from nodes to target.
    // Dimensions = 2 for 2D, 3 for 3D localization. Minimum anchor nodes required = Dim + 1;

    public Trilaterate() {

    }

    /*
    public Trilaterate(Matrix AnchorNodesPos, int Dim){
        this.AnchorNodePositions = AnchorNodesPos;
        this.Dimensions = Dim;
    }
    */

    public void setAnchorPositions(Matrix AnchorNodesPos, int Dim){
        this.AnchorNodePositions = AnchorNodesPos;
        this.Dimensions = Dim;
        /*
        String ToastText = "";
        for (int i = 0; i < this.AnchorNodePositions.getRowDimension(); i++) {
            for (int j = 0; j < this.AnchorNodePositions.getColumnDimension(); j++) {
                ToastText += Double.toString(this.AnchorNodePositions.getEntry(i, j)) + " ";
            }
            ToastText += "\n";
        }
        Log.d("Beacons", ToastText);
        */

    }

    public Matrix getTargetPosition(Matrix DistancesSquared){
        this.DistancesFromAnchorsSquared = DistancesSquared;
        /*
        String ToastText = "";
        for (int i = 0; i < this.DistancesFromAnchorsSquared.getRowDimension(); i++) {
            for (int j = 0; j < this.DistancesFromAnchorsSquared.getColumnDimension(); j++) {
                ToastText += Double.toString(this.DistancesFromAnchorsSquared.getEntry(i, j)) + " ";
            }
            ToastText += "\n";
        }
        Log.d("Beacons", ToastText);
        */
        int[] p1 = new int[this.AnchorNodePositions.getRowDimension()-1];
        int[] p2 = new int[this.AnchorNodePositions.getRowDimension()-1];
        int[] c1 = new int[this.AnchorNodePositions.getColumnDimension()];

        for (int i = 0; i < this.AnchorNodePositions.getColumnDimension(); i++){
            c1[i] = i;
        }

        Matrix Y = Matlab.zeros(this.AnchorNodePositions.getRowDimension()-1,1); // a row vector

        for (int i = 0; i < this.AnchorNodePositions.getRowDimension()-1; i++){

            p1[i] = i; p2[i] = i+1; // sub-coordinates of AnchorPositions matrix for making H matrix.

            if (this.Dimensions == 2){
                Y.setEntry(i, 0, DistancesFromAnchorsSquared.getEntry(i, 0) - DistancesFromAnchorsSquared.getEntry(i+1, 0)
                        + this.AnchorNodePositions.getEntry(i+1, 0)*this.AnchorNodePositions.getEntry(i+1, 0) +
                        this.AnchorNodePositions.getEntry(i+1, 1)*this.AnchorNodePositions.getEntry(i+1, 1) -
                        this.AnchorNodePositions.getEntry(i, 0)*this.AnchorNodePositions.getEntry(i, 0) -
                        this.AnchorNodePositions.getEntry(i, 1)*this.AnchorNodePositions.getEntry(i, 1));
            }
            else{
                Y.setEntry(i, 0, DistancesFromAnchorsSquared.getEntry(i, 0) - DistancesFromAnchorsSquared.getEntry(i+1, 0)
                        + this.AnchorNodePositions.getEntry(i+1, 0)*this.AnchorNodePositions.getEntry(i+1, 0) +
                        this.AnchorNodePositions.getEntry(i+1, 1)*this.AnchorNodePositions.getEntry(i+1, 1) +
                        this.AnchorNodePositions.getEntry(i+1, 2)*this.AnchorNodePositions.getEntry(i+1, 2) -
                        this.AnchorNodePositions.getEntry(i, 0)*this.AnchorNodePositions.getEntry(i, 0) -
                        this.AnchorNodePositions.getEntry(i, 1)*this.AnchorNodePositions.getEntry(i, 1) -
                        this.AnchorNodePositions.getEntry(i, 2)*this.AnchorNodePositions.getEntry(i, 2));
            }
        }

        Matrix H = (this.AnchorNodePositions.getSubMatrix(p2,c1)).minus(this.AnchorNodePositions.getSubMatrix(p1,c1));

        TargetPosition = ((Matlab.inv((H.transpose()).mtimes(H))).mtimes((H.transpose()).mtimes(Y))).times(0.5);

        return TargetPosition;
    }
}
