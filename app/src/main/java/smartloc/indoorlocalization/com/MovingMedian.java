package smartloc.indoorlocalization.com;

import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;

/**
 * Created by spider on 7/10/17.
 */

public class MovingMedian {
    LinkedList<Integer> queue;
    int size;
    double median;

    /** Initialize your data structure here. */
    public MovingMedian(int size) {
        this.queue = new LinkedList<Integer>();
        this.size = size;
    }

    public double next(int val) {
        if(queue.size()<this.size){
            queue.offer(val);
        }else{
            int head = queue.poll();
            queue.offer(val);
        }

        LinkedList<Integer> Q = queue;
        Collections.sort(Q, new Comparator<Integer>() {
            @Override
            public int compare(Integer integer, Integer t1) {
                return integer - t1;
            }
        });

        if (Q.size() % 2 == 0) // Even
        {
            median = (Q.get((Q.size()/2)) + Q.get((Q.size()/2 - 1)))/2;
        }
        else{
            median = Q.get((Q.size()-1)/2);
        }

        return median;
    }
}
